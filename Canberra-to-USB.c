// Canberra-to-USB: top-level program that runs the 'input' PIO state machine
// and streams 14-bit samples from the PIO RX FIFO to stdout (USB CDC) as
// human-readable lines for debugging. This version uses DMA double-buffering
// to transfer PIO RX FIFO words into RAM, minimizing CPU load.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "tusb.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// Local constants - must be visible before including the generated PIO header
#define ADC_COUNT 14
#define BASE_PIN 2
#define ENC_PIN 21   // ADC Enable (active-low)

#include "input.pio.h"

// DMA buffer configuration: split the total buffer into smaller segments
// so the CPU can process shorter chunks (lower latency) using a circular buffer.
#define DMA_BUF_WORDS 4096
#define SEGMENT_WORDS 512
#define SEGMENT_COUNT (DMA_BUF_WORDS / SEGMENT_WORDS)

static volatile uint32_t dma_buf[SEGMENT_COUNT][SEGMENT_WORDS];
static volatile bool seg_ready[SEGMENT_COUNT];   // true when segment has been filled by DMA
static volatile int dma_chan = -1;
static volatile int write_idx = 0; // segment index DMA is currently (or just finished) writing to
static volatile int read_idx = 0;  // segment index next to be consumed by CPU
static volatile bool overflow = false;

// DMA IRQ handler: mark current segment ready, advance write pointer and
// switch DMA write address to the next segment. If the next segment is
// still marked ready, we have an overrun — drop the oldest segment to free space.
static void dma_handler(void) {
	if (dma_chan >= 0) dma_hw->ints0 = 1u << dma_chan;

	// mark the segment we just finished as ready
	seg_ready[write_idx] = true;

	int next = (write_idx + 1) % SEGMENT_COUNT;

	// if next segment hasn't been consumed yet, we have an overrun
	if (seg_ready[next]) {
		overflow = true;
		// drop the oldest segment to make room
		read_idx = (read_idx + 1) % SEGMENT_COUNT;
		seg_ready[next] = false; // consider it free for writing
	}

	// advance write index and point DMA at the next segment
	write_idx = next;
	dma_channel_set_write_addr(dma_chan, (void *)dma_buf[write_idx], true);
}

// Helper to (re)enable or disable ADC + SM (file-scope function)
static void set_acq_enabled(PIO pio, int sm, bool on) {
	if (on) {
		// Clear and restart SM before enabling, and clear ring state
		pio_sm_clear_fifos(pio, sm);
		pio_sm_restart(pio, sm);
		for (int i = 0; i < SEGMENT_COUNT; ++i) seg_ready[i] = false;
		write_idx = read_idx = 0;
		// Enable ADC (active-low ENC)
		gpio_put(ENC_PIN, 0);
		sleep_ms(1); // brief settle
		pio_sm_set_enabled(pio, sm, true);
		printf("ADC ENABLED (ENC=0), acquisition started\r\n");
	} else {
		pio_sm_set_enabled(pio, sm, false);
		gpio_put(ENC_PIN, 1); // disable ADC
		printf("ADC DISABLED (ENC=1), acquisition stopped\r\n");
	}
}

int main(void) {
	// Enable level shifter: set pin 22 high
	gpio_init(22);
	gpio_set_dir(22, GPIO_OUT);
	gpio_put(22, 1);

	// Configure ENC (active low): start disabled (high)
	gpio_init(ENC_PIN);
	gpio_set_dir(ENC_PIN, GPIO_OUT);
	gpio_put(ENC_PIN, 1); // ADC off

	// Initialize stdio (USB CDC if enabled in CMake)
	stdio_init_all();
	setvbuf(stdout, NULL, _IONBF, 0);
	sleep_ms(200);

	// Initialize ENC pin (active low): default disabled (high)
	gpio_init(ENC_PIN);
	gpio_set_dir(ENC_PIN, GPIO_OUT);
	gpio_put(ENC_PIN, 1);

	// Select PIO and claim a state machine
	PIO pio = pio0;
	int sm = pio_claim_unused_sm(pio, true);
	if (sm < 0) {
		// no available state machine; cannot proceed - halt silently
		while (1) tight_loop_contents();
	}

	// Load and initialise PIO program (this configures the input pins)
	uint offset = pio_add_program(pio, &input_program);
	input_program_init(pio, sm, offset, BASE_PIN);

	// Start with state machine disabled until explicitly enabled via serial
	pio_sm_set_enabled(pio, sm, false);

	// Precompute mask for ADC_COUNT bits. PIO uses autopush of 14 bits.
	const uint32_t mask = (1u << ADC_COUNT) - 1u;
	// Canberra ADC lines are inverted; correct in C (PIO unchanged)
	static bool invert_polarity = false; // invert by default per module polarity

	// Claim DMA channel and configure
	dma_chan = dma_claim_unused_channel(true);
	dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
	channel_config_set_read_increment(&cfg, false); // read from FIFO reg
	channel_config_set_write_increment(&cfg, true); // write through buffer
	channel_config_set_dreq(&cfg, DREQ_PIO0_RX0 + sm);

	// Configure DMA to start writing into segment 0
	dma_channel_configure(dma_chan, &cfg,
		(void *)dma_buf[0],              // dest (first segment)
		&pio->rxf[sm],                   // src (PIO RX FIFO for this SM)
		SEGMENT_WORDS,                   // transfer count per segment
		true);                           // start immediately

	// Hook up IRQ0 for DMA channel completion
	dma_channel_set_irq0_enabled(dma_chan, true);
	irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	printf("Commands: e=enable, d=disable, t=toggle, s=status\r\n");
	bool enabled = false; // current desired state

	// Main loop: process ready segments from the ring buffer and print samples
	while (1) {
		// Handle simple USB CDC commands to control ENC and SM
		// Commands (single characters):
		//   'e'/'E'/'1' -> enable ADC: ENC low, start SM
		//   'd'/'D'/'0' -> disable ADC: ENC high, stop SM
		//   's'/'S'/'?' -> print status
		if (tud_cdc_connected() && tud_cdc_available()) {
			char buf[16];
			uint32_t n = tud_cdc_read(buf, sizeof(buf));
			for (uint32_t i = 0; i < n; ++i) {
				char c = buf[i];
				if (c == 'e' || c == 'E' || c == '1') {
					// Enable: pull ENC low and (re)start the state machine
					gpio_put(ENC_PIN, 0);
					pio_sm_clear_fifos(pio, sm);
					pio_sm_restart(pio, sm);
					pio_sm_set_enabled(pio, sm, true);
					printf("# ENC=LOW (ADC enabled), SM started\r\n");
				} else if (c == 'd' || c == 'D' || c == '0') {
					// Disable: drive ENC high and stop the state machine
					gpio_put(ENC_PIN, 1);
					pio_sm_set_enabled(pio, sm, false);
					printf("# ENC=HIGH (ADC disabled), SM stopped\r\n");
				} else if (c == 's' || c == 'S' || c == '?') {
					bool enc = gpio_get(ENC_PIN);
					bool sm_on = (pio->ctrl & (1u << sm)) != 0; // SM enable bit
					printf("# Status: ENC=%s, SM=%s\r\n", enc ? "HIGH" : "LOW", sm_on ? "ON" : "OFF");
				}
			}
		}
		// Handle serial commands (non-blocking)
		int ch;
		while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
			if (ch == '\r' || ch == '\n') continue;
			switch (ch) {
				case 'e': case 'E':
					if (!enabled) { enabled = true; set_acq_enabled(pio, sm, true); }
					break;
				case 'd': case 'D':
					if (enabled) { enabled = false; set_acq_enabled(pio, sm, false); }
					break;
				case 't': case 'T':
					enabled = !enabled; set_acq_enabled(pio, sm, enabled);
					break;
				case 's': case 'S':
					printf("Status: %s (ENC=%d)\r\n", enabled ? "ENABLED" : "DISABLED", gpio_get(ENC_PIN));
					break;
				default:
					printf("Unknown cmd '%c'\r\n", ch);
			}
		}

		// if the next segment is ready, consume it
		if (seg_ready[read_idx]) {
			// process SEGMENT_WORDS words from dma_buf[read_idx]
			for (int i = 0; i < SEGMENT_WORDS; ++i) {
				uint32_t raw = dma_buf[read_idx][i];
				// PIO is configured to shift left with autopush at 14 bits,
				// so samples are left-aligned in the 32-bit word. Right-align them.
				uint32_t aligned = raw >> (32 - ADC_COUNT);
				uint16_t sample = (uint16_t)(aligned & mask);
				// Correct polarity in C only (PIO unchanged)
				if (invert_polarity) {
					sample ^= (uint16_t)mask;
				}
				// print numeric sample only (no letters)
				printf("%u\r\n", sample);
			}

			// mark consumed and advance read pointer
			seg_ready[read_idx] = false;
			read_idx = (read_idx + 1) % SEGMENT_COUNT;
			continue; // try to consume more without sleeping
		}

	// small sleep to yield to interrupts and avoid busy loop
	sleep_ms(1);
	}

	return 0;
}
