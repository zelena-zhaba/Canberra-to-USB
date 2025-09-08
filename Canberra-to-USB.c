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
#define READY_PIN 17

#include "input.pio.h"

// DMA buffer size (words) and buffers - file scope so IRQ handler can access
#define DMA_BUF_WORDS 4096
static volatile uint32_t dma_buf0[DMA_BUF_WORDS];
static volatile uint32_t dma_buf1[DMA_BUF_WORDS];
static volatile bool buf0_ready = false;
static volatile bool buf1_ready = false;
static volatile bool buf0_active = true; // DMA currently writing to buf0
static int dma_chan = -1;

// DMA IRQ handler: mark buffer ready and switch DMA write address
static void dma_handler(void) {
	if (dma_chan >= 0) dma_hw->ints0 = 1u << dma_chan;

	if (buf0_active) {
		buf0_ready = true;
		dma_channel_set_write_addr(dma_chan, (void *)dma_buf1, true);
		buf0_active = false;
	} else {
		buf1_ready = true;
		dma_channel_set_write_addr(dma_chan, (void *)dma_buf0, true);
		buf0_active = true;
	}
}

int main(void) {
	// Enable level shifter: set pin 22 high
	gpio_init(22);
	gpio_set_dir(22, GPIO_OUT);
	gpio_put(22, 1);

	// Initialize stdio (USB CDC if enabled in CMake)
	stdio_init_all();
	setvbuf(stdout, NULL, _IONBF, 0);
	sleep_ms(200);

	// Select PIO and claim a state machine
	PIO pio = pio0;
	int sm = pio_claim_unused_sm(pio, true);
	if (sm < 0) {
		// no available state machine; cannot proceed - halt silently
		while (1) tight_loop_contents();
	}

	// Load and initialise PIO program (this configures the input pins)
	uint offset = pio_add_program(pio, &input_program);
	input_program_init(pio, sm, offset, BASE_PIN, READY_PIN);

	// Precompute left-aligned mask for ADC_COUNT bits (shift_right = false -> bits are MSB-aligned)
	const uint32_t mask = ((1u << ADC_COUNT) - 1u) << (32 - ADC_COUNT);

	// Claim DMA channel and configure
	dma_chan = dma_claim_unused_channel(true);
	dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
	channel_config_set_read_increment(&cfg, false); // read from FIFO reg
	channel_config_set_write_increment(&cfg, true); // write through buffer
	channel_config_set_dreq(&cfg, DREQ_PIO0_RX0 + sm);

	// Configure DMA to start writing into buf0
	dma_channel_configure(dma_chan, &cfg,
		(void *)dma_buf0,                // dest
		&pio->rxf[sm],                   // src (PIO RX FIFO for this SM)
		DMA_BUF_WORDS,                   // transfer count
		true);                           // start immediately

	// Hook up IRQ0 for DMA channel completion
	dma_channel_set_irq0_enabled(dma_chan, true);
	irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	// Main loop: process buffers when ready and print numeric samples only
	while (1) {
		if (buf0_ready) {
			for (int i = 0; i < DMA_BUF_WORDS; ++i) {
				uint32_t raw = dma_buf0[i];
				uint16_t sample = (uint16_t)((raw & mask) >> (32 - ADC_COUNT));
				// print numeric sample only (no letters)
				printf("%u\r\n", sample);
			}
			buf0_ready = false;
		}

		if (buf1_ready) {
			for (int i = 0; i < DMA_BUF_WORDS; ++i) {
				uint32_t raw = dma_buf1[i];
				uint16_t sample = (uint16_t)((raw & mask) >> (32 - ADC_COUNT));
				// print numeric sample only (no letters)
				printf("%u\r\n", sample);
			}
			buf1_ready = false;
		}

		// small sleep to yield to interrupts and avoid busy loop
		sleep_ms(1);
	}

	return 0;
}
