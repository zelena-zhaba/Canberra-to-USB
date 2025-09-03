// Canberra-to-USB: top-level program that runs the 'input' PIO state machine
// and streams samples read by the PIO (ADC_COUNT bits) to the host over
// USB CDC as binary 16-bit little-endian words. DMA with hardware wrap
// writes PIO FIFO words into a circular RAM buffer; the CPU reads complete
// segments and forwards them to USB.

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


// On-board LED (Pico typically uses GPIO 25)
#define LED_PIN 25



#include "input.pio.h"

// DMA ring buffer configuration (words) and buffers - file scope so IRQ handler can access
// We break the ring into several segments; the DMA will write one segment at a time
// and the IRQ will advance the write head to the next segment, forming a circular buffer.
#define DMA_SEG_WORDS 512
#define DMA_RING_SEGS 8
#define DMA_RING_WORDS (DMA_SEG_WORDS * DMA_RING_SEGS)
static volatile uint32_t dma_ring[DMA_RING_WORDS];
// ring indices (word offsets into dma_ring)
static volatile uint32_t ring_read_index = 0; // index of next word to be consumed
static volatile uint32_t ring_write_head = 0; // start index of next segment to be written
static volatile bool ring_overflow = false;
static volatile bool overflow_led_triggered = false;
static int dma_chan = -1;


// DMA IRQ handler: called when a DMA transfer of one segment completes.
// Advance the software write head, detect overflow, and re-arm DMA to the
// next segment so transfers continue indefinitely.
static void dma_handler(void) {
	if (dma_chan >= 0) dma_channel_acknowledge_irq0(dma_chan);

	// Advance write head by one segment
	uint32_t next_head = ring_write_head + DMA_SEG_WORDS;
	if (next_head >= DMA_RING_WORDS) next_head -= DMA_RING_WORDS;
	ring_write_head = next_head;

	// If the producer caught up to the consumer, drop the oldest segment
	// to make room and mark overflow so main loop can report it once.
	if (ring_write_head == ring_read_index) {
		uint32_t next_read = ring_read_index + DMA_SEG_WORDS;
		if (next_read >= DMA_RING_WORDS) next_read -= DMA_RING_WORDS;
		ring_read_index = next_read;
		ring_overflow = true;
	}

	// Re-arm DMA to write into the next segment
	dma_channel_set_write_addr(dma_chan, (void *)&dma_ring[ring_write_head], true);
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

	// Print boot banner
	printf("Canberra-to-USB firmware booted (PIO DMA mode)\r\n");

	// Initialize on-board LED pin
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	gpio_put(LED_PIN, 0);

	// Select PIO and claim a state machine
	PIO pio = pio0;
	int sm = pio_claim_unused_sm(pio, true);
	if (sm < 0) {
		printf("ERROR: no available PIO state machine\r\n");
		fflush(stdout);
		while (1) tight_loop_contents();
	}

	// Load and initialise PIO program (this configures the input pins)
	uint offset = pio_add_program(pio, &input_program);
	input_program_init(pio, sm, offset, BASE_PIN, READY_PIN);

	// Precompute mask for ADC_COUNT bits
	const uint32_t mask = (1u << ADC_COUNT) - 1u;

	// Claim DMA channel and configure
	dma_chan = dma_claim_unused_channel(true);
	dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
	channel_config_set_read_increment(&cfg, false); // read from FIFO reg
	channel_config_set_write_increment(&cfg, true); // write through buffer
	channel_config_set_dreq(&cfg, DREQ_PIO0_RX0 + sm);

	// Enable hardware wrap on the write address so the DMA automatically
	// wraps inside the `dma_ring` buffer. We choose size_bits so the ring
	// length in bytes is a power of two: DMA_RING_WORDS * 4 bytes.
	// size_bits = log2(DMA_RING_WORDS * sizeof(uint32_t)).
	// Compute size_bits at compile time by a small static loop (macro-like).
	uint size_bits = 0;
	uint32_t ring_bytes = DMA_RING_WORDS * sizeof(dma_ring[0]);
	while ((1u << size_bits) < ring_bytes) ++size_bits;
	channel_config_set_ring(&cfg, true, size_bits);

	// Configure DMA to start writing into the first ring segment
	dma_channel_configure(dma_chan, &cfg,
		(void *)&dma_ring[0],            // dest (start of ring)
		&pio->rxf[sm],                   // src (PIO RX FIFO for this SM)
		DMA_SEG_WORDS,                   // transfer count (one segment)
		true);                           // start immediately

	// Hook up IRQ0 for DMA channel completion
	dma_channel_set_irq0_enabled(dma_chan, true);
	irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	// (heartbeat removed)

	// Main loop: process buffers when ready
	while (1) {
	// main loop

		// Service TinyUSB stack
		tud_task();

	// Use the ISR-maintained write head (points at the start of the next segment
	// DMA will write). The ISR advances this and re-arms DMA per segment.

	// Process any fully-written whole segments from ring_read_index up to
	// ring_write_head. Only process whole segments to avoid reading partially
	// updated data.
		while (ring_read_index != ring_write_head) {
			// If not enough words remain to form a whole segment, break.
			uint32_t available = (ring_write_head + DMA_RING_WORDS - ring_read_index) % DMA_RING_WORDS;
			if (available < DMA_SEG_WORDS) break;

			// Prepare a binary chunk: 16-bit little-endian samples (2 bytes per sample).
			static uint8_t outbuf[DMA_SEG_WORDS * 2];
			size_t out_len = DMA_SEG_WORDS * 2;
			for (uint32_t i = 0; i < DMA_SEG_WORDS; ++i) {
				uint32_t idx = ring_read_index + i;
				if (idx >= DMA_RING_WORDS) idx -= DMA_RING_WORDS;
				uint32_t raw = dma_ring[idx];
				// Extract ADC_COUNT bits from the MSB-aligned word coming from PIO
				uint16_t sample = (uint16_t)((raw >> (32 - ADC_COUNT)) & mask);
				outbuf[i*2 + 0] = (uint8_t)(sample & 0xFF);
				outbuf[i*2 + 1] = (uint8_t)(sample >> 8);
			}

			// Write to USB CDC if connected
			if (tud_cdc_connected() && out_len > 0) {
				tud_cdc_write(outbuf, out_len);
				tud_cdc_write_flush();
			}

			// advance read index by one segment
			ring_read_index += DMA_SEG_WORDS;
			if (ring_read_index >= DMA_RING_WORDS) ring_read_index -= DMA_RING_WORDS;

			// Report overflow once if it happened and latch the on-board LED solid
			if (ring_overflow) {
				if (!overflow_led_triggered) {
					// Turn the LED on and remember we've triggered it; never clear so it
					// remains lit until device reset/reboot.
					gpio_put(LED_PIN, 1);
					overflow_led_triggered = true;
					printf("--- RING OVERFLOW, oldest data dropped ---\r\n");
					fflush(stdout);
				}
				ring_overflow = false;
			}
		}

		// small sleep to yield to interrupts and avoid busy loop
		sleep_ms(1);
	}

	return 0;
}
