// Canberra-to-USB: top-level program that runs the 'input' PIO state machine
// and streams 14-bit samples from the PIO RX FIFO to stdout (USB CDC) as
// human-readable lines for debugging. This version uses DMA double-buffering
// to transfer PIO RX FIFO words into RAM, minimizing CPU load.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
// Minimal always-on version: force ENC low, run PIO, poll RX FIFO.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#define ADC_COUNT 14
#define BASE_PIN 2
#define ENC_PIN 21

#include "input.pio.h"

int main(void) {
    stdio_init_all();
    sleep_ms(150); 

    // Level shifter enable 
    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);

    // ENC active-low: drive low permanently
    gpio_init(ENC_PIN);
    gpio_set_dir(ENC_PIN, GPIO_OUT);
    gpio_put(ENC_PIN, 0);

    // Load PIO program and claim a state machine
    PIO pio = pio0;
    int sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &input_program);
    input_program_init(pio, sm, offset, BASE_PIN);
    // Program's init enabled SM already; ensure it's running
    pio_sm_set_enabled(pio, sm, true);

    const uint16_t mask = (1u << ADC_COUNT) - 1u;
    while (true) {
        // Block until a 32-bit word (containing one 14-bit sample) is available
        while (pio_sm_is_rx_fifo_empty(pio, sm)) tight_loop_contents();
        uint32_t raw = pio->rxf[sm];
        uint16_t sample = (uint16_t)(raw & mask); // right-aligned by PIO
        // Invert since lines are inverted at source
        sample ^= mask;
        printf("%u\r\n", sample);
    }
        return 0;
    }
