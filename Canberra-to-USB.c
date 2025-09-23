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

#include "input.pio.h"


void FIFO_to_USB(PIO pio, int sm) {
    // Mask to extract 14-bit sample from 32-bit word
    const uint16_t mask = (1u << ADC_COUNT) - 1u;
    // Block until a 32-bit word (containing one 14-bit sample) is available
    while (pio_sm_is_rx_fifo_empty(pio, sm)) tight_loop_contents();
    uint32_t raw = pio->rxf[sm];
    uint16_t sample = (uint16_t)(raw & mask); // right-aligned by PIO
    // Invert since lines are inverted at source
    sample ^= mask;
    printf("%u\r\n", sample);
}



int main(void) {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(150); 

    // Level shifter enable 
    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);

    // Load PIO program and claim a state machine
    PIO pio = pio0;
    int sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &input_program);
    input_program_init(pio, sm, offset);

    // Main loop: listen to serial input and print data from FIFO to USB
    while (true) {
    //int input = getchar_timeout_us(0);
    //if (input == '1') {
        pio_sm_exec(pio, sm, pio_encode_set(pio_x, 1));
    //}
    //if (input == '0') {
    //    pio_sm_exec(pio, sm, pio_encode_set(pio_x, 0));
    //}
        FIFO_to_USB(pio, sm);
    }


    return 0;
}

