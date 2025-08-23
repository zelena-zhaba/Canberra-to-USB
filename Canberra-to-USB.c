/**
 * Copyright (c) 2024 Zelena Zhaba
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
// Access to dma_hw struct for reading channel write_addr
#include "hardware/structs/dma.h"

// Include the header generated from our PIO file
#include "input.pio.h"

// --- Pin Definitions ---
// The PIO assembler does not export .define symbols into the generated
// header, so provide the numeric pin values here to match `input.pio`.
// These come from the `.define BASE_PIN 2` and `.define READY 17` in
// the PIO source.
#define BASE_PIN 2
#define READY_PIN 17

// --- Configuration ---

// Define the circular buffer size. A power of 2 is most efficient for DMA.
#define BUFFER_SIZE 256
// Calculate the log2 of the buffer size for DMA ring configuration
#define BUFFER_SIZE_LOG2 8 // 2^8 = 256

// --- Global Variables ---

// PIO and State Machine instances
PIO pio = pio0;
uint sm;

// DMA channel
int dma_chan;

// The circular buffer where DMA will write the PIO data
// The 'volatile' keyword is important to prevent the compiler from optimizing
// away reads from this buffer, as it's modified by hardware (DMA).
volatile uint32_t dma_buffer[BUFFER_SIZE];

// Index for the main loop to read from the circular buffer
uint32_t read_index = 0;

// --- Main Program ---

int main() {
    // Initialize standard I/O for printing to the serial monitor
    stdio_init_all();

    // Give the user a moment to connect the serial monitor
    sleep_ms(2000);
    printf("Canberra-to-USB Interface\n");
    printf("Initializing PIO and DMA...\n");

    // --- PIO Setup ---

    // Find a free state machine on our chosen PIO instance (pio0)
    sm = pio_claim_unused_sm(pio, true);
    // Load the 'input' program into the PIO's instruction memory
    uint offset = pio_add_program(pio, &input_program);
    // Initialize the state machine using the helper function from the .pio.h file
    // This configures the GPIOs and sets up the state machine's initial state.
    // We pass the defined pin numbers from the .pio file.
    input_program_init(pio, sm, offset, BASE_PIN, READY_PIN);
    printf("PIO State Machine initialized on SM %d.\n", sm);

    // --- DMA Setup ---

    // Claim an unused DMA channel
    dma_chan = dma_claim_unused_channel(true);
    printf("DMA Channel %d claimed.\n", dma_chan);

    // Get the default configuration for the DMA channel
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    
    // Configure the DMA channel for a 32-bit transfer
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    // The read address is the PIO RX FIFO. Don't increment this address,
    // as we are always reading from the same hardware register.
    channel_config_set_read_increment(&cfg, false);
    // The write address is our circular buffer. Increment this address after each write.
    channel_config_set_write_increment(&cfg, true);
    // Set the DMA Request signal (DREQ) to trigger when the PIO RX FIFO is not empty.
    // This links the DMA channel to our specific state machine's output.
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, false));
    // Configure the write address to wrap around, creating the circular buffer.
    // The third argument is the size of the wrap region in log2 format.
    channel_config_set_ring(&cfg, true, BUFFER_SIZE_LOG2);

    // Apply the configuration to the DMA channel.
    // - Write destination: our dma_buffer
    // - Read source: the PIO RX FIFO for our state machine
    // - Transfer count: a large number, DMA will run continuously
    // - Trigger: false, don't start yet. We'll start it after the PIO SM.
    dma_channel_configure(
        dma_chan,
        &cfg,
        dma_buffer,             // Write address
        &pio->rxf[sm],          // Read address
        -1,                     // Transfer count (run forever)
        false                   // Don't start immediately
    );

    // --- Start Everything ---

    // NOTE: The PIO state machine is already enabled by the init function.
    // We just need to start the DMA.
    dma_channel_start(dma_chan);
    printf("PIO and DMA are running. Waiting for data...\n");

    // --- Main Processing Loop ---

    while (true) {
    // Read the DMA channel's write address from the hardware registers.
    // The SDK doesn't provide a public dma_channel_get_write_addr() in
    // this version, so read the register directly via dma_hw.
    uintptr_t write_addr = (uintptr_t)dma_hw->ch[dma_chan].write_addr;
    // Convert the raw memory address to an index into our buffer.
    uint32_t write_index = (write_addr - (uintptr_t)dma_buffer) / sizeof(dma_buffer[0]);

        // Process all the new data that has arrived since our last check.
        while (read_index != write_index) {
            // Read the data from the buffer
            uint32_t value = dma_buffer[read_index];

            // Process the data (here, we just print it)
            // The value is a 14-bit number from the ADC.
            printf("Received ADC value: 0x%04X (%u)\n", value, value);

            // Move to the next position in the circular buffer
            read_index = (read_index + 1) % BUFFER_SIZE;
        }

        // A small delay to prevent this loop from consuming 100% CPU
        // when no new data is arriving.
        sleep_ms(10);
    }
}

