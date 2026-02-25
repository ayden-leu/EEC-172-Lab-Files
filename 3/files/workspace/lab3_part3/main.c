//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Lab 3 Part 4
// Application Overview - This application allows two CC3200 boards to send
//                        messages to each other and display them using an
//                        OLED screen.
//
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>

#include "hw_types.h"
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "hw_memmap.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "utils.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "systick.h"
#include "uart.h"
#include "uart_if.h"
#include "spi.h"
#include "gpio.h"
#include "gpio_if.h"
#include "pin_mux_config.h"

//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************

#define APPLICATION_VERSION        "1.4.0"
#define FOREVER                    1

#define SPI_IF_BIT_RATE   1000000

// SysTick Timing
#define CPU_HZ        80000000UL
#define SYSTICK_MASK  0x00FFFFFFUL  // old value:  0x00FFFFFFUL
#define TICKS_PER_US  (CPU_HZ/1000000UL)
#define TICKS_TO_US(t) ((uint32_t)((t)/TICKS_PER_US))

// Signal related
#define IR_EDGES_TARGET   50
#define IR_BUF_MAX        64
#define BIT_0_LENGTH_THRES   1700      // in micro s
#define LEADER_BURST_LENGTH  9000-1000 // in micro s, found in tv remote data formats: The NEC Code

// Remote button related
#define TV_CODE       0b1111000001110000  // 1006 signal in binary
#define BUTTON_1      0b0001000001101111
#define BUTTON_2      0b0101000000101111
#define BUTTON_3      0b0011000001001111
#define BUTTON_4      0b0000100001110111
#define BUTTON_5      0b0100100000110111
#define BUTTON_6      0b0010100001010111
#define BUTTON_7      0b0001100001100111
#define BUTTON_8      0b0101100000100111
#define BUTTON_9      0b0011100001000111
#define BUTTON_0      0b0100010000111011

#define BUTTON_LAST   0b0110010000011011
#define BUTTON_MUTE   0b0111100000000111

//*****************************************************************************
//                      Global Variables
//*****************************************************************************

typedef struct PinConfig {
    uint32_t base;
    uint32_t pin;
    uint32_t base_interrupt;
} PinConfig;

static const PinConfig IR = {
    .base = GPIOA3_BASE,          // PIN_18
    .pin = GPIO_PIN_4,            // from pin_mux_config
    .base_interrupt = INT_GPIOA3
};

// Signal related
static volatile uint32_t time_elapsed = 0;       // in micro s
volatile uint32_t time_btwn_ir_edges[IR_BUF_MAX];
volatile uint32_t last_edge_systick = 0;
volatile int      ir_edge_count = 0;
volatile int      ir_frame_ready = 0;

static volatile int msg_received_fully = 0;


//*****************************************************************************
//                      Function Definitions
//*****************************************************************************

extern void (* const g_pfnVectors[])(void);

static inline uint32_t systick_delta(uint32_t prev, uint32_t now){
    return (prev - now) & SYSTICK_MASK;
}

static uint8_t determine_bit_from_time(uint32_t time_gap){
    if(time_gap > BIT_0_LENGTH_THRES){
        return 1;   // long => 1
    }
    return 0;       // short => 0
}

static void report_binary_list(uint8_t* bits, int bit_length){
    int i;
    for(i = 0; i < bit_length; i++) {
        Report("%d", bits[i]);

        // space every 8 bits
        if      (i == 7)   Report(" ");
        else if(i == 15)  Report(" ");
        else if(i == 23)  Report(" ");
    }
}

static void handle_remote_button_pressed(uint16_t key){
    // Print out which button was pressed
    switch (key) {
        case BUTTON_1: Message("1\n\r"); break;
        case BUTTON_2: Message("2\n\r"); break;
        case BUTTON_3: Message("3\n\r"); break;
        case BUTTON_4: Message("4\n\r"); break;
        case BUTTON_5: Message("5\n\r"); break;
        case BUTTON_6: Message("6\n\r"); break;
        case BUTTON_7: Message("7\n\r"); break;
        case BUTTON_8: Message("8\n\r"); break;
        case BUTTON_9: Message("9\n\r"); break;
        case BUTTON_0: Message("0\n\r"); break;

        case BUTTON_LAST: Message("LAST\n\r"); break;
        case BUTTON_MUTE: Message("MUTE\n\r"); break;

        default: Message("Unknown\n\r"); break;
    }
}

//-----------------------------------------------------------------------------
//                      Initialization Functions
//-----------------------------------------------------------------------------
static void IRIntHandler(void);

static void SysTickInit(void){
    MAP_SysTickPeriodSet(SYSTICK_MASK);

    MAP_SysTickEnable();
    last_edge_systick = SysTickValueGet();
}

static void IRIntInit(void){
    MAP_IntPrioritySet(IR.base_interrupt, INT_PRIORITY_LVL_0);
    MAP_GPIOIntRegister(IR.base, IRIntHandler);

    MAP_GPIOIntTypeSet(IR.base, IR.pin, GPIO_RISING_EDGE);

    MAP_GPIOIntClear(IR.base, IR.pin);
    MAP_GPIOIntEnable(IR.base, IR.pin);
    MAP_IntEnable(IR.base_interrupt);
}

static void UARTA0Init(void){
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);

    MAP_UARTConfigSetExpClk(UARTA0_BASE, 80000000, 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    MAP_UARTEnable(UARTA0_BASE);
}

static void SPIInit(){
    // Reset SPI
    MAP_SPIReset(GSPI_BASE);
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    // Configure SPI interface
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                    SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    // Enable the SPI module clock
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);

    // Enable SPI for communication
    MAP_SPIEnable(GSPI_BASE);
}

//-----------------------------------------------------------------------------
//                      Interrupts
//-----------------------------------------------------------------------------

static void IRIntHandler(void){
    // Get the interrupt status
    unsigned long status = MAP_GPIOIntStatus(IR.base, true);
    MAP_GPIOIntClear(IR.base, status);

    // Continue if the IR caused the interrupt
    if((status & IR.pin) == 0) return;

    // Prepare to get data for this data frame
    uint32_t current_edge_systick = SysTickValueGet();
    uint32_t time_btwn_edges = systick_delta(last_edge_systick, current_edge_systick);

//    Report("Last Edge SysTick: %d\n\r", last_edge_systick);
//    Report("Current Edge SysTick: %d\n\r", current_edge_systick);
    last_edge_systick = current_edge_systick;

    uint32_t time_btwn_edges_us = TICKS_TO_US(time_btwn_edges);
    time_elapsed += time_btwn_edges_us;

    // Stop if we already got the full data frame
    if(ir_frame_ready) return;

    // Save times between edges
    if(ir_edge_count < IR_EDGES_TARGET && ir_edge_count < IR_BUF_MAX) {
        time_btwn_ir_edges[ir_edge_count] = time_btwn_edges_us;
        ir_edge_count++;
    }

    // Tell the program that we got the full data frame
    if(ir_edge_count >= IR_EDGES_TARGET) {
        ir_frame_ready = 1;
    }
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    //MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//!    main function demonstrates the use of the timers to generate
//! periodic interrupts.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************

static volatile uint32_t times_overflowed = 0;

int main(void){

    // Initialize board configurations
    BoardInit();

    // Configure pins
    PinMuxConfig();

    // initialize and clear terminal
    InitTerm();
    ClearTerm();

    Message("Initializing SPI...\n\r");
    SPIInit();
    Message("Initializing IR...\n\r");
    IRIntInit();
    Message("Initializing SysTick...\n\r");
    SysTickInit();
    Message("Initializing UARTA0...\n\r");
    UARTA0Init();

    Message("\t\t****************************************************\n\r");
    Message("\t\t*        Ready to receive TV remote signals        *\n\r");
    Message("\t\t****************************************************\n\r");
    Message("\n\r");


    while(FOREVER) {
        if(ir_frame_ready){
            // Duplicate edge timings for this data frame
            uint32_t local_time_btwn_ir_edges[IR_EDGES_TARGET];
            int i;
            for(i = 0; i < IR_EDGES_TARGET; i++)
                local_time_btwn_ir_edges[i] = time_btwn_ir_edges[i];

            // Let program receive data while this is going on
            ir_frame_ready = 0;
            ir_edge_count = 0;
            for(i=0; i<IR_BUF_MAX; i++){
                time_btwn_ir_edges[i] = 0;
            }

            // Find leader bit (~9ms)
            int start = -1;
            for(i = 0; i < IR_EDGES_TARGET; i++) {
                if(local_time_btwn_ir_edges[i] > LEADER_BURST_LENGTH) {
                    start = i + 1;
                    break;
                }
            }
            if(start < 0) continue;  // no valid leader

            // Decode 32 bit-cells
            uint8_t bits[32];
            int j;
            int bit_index = 0;
            for(j = start; j < IR_EDGES_TARGET && bit_index < 32; j++) {
                bits[bit_index++] = determine_bit_from_time(local_time_btwn_ir_edges[j]);
            }

            if(bit_index < 32) continue;  // incomplete frame

            // Print full 32-bit binary representation
            Report("All Bits: ");
            report_binary_list(bits, 32);
            Report("\n\r");

            // Extract TV and button code bits (bits 16-31)
            uint8_t tv_code_bits[16];
            uint8_t button_code_bits[16];
            for(i = 0; i < 16; i++){
                tv_code_bits[i] = bits[i];
                button_code_bits[i] = bits[16 + i];
            }

            // Print the TV Code
            Report("TV Code: ");
            report_binary_list(bits, 16);
            Report("\n\r");

            // Print the data sent with the signal
            Report("Data: ");
            report_binary_list(button_code_bits, 16);
            Report("\n\r");

            // Convert list of bits to an integer
            uint16_t tv_code = 0;
            uint16_t button_code = 0;
            for(i = 0; i < 16; i++){
                tv_code = (tv_code << 1) | tv_code_bits[i];
                button_code = (button_code << 1) | button_code_bits[i];
            }

            // Verify the leader code matches our group code
            if(tv_code != TV_CODE){
                continue;
            }

            Message("Pressed button: ");
            handle_remote_button_pressed(button_code);
            Message("\n\r");
        }
    }
}
