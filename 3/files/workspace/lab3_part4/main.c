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
#include "timer.h"
#include "timer_if.h"
#include "systick.h"
#include "uart.h"
#include "uart_if.h"
#include "spi.h"
#include "gpio.h"
#include "gpio_if.h"
#include "Adafruit_SSD1351.h"
#include "Adafruit_GFX.h"
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


// Text message related
#define MAX_MSG_LENGTH 40
#define CHAR_CYCLE_THRES 5  // in roughly seconds
#define DEFAULT_USERNAME_SENDER  "Waiting..."
#define DEFAULT_USERNAME_MINE    "Default"
#define MAX_USERNAME_LENGTH   16
#define MAX_COMMAND_NAME_LENGTH  16
#define ATTRIBUTE_SEPARATOR  "~"


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
#define BUTTON_DELETE   BUTTON_LAST   // use MUTE as backspace
#define BUTTON_SEND     BUTTON_MUTE   // use LAST as send


// Colors
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

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
    .base_interrupt = INT_GPIOA3   //
};
// Don't forget to modify Adafruit_OLED.c if the GPIO pin configuration for the screen changes.


volatile int systick_count = 0;
volatile int systick_count_prev_key_signal = 0;

// Signal related
static volatile uint32_t time_elapsed = 0;       // in micro s
volatile uint32_t time_btwn_ir_edges[IR_BUF_MAX];
volatile uint32_t last_edge_systick = 0;
volatile int      ir_edge_count = 0;
volatile int      ir_frame_ready = 0;


// Text message related
static char msg_send[MAX_MSG_LENGTH + 1];
static int  msg_send_length = 0;

static char msg_receive[MAX_MSG_LENGTH + 1];
static int  msg_receive_length = 0;

static volatile int msg_received_fully = 0;

static char sender_username[MAX_USERNAME_LENGTH + 1] = DEFAULT_USERNAME_SENDER;
static unsigned long sender_color = CYAN;
static volatile int update_sender = 0;

static char my_username[MAX_USERNAME_LENGTH + 1] = DEFAULT_USERNAME_MINE;
static unsigned long my_color = YELLOW;
static char my_color_str[MAX_USERNAME_LENGTH + 1 ] = "yellow";
static volatile int update_me = 0;


// Remote button related
static uint16_t prev_key_signal = 0;
static uint8_t char_cycle_index = 0;

//*****************************************************************************
//                      Function Definitions
//*****************************************************************************

extern void (* const g_pfnVectors[])(void);

static inline uint32_t systick_delta(uint32_t prev, uint32_t now){
    return (prev - now) & SYSTICK_MASK;
}

// Map numeric keys
static const char *button_to_char_group(uint16_t key){
    switch (key) {
        case BUTTON_1: return "/.,?!";
        case BUTTON_2: return "abc";
        case BUTTON_3: return "def";
        case BUTTON_4: return "ghi";
        case BUTTON_5: return "jkl";
        case BUTTON_6: return "mno";
        case BUTTON_7: return "pqrs";
        case BUTTON_8: return "tuv";
        case BUTTON_9: return "wxyz";
        case BUTTON_0: return " ";
        default: return NULL;
    }
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

static void updateColor(unsigned long* user_color_attr, char* user_color_str, const char *color){
    Report("Updating color to: %s\n\r", color);

    if(strcmp(color, "r") == 0 || strcmp(color, "red") == 0){
        *user_color_attr = RED;
        strcpy(user_color_str, "red");
    }
    else if(strcmp(color, "y") == 0 || strcmp(color, "yellow") == 0){
        *user_color_attr = YELLOW;
        strcpy(user_color_str, "yellow");
    }
    else if(strcmp(color, "g") == 0 || strcmp(color, "green") == 0){
        *user_color_attr = GREEN;
        strcpy(user_color_str, "green");
    }
    else if(strcmp(color, "c") == 0 || strcmp(color, "cyan") == 0){
        *user_color_attr = CYAN;
        strcpy(user_color_str, "cyan");
    }
    else if(strcmp(color, "b") == 0 || strcmp(color, "blue") == 0){
        *user_color_attr = BLUE;
        strcpy(user_color_str, "blue");
    }
    else if(strcmp(color, "m") == 0 || strcmp(color, "magenta") == 0){
        *user_color_attr = MAGENTA;
        strcpy(user_color_str, "magenta");
    }
    else if(strcmp(color, "w") == 0 || strcmp(color, "white") == 0){
        *user_color_attr = WHITE;
        strcpy(user_color_str, "white");
    }
    else if(strcmp(color, "b") == 0 || strcmp(color, "black") == 0){
        *user_color_attr = BLACK;
        strcpy(user_color_str, "black");
    }
}

//-----------------------------------------------------------------------------
//                      Texting Helpers
//-----------------------------------------------------------------------------

static void send_string_UARTA1(const char *str){
    // Put string into UART1 buffer
    while(*str) {
        // Only continue if UART1 isn't busy
        while(MAP_UARTBusy(UARTA1_BASE)) {}

        MAP_UARTCharPut(UARTA1_BASE, *str++);
    }

    // Only continue if UART1 isn't busy
    while(MAP_UARTBusy(UARTA1_BASE)) {}

    // Put string terminator at the end
    MAP_UARTCharPut(UARTA1_BASE, '\0');
}

static void print_sending(void){
    Report("To send: [%s]\r\n", msg_send);
}

static void clear_sending(){
    msg_send_length = 0;
    msg_send[0] = '\0';
}

static void delete_char_from_sending(void){
    if(msg_send_length > 0) {
        msg_send_length--;
        msg_send[msg_send_length] = '\0';
        Report("Deleted character\r\n");
        print_sending();
    }

    systick_count = 0;
}

static char* build_message(){
    char* to_send = (char*)  malloc(
        (MAX_MSG_LENGTH) * sizeof(char)
    );

    strcat(to_send, my_username);
    strcat(to_send, ATTRIBUTE_SEPARATOR);
    strcat(to_send, my_color_str);
    strcat(to_send, ATTRIBUTE_SEPARATOR);
    strcat(to_send, msg_send);

    return to_send;
}

static void send_message(void){
    char* msg_to_send = build_message();

    if(msg_send_length > 0) {
        Report("Sending: %s\r\n", msg_to_send);
        send_string_UARTA1(msg_to_send);

        clear_sending();
    } else {
        Report("No message to send\r\n");
    }

    systick_count = 0;
}

static void handle_remote_button_pressed(uint16_t key){
    // Get potential characters to type based on the remote button pressed
    const char *potential_chars = button_to_char_group(key);
    if(!potential_chars) return;

    // Determine if we should cycle through the list of potential characters
//    Report("                systick_count: %d\n\r", systick_count);
//    Report("systick_count_prev_key_signal: %d\n\r", systick_count_prev_key_signal);
//    Report("                       Result: %d\n\r", systick_count - systick_count_prev_key_signal);
//    Report("             CHAR_CYCLE_THRES: %d\n\r", CHAR_CYCLE_THRES);

    int same_key = (key == prev_key_signal);
//    int within = (time_elapsed - time_prev_key_signal) <= CHAR_CYCLE_THRES;  // old version of check
    int within = (systick_count - systick_count_prev_key_signal) <= CHAR_CYCLE_THRES;

    if(same_key && within && msg_send_length > 0) {
        // Cycle the last character
        int size_potential_chars = 0;
        while(potential_chars[size_potential_chars]){
            size_potential_chars++;
        }

        char_cycle_index = (char_cycle_index + 1) % size_potential_chars;
        msg_send[msg_send_length - 1] = potential_chars[char_cycle_index];
    }
    else{
        // Add a new character
        if(msg_send_length < MAX_MSG_LENGTH) {
            char_cycle_index = 0;
            msg_send[msg_send_length++] = potential_chars[char_cycle_index];
            msg_send[msg_send_length] = '\0';
        }

        systick_count = 0;
    }

    prev_key_signal = key;
    systick_count_prev_key_signal = systick_count;

    print_sending();
//    Message("\n\r");
}

static void run_command(){
//    Message("Command!\n\r");

    // Get command to run
    int i = 0;
    while(msg_send[i] != ' ' && msg_send[i] != '\0'){
        i++;
    }
    char command[MAX_COMMAND_NAME_LENGTH + 1];
    strncpy(command, msg_send + 1, i-1);
    command[i-1] = '\0';

    // Get command parameter
    int j = i+1;
    while(msg_send[j] != ' ' && msg_send[j] != '\0'){
        j++;
    }
    char parameter[MAX_COMMAND_NAME_LENGTH + 1];
    strncpy(parameter, msg_send + 1 + i, j-1);
    parameter[j-i-1] = '\0';

    Report("Running: [%s] [%s]\n\r", command, parameter);

    if(strcmp(command, "c") == 0 || strcmp(command, "color") == 0){
        updateColor(&my_color, my_color_str, parameter);
        update_me = 1;
    }
    else if(strcmp(command, "u") == 0 || strcmp(command, "user") == 0){
        strcpy(my_username, parameter);
        update_me = 1;
    }

    clear_sending();
    systick_count = 0;
}

//-----------------------------------------------------------------------------
//                      OLED Screen Related
//-----------------------------------------------------------------------------

static void drawText(
    int x, int y, const char *str,
    unsigned int fg, unsigned int bg,
    unsigned char size
){
    while(*str) {
        drawChar(x, y, (unsigned char)*str, fg, bg, size);
        x += 6 * size;
        str++;
    }
}

static void updateSenderUsername(){
    fillRect(0, 0, 64, 8, BLACK);
    drawText(0, 0, sender_username, sender_color, BLACK, 1);
    update_sender = 0;
}

static void updateMyUsername(){
    fillRect(0, 70, 64, 8, BLACK);
    drawText(0, 70, my_username, my_color, BLACK, 1);
    update_me = 0;
}

static void drawUI(void){
    fillScreen(BLACK);
    updateSenderUsername();
    drawLine(0, 63, 127, 63, WHITE);
    updateMyUsername();
}

static void drawMessages(void){
    // Top area
    fillRect(0, 12, 127, 8, BLACK);
    drawText(0, 12, msg_receive, WHITE, BLACK, 1);

    // Bottom area
    fillRect(0, 82, 127, 8, BLACK);
    drawText(0, 82, msg_send, WHITE, BLACK, 1);
}

//-----------------------------------------------------------------------------
//                      Initialization Functions
//-----------------------------------------------------------------------------
static void IRIntHandler(void);
static void UARTA1_RXIntHandler(void);
static void SysTickHandler(void);

static void SysTickInit(void){
    MAP_SysTickPeriodSet(SYSTICK_MASK);

    MAP_SysTickIntRegister(SysTickHandler);
    MAP_SysTickIntEnable();

    MAP_SysTickEnable();
    last_edge_systick = SysTickValueGet();
}

static void IRIntInit(void){
    MAP_IntPrioritySet(IR.base_interrupt, INT_PRIORITY_LVL_0);
    MAP_GPIOIntRegister(IR.base, IRIntHandler);

    MAP_GPIOIntTypeSet(IR.base, IR.pin, GPIO_FALLING_EDGE);

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

static void UARTA1Init(void){
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);

    MAP_UARTConfigSetExpClk(UARTA1_BASE, 80000000, 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    MAP_UARTEnable(UARTA1_BASE);

    // Enable RX interrupts
    MAP_UARTIntRegister(UARTA1_BASE, UARTA1_RXIntHandler);
    MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX | UART_INT_RT);

    MAP_IntEnable(INT_UARTA1);
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

static void OLEDInit(){
    Adafruit_Init();
    drawUI();
}

//-----------------------------------------------------------------------------
//                      Interrupts
//-----------------------------------------------------------------------------
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_count++;
}

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


static void UARTA1_RXIntHandler(void){
    // Get interrupt status
    unsigned long status = MAP_UARTIntStatus(UARTA1_BASE, true);
    MAP_UARTIntClear(UARTA1_BASE, status);

    // Get data from UARTA1 interrupt data buffer
    static char temp[MAX_MSG_LENGTH+1];
    while(MAP_UARTCharsAvail(UARTA1_BASE)) {
        char c = (char)MAP_UARTCharGetNonBlocking(UARTA1_BASE);

        if(c == '\0') {
            temp[msg_receive_length] = '\0';
        } else {
            if(msg_receive_length < MAX_MSG_LENGTH) {
                temp[msg_receive_length++] = c;
            }
        }
    }

    // Unpack data
    // Get username
    int i = 0;
    while(temp[i] != ATTRIBUTE_SEPARATOR[0] && temp[i] != '\0'){
        i++;
    }
    strncpy(sender_username, temp, i);
    sender_username[i] = '\0';

    // Get user color
    int j = i+1;
    while(temp[j] != ATTRIBUTE_SEPARATOR[0] && temp[j] != '\0'){
        j++;
    }
    char user_color[MAX_USERNAME_LENGTH + 1];
    strncpy(user_color, temp+i+1, j-i-1);
    user_color[j-i-1] = '\0';
    updateColor(&sender_color, user_color, user_color);

    // Get actual message
    int k = j+1;
    while(temp[k] != ATTRIBUTE_SEPARATOR[0] && temp[k] != '\0'){
        k++;
    }
    strncpy(msg_receive, temp+j+1, k-j-1);
    msg_receive[k-j-1] = '\0';

    update_sender = 1;
    msg_received_fully = 1;
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
    Message("Initializing OLED...\n\r");
    OLEDInit();
    Message("Initializing IR...\n\r");
    IRIntInit();
    Message("Initializing SysTick...\n\r");
    SysTickInit();
    Message("Initializing UARTA0...\n\r");
    UARTA0Init();
    Message("Initializing UARTA1...\n\r");
    UARTA1Init();

    Message("\t\t****************************************************\n\r");
    Message("\t\t*              Ready to text messages              *\n\r");
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
//            Report("All Bits: ");
//            report_binary_list(bits, 32);
//            Report("\r\n");

            // Extract TV and button code bits (bits 16-31)
            uint8_t tv_code_bits[16];
            uint8_t button_code_bits[16];
            for(i = 0; i < 16; i++){
                tv_code_bits[i] = bits[i];
                button_code_bits[i] = bits[16 + i];
            }

            // Print the TV Code
//            Report("TV Code: ");
//            report_binary_list(bits, 16);
//            Report("\r\n");

            // Print the data sent with the signal
//            Report("Data: ");
//            report_binary_list(button_code_bits, 16);
//            Report("\r\n");

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

            // Identify button
            switch (button_code) {
                case BUTTON_1:
                case BUTTON_2:
                case BUTTON_3:
                case BUTTON_4:
                case BUTTON_5:
                case BUTTON_6:
                case BUTTON_7:
                case BUTTON_8:
                case BUTTON_9:
                case BUTTON_0:
                    handle_remote_button_pressed(button_code);
                    drawMessages();
                    break;

                case BUTTON_DELETE:
                    delete_char_from_sending();
                    drawMessages();
                    break;

                case BUTTON_SEND:
                    if(msg_send[0] == '/'){
                        run_command();
                        drawMessages();
                        break;
                    }

                    send_message();        // sends over UART0 and clears compose
                    drawMessages();
                    break;

                default:
                    // ignore other keys for now
                    break;
            }

            if(msg_received_fully) {
                msg_received_fully = 0;
                Report("Just got: %s\r\n", msg_receive);
                drawMessages();

                msg_receive_length = 0;
            }

            if(update_me){
                updateMyUsername();
            }

            if(update_sender){
                updateSenderUsername();
            }
        }
    }
}
