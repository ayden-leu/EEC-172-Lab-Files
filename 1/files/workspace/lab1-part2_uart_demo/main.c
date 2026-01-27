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
// Application Name     - UART Demo
// Application Overview - The objective of this application is to showcase the 
//                        use of UART. The use case includes getting input from 
//                        the user and display information on the terminal. This 
//                        example take a string as input and display the same 
//                        when enter is received.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup uart_demo
//! @{
//
//*****************************************************************************

// Driverlib includes
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "uart.h"
#include "interrupt.h"
#include "pin_mux_config.h"
#include "utils.h"
#include "prcm.h"

// Driverlib includes
#include "hw_apps_rcm.h"

// Common interface include
#include "uart_if.h"

#include "gpio.h"
//#include "gpio_if.h"

//*****************************************************************************
//                          MACROS                                  
//*****************************************************************************
#define APPLICATION_VERSION  "1.4.0"
#define APP_NAME             "GPIO"
#define CONSOLE              UARTA0_BASE
#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)
#define MAX_STRING_LENGTH    80

// ====== new macros ======
        // Signal Name   Pin Num   Device               GPIO Bit
        // GPIO_9          64      Red LED              1
        // GPIO_10          1      Yellow LED           2
        // GPIO_11          2      Green LED            3
        // GPIO_13          4      SW3                  5
        // GPIO_22         15      SW2                  6
        // GPIO_28         18      P18 on P2 header     4

        // the corresponding pin is GPIO_PIN_[GPIO Bit]
        //      GPIO Bit found in CC3200 tech ref manual

        // to figure out the base to use:
        //      the name is GPIOA[the number]_BASE
        //      to find the number, take the number in the signal name, divide by 8, then round down
        //      found in blinky/gpio_if.c/GPIO_IF_GetPortNPin()
        //      also found in the CC3200 tech ref manual

#define RED_LED_BASE      GPIOA1_BASE
#define RED_LED_PIN       GPIO_PIN_1
#define YELLOW_LED_BASE     GPIOA1_BASE
#define YELLOW_LED_PIN      GPIO_PIN_2
#define GREEN_LED_BASE        GPIOA1_BASE
#define GREEN_LED_PIN         GPIO_PIN_3

#define SW3_BASE   GPIOA1_BASE
#define SW3_PIN    GPIO_PIN_5
#define SW2_BASE   GPIOA2_BASE
#define SW2_PIN    GPIO_PIN_6

#define P18_BASE   GPIOA3_BASE
#define P18_PIN    GPIO_PIN_4

#define MODE_IDLE          0
#define MODE_LED_BINARY    1
#define MODE_LED_BLINK     2


void printInt(unsigned short num){
    if(num == 0){ Message("0\n\r"); }
    else if(num == 1){ Message("1\n\r"); }
    else if(num == 2){ Message("2\n\r"); }
    else if(num == 3){ Message("3\n\r"); }
    else if(num == 4){ Message("4\n\r"); }
    else if(num == 5){ Message("5\n\r"); }
    else if(num == 6){ Message("6\n\r"); }
    else if(num == 7){ Message("7\n\r"); }
    else if(num == 8){ Message("8\n\r"); }
    else if(num == 9){ Message("9\n\r"); }
    else if(num == 10){ Message("10\n\r"); }
    else if(num == 11){ Message("11\n\r"); }
    else if(num == 12){ Message("12\n\r"); }
    else if(num == 13){ Message("13\n\r"); }
    else if(num == 14){ Message("14\n\r"); }
    else if(num == 15){ Message("15\n\r"); }
    else if(num == 16){ Message("16\n\r"); }
    else{ Message("Number too big!\n\r"); }
}



//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile int g_iCounter = 0;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//*****************************************************************************
//                      LOCAL DEFINITION                                   
//*****************************************************************************

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t        CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
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
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//   new stuff
//*****************************************************************************
void turnOffLEDs(unsigned short *hardwareStates){
    GPIOPinWrite(RED_LED_BASE, RED_LED_PIN, 0);
    GPIOPinWrite(YELLOW_LED_BASE, YELLOW_LED_PIN, 0);
    GPIOPinWrite(GREEN_LED_BASE, GREEN_LED_PIN, 0);

    // update state
    *hardwareStates = *hardwareStates & 0b1111111111111000;
}

void blinkAllLEDs(unsigned long counterVal, unsigned short *hardwareStates){
    // don't know how to parallelize, so use a counter system
    unsigned long durationOn = 1 * 100000;
    unsigned long durationOff = 1 * 100000;
    unsigned long clippedCounter = counterVal % (durationOn + durationOff);

    if(clippedCounter == 0){
//        Message("LEDs on\n\r");

        GPIOPinWrite(RED_LED_BASE, RED_LED_PIN, RED_LED_PIN);
        GPIOPinWrite(YELLOW_LED_BASE, YELLOW_LED_PIN, YELLOW_LED_PIN);
        GPIOPinWrite(GREEN_LED_BASE, GREEN_LED_PIN, GREEN_LED_PIN);

        // update state
        *hardwareStates = *hardwareStates | 0b0000000000000111;
    }
    else if(clippedCounter == durationOn){
//        Message("LEDs off\n\r");
        turnOffLEDs(hardwareStates);
    }
}

void binaryBlinkLEDs(unsigned long counterVal, unsigned short *hardwareStates){
    // don't know how to parallelize, so use a counter system
    unsigned long timeBetweenIncrement = 1 * 100000;
    unsigned long clippedCounter = counterVal % timeBetweenIncrement;

    // also acts as the binary counter
    unsigned short LEDStates = *hardwareStates & 0b0000000000000111;

    if(clippedCounter == 0){
        // increment counter and account for overflow
        LEDStates++;
        if(LEDStates > 0b111){
//            Message("Wrap around\n\r");
            LEDStates = 0;
        }

        // update physical LED counter
        if(LEDStates & 0b001){ GPIOPinWrite(RED_LED_BASE, RED_LED_PIN, RED_LED_PIN); }
                         else{ GPIOPinWrite(RED_LED_BASE, RED_LED_PIN, 0); }
        if(LEDStates & 0b010){ GPIOPinWrite(YELLOW_LED_BASE, YELLOW_LED_PIN, YELLOW_LED_PIN); }
                         else{ GPIOPinWrite(YELLOW_LED_BASE, YELLOW_LED_PIN, 0); }
        if(LEDStates & 0b100){ GPIOPinWrite(GREEN_LED_BASE, GREEN_LED_PIN, GREEN_LED_PIN); }
                         else{ GPIOPinWrite(GREEN_LED_BASE, GREEN_LED_PIN, 0); }

        // reset the bits corresponding to the LEDs before updating
        *hardwareStates = *hardwareStates & 0b1111111111111000;
        *hardwareStates = *hardwareStates | LEDStates;
    }
}

void raiseP18(unsigned short *hardwareStates){
    // don't bother raising if risen already
    if(*hardwareStates & 0b1000) return;

    GPIOPinWrite(P18_BASE, P18_PIN, P18_PIN);
    *hardwareStates = *hardwareStates | 0b0000000000001000;
}
void lowerP18(unsigned short *hardwareStates){
    // don't bother lowering if lower already
    if(!(*hardwareStates & 0b1000)) return;

    GPIOPinWrite(P18_BASE, P18_PIN, 0);
    *hardwareStates = *hardwareStates & 0b1111111111110111;
}

void handleCurrentMode(unsigned long counterVal, unsigned short *hardwareStates, unsigned short mode){
    switch(mode){
        case MODE_IDLE:
            turnOffLEDs(hardwareStates);
            break;
        case MODE_LED_BINARY:
            binaryBlinkLEDs(counterVal, hardwareStates);
            break;
        case MODE_LED_BLINK:
            blinkAllLEDs(counterVal, hardwareStates);
            break;
    }
}

//*****************************************************************************
//
//! Main function handling the uart echo. It takes the input string from the
//! terminal while displaying each character of string. whenever enter command 
//! is received it will echo the string(display). if the input the maximum input
//! can be of 80 characters, after that the characters will be treated as a part
//! of next string.
//!
//! \param  None
//!
//! \return None
//! 
//*****************************************************************************
void main()
{
//    char cString[MAX_STRING_LENGTH+1];
//    char cCharacter;
//    int iStringLength = 0;
    //
    // Initailizing the board
    //
    BoardInit();
    //
    // Muxing for Enabling UART_TX and UART_RX.
    //
    PinMuxConfig();
    //
    // Initialising the Terminal.
    //
    InitTerm();
    //
    // Clearing the Terminal.
    //
    ClearTerm();

    DisplayBanner(APP_NAME);
    Message("\t\t****************************************************\n\r");
    Message("\n\r");
    Message("\t\t Push SW3 to start LED binary counting  \n\r");
    Message("\n\r");
    Message("\t\t Push SW2 to blink LEDs on and off      \n\r");
    Message("\n\r");
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");

    unsigned long switch3WasPressed, switch3IsPressed = 0;
    unsigned long switch2WasPressed, switch2IsPressed = 0;
    unsigned short currentMode = MODE_IDLE;

    // Bit usage.  Last entry is the most rightwards bit.
    // P18 State
    // Green LED State, Yellow LED State, Red LED state
    unsigned short hardwareStates = 0b0000000000000000;

    unsigned long counter = 0;

    while(1){
        switch3WasPressed = switch3IsPressed;
        switch3IsPressed = GPIOPinRead(SW3_BASE, SW3_PIN);

        switch2WasPressed = switch2IsPressed;
        switch2IsPressed = GPIOPinRead(SW2_BASE, SW2_PIN);

        // when switch 3 pressed, start binary counting sequence on LEDs on loop
        //      also, print to console "SW3 pressed"
        //      should not be printed again until switch 2 is pressed
        //      also, set P18 to low
        if(switch3IsPressed && !switch3WasPressed && currentMode != MODE_LED_BINARY){
            Message("SW3 Pressed\n\r");
            currentMode = MODE_LED_BINARY;
            counter = 0;
            lowerP18(&hardwareStates);
        }

        // when switch 2 pressed, blink the LEDs continuously
        //      also, print "SW2 pressed"
        //      should not be printed again until switch 3 is pressed
        //      also, set P18 to high
        if(switch2IsPressed && !switch2WasPressed && currentMode != MODE_LED_BLINK){
            Message("SW2 Pressed\n\r");
            currentMode = MODE_LED_BLINK;
            counter = 0;
            raiseP18(&hardwareStates);
        }

        if(switch3IsPressed && switch2IsPressed && currentMode != MODE_IDLE){
            Message("Now idling...\n\r");
            currentMode = MODE_IDLE;
        }

        handleCurrentMode(counter, &hardwareStates, currentMode);
        counter++;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
    

