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
// Application Name     - Timer Demo
// Application Overview - This application is to showcases the usage of Timer 
//                        DriverLib APIs. The objective of this application is 
//                        to showcase the usage of 16 bit timers to generate 
//                        interrupts which in turn toggle the state of the GPIO 
//                        (driving LEDs).
//                        Two timers with different timeout value(one is twice 
//                        the other) are set to toggle two different GPIOs which 
//                        in turn drives two different LEDs, which will give a 
//                        blinking effect.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup timer_demo
//! @{
//
//*****************************************************************************

// Standard include
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "timer.h"
#include "utils.h"

// Common interface includes
#include "timer_if.h"
#include "gpio_if.h"

#include "pin_mux_config.h"

// new stuff
#include "uart_if.h"
#include "gpio.h"
#include "systick.h"


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION        "1.4.0"
#define FOREVER                    1


//---------------------------------------------------
// new stuff
//---------------------------------------------------

// from systick-example

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL


//*****************************************************************************
//                      Global Variables
//*****************************************************************************

// for Vector Table
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif


// for the timer interrupt handler.
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;


//---------------------------------------------------
// new stuff
//---------------------------------------------------

// taken from gpio-interrupt-example
typedef struct PinSetting {
    unsigned long base;
    unsigned int baseInterrupt;
    unsigned int pin;
} PinSetting;

static const PinSetting RECEIVER = {
    .base = GPIOA3_BASE,
    .baseInterrupt = INT_GPIOA3,
    .pin = 0x10
};



//static void receiverIntHandler(void) { // SW3 handler
//    unsigned long ulStatus;
//
//    ulStatus = MAP_GPIOIntStatus (RECEIVER.base, true);
//    MAP_GPIOIntClear(RECEIVER.base, ulStatus);        // clear interrupts on GPIOA1
//    SW3_intcount++;
//    SW3_intflag=1;
//}

// track systick counter periods elapsed
// if it is not 0, we ktimeCurrSignal the transmission ended
volatile int systick_cnt = 0;

// Decoder state
volatile unsigned long timeLastSignal = 0;
volatile unsigned long prevTimeBtwnSignals = 0;

volatile unsigned long receiverData = 0;
volatile int      numReceiverDataBits = 0;
volatile int      receivingNewFrame = 0;
volatile int      allDataReceived = 0;

const unsigned long TICKS_FOR_NEW_FRAME = 30000;  // big gap indicates new frame
const unsigned long TICKS_BTWN_BITS   = 2000;   // threshold between 0 and 1


static inline unsigned long ticksBetweenSignals(unsigned long prev, unsigned long timeCurrSignal){
    // SysTick counts down
    // Mask handles wrap-around for 24-bit counter
    return (prev - timeCurrSignal) & SYSTICK_RELOAD_VAL;
}

static void receiverIntHandler(void) {
    unsigned long ulStatus = MAP_GPIOIntStatus(RECEIVER.base, true);
    MAP_GPIOIntClear(RECEIVER.base, ulStatus);   // clear interrupts on the receiver's base

    // Only handle if our receiver pin got something
    if (! ((ulStatus & RECEIVER.pin) > 0) ) return;

    Message("Receiver received signal.\n\r");
    return;

    unsigned long timeCurrSignal = SysTickValueGet();
    unsigned long timeBtwnSignals = ticksBetweenSignals(timeLastSignal, timeCurrSignal);
    timeLastSignal = timeCurrSignal;

    prevTimeBtwnSignals = timeBtwnSignals;

    ////
    receiverData = timeCurrSignal;
    allDataReceived = 1;
    return;
    ////

    // 1) Detect new frame
    if (timeBtwnSignals > TICKS_FOR_NEW_FRAME) {
        receiverData = 0;
        numReceiverDataBits = 0;
        receivingNewFrame = 1;
        return;
    }

    if (!receivingNewFrame) return;

    // 2) Convert pulse timing -> bit
    int bit = (timeBtwnSignals > TICKS_BTWN_BITS) ? 1 : 0;

    // 3) Shift into code
    receiverData = (receiverData << 1) | (unsigned long)bit;
    numReceiverDataBits++;

    // 4) If complete (commonly 32 bits)
    if (numReceiverDataBits >= 32) {
        allDataReceived = 1;        // tell main loop to print/interpret
        receivingNewFrame = 0;
    }
}

static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
}


static void initializeSysTickCounter(void)
{
    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();

    Message("Initialized systick counter.\n\r");
}

static void initializeReceiverInterrupt(void)
{
    // Register the receiver interrupt
    MAP_GPIOIntRegister(RECEIVER.base, receiverIntHandler);
//    MAP_IntPrioritySet(RECEIVER.baseInterrupt, INT_PRIORITY_LVL_0);

    // The receiver idles high/is active-low. Start with FALLING edge for simpler timing.
    MAP_GPIOIntTypeSet(RECEIVER.base, RECEIVER.pin, GPIO_FALLING_EDGE);

    // clear the interrupt on the receiver base
    unsigned long ulStatus = MAP_GPIOIntStatus(RECEIVER.base, false);
    MAP_GPIOIntClear(RECEIVER.base, ulStatus);

    // enable the receiver's interrupt
    MAP_GPIOIntEnable(RECEIVER.base, RECEIVER.pin);

    Message("Initialized receiver interrupt.\n\r");
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
//
//!    main function demonstrates the use of the timers to generate
//! periodic interrupts.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
int main(void){
    //
    // Initialize board configurations
    BoardInit();

    //
    // Configure pins
    //
    PinMuxConfig();

    // Register the interrupt handler for the signal
    MAP_GPIOIntRegister(RECEIVER.base, receiverIntHandler);

    // initialize and clear terminal
    InitTerm();
    ClearTerm();

    // initialize interrupts
//    initializeSysTickCounter();
    initializeReceiverInterrupt();

    timeLastSignal = SysTickValueGet();


    Message("\t\t****************************************************\n\r");
    Message("\t\t                            Lab 3\n\r");
    Message("\t\t****************************************************\n\r");
    Message("\n\n\n\r");

    Report("receiver data:\n\r");
    Report("receiver base: %d\n\r", RECEIVER.base);
    Report("receiver interrupt: %d\n\r", RECEIVER.baseInterrupt);
    Report("receiver pin: %d\n\r", RECEIVER.pin);


    while(FOREVER){
//        while(allDataReceived == 0){;}

        if(allDataReceived){
            allDataReceived = 0;

            Report("Got: %d\n\r", receiverData);
        }
    }



//    //
//    // configure the LED RED and GREEN
//    //
//    GPIO_IF_LedConfigure(LED1|LED3);
//
//    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
//    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
//
//    //
//    // Base address for first timer
//    //
//    g_ulBase = TIMERA0_BASE;
//    //
//    // Base address for second timer
//    //
//    g_ulRefBase = TIMERA1_BASE;
//    //
//    // Configuring the timers
//    //
//    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
//    Timer_IF_Init(PRCM_TIMERA1, g_ulRefBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
//
//    //
//    // Setup the interrupts for the timer timeouts.
//    //
//    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
//    Timer_IF_IntSetup(g_ulRefBase, TIMER_A, TimerRefIntHandler);
//
//    //
//    // Turn on the timers feeding values in mSec
//    //
//    Timer_IF_Start(g_ulBase, TIMER_A, 500);
//    Timer_IF_Start(g_ulRefBase, TIMER_A, 1000);
//
//    //
//    // Loop forever while the timers run.
//    //
//    while(FOREVER)
//    {
//    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
