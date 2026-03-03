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
// Application Name     -   SSL Demo
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************

#include <stdio.h>
#include <stdlib.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

//Common interface includes
#include "pinmux.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"

// Custom includes
#include "utils/network_utils.h"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                2    /* Current Date */
#define MONTH               3     /* Month 1-12 */
#define YEAR                2026  /* Current year */
#define HOUR                12     /* Time - hours */
#define MINUTE              17    /* Time - minutes */
#define SECOND              0     /* Time - seconds */


#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "a2u36470ncd0n8-ats.iot.us-east-2.amazonaws.com" // updated
#define GOOGLE_DST_PORT       8443

// available endpoints found at:
//      AWS IoT > Manage > Things > ayden_eec172_cc3200_board > Classic Shadow > MQTT Topics
#define GETHEADER "GET /things/ayden_eec172_cc3200_board/shadow HTTP/1.1\r\n"     // updated
#define POSTHEADER "POST /things/ayden_eec172_cc3200_board/shadow HTTP/1.1\r\n"   // updated
#define HOSTHEADER "Host: a2u36470ncd0n8-ats.iot.us-east-2.amazonaws.com\r\n"            // updated

#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static int set_time();
static void BoardInit(void);
static int http_post(int);
static int http_get(int);

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
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
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void main() {
    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    InitTerm();
    ClearTerm();
    UART_PRINT("My terminal works!\n\r");

    // initialize global default app configuration
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    //Connect the CC3200 to the local access point
    long status = connectToAccessPoint();
    //Set time so that encryption can be used
    status = set_time();
    if(status < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }

    //Connect to the website with TLS encryption
    status = tls_connect();
    if(status < 0) {
        ERR_PRINT(status);
        return;
    }

//    http_post(status);
    http_get(status);

    sl_Stop(SL_STOP_TIMEOUT);
    LOOP_FOREVER();
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

static char* createMessageJsonString(char messageToSend[]){
    char* jsonString = (char*) malloc(sizeof(char) * 512);

    char everythingBeforeMessage[] = "{"  \
            "\"state\":{"  \
                "\"desired\":{"  \
                    "\"message\":\"";

    char everythingAfterMessage[] = "\""  \
                "}"  \
            "}"  \
        "}\r\n\r\n";

    char* jsonStringCursor = jsonString;
    strcpy(jsonStringCursor, everythingBeforeMessage);
    jsonStringCursor += strlen(everythingBeforeMessage);
    strcpy(jsonStringCursor, messageToSend);
    jsonStringCursor += strlen(messageToSend);
    strcpy(jsonStringCursor, everythingAfterMessage);
    jsonStringCursor += strlen(everythingAfterMessage);

    return jsonString;
}

static char* createPostPayload(char messageToSend[]){
    char* fullPayload = (char*) malloc(sizeof(char) * 512);
    char* payloadCursor;
    char payloadLengthString[200];

    payloadCursor = fullPayload;
    strcpy(payloadCursor, POSTHEADER);
    payloadCursor += strlen(POSTHEADER);
    strcpy(payloadCursor, HOSTHEADER);
    payloadCursor += strlen(HOSTHEADER);
    strcpy(payloadCursor, CHEADER);
    payloadCursor += strlen(CHEADER);
    strcpy(payloadCursor, "\r\n\r\n");

    strcpy(payloadCursor, CTHEADER);
    payloadCursor += strlen(CTHEADER);
    strcpy(payloadCursor, CLHEADER1);
    payloadCursor += strlen(CLHEADER1);

    char* jsonMessage;
    jsonMessage = createMessageJsonString(messageToSend);

    int dataLength = strlen(jsonMessage);
    sprintf(payloadLengthString, "%d", dataLength);
    strcpy(payloadCursor, payloadLengthString);
    payloadCursor += strlen(payloadLengthString);
    strcpy(payloadCursor, CLHEADER2);
    payloadCursor += strlen(CLHEADER2);

    strcpy(payloadCursor, jsonMessage);
    payloadCursor += strlen(jsonMessage);

//    UART_PRINT(fullPayload);

    free(jsonMessage);
    return fullPayload;
}

static char* createGetPayload(){
    char* fullPayload = (char*) malloc(sizeof(char) * 512);
    char* payloadCursor;

    payloadCursor = fullPayload;
    strcpy(payloadCursor, GETHEADER);
    payloadCursor += strlen(GETHEADER);
    strcpy(payloadCursor, HOSTHEADER);
    payloadCursor += strlen(HOSTHEADER);
    strcpy(payloadCursor, CHEADER);
    payloadCursor += strlen(CHEADER);
    strcpy(payloadCursor, "\r\n");

//    UART_PRINT(fullPayload);

    return fullPayload;
}

static int getResponse(int socketID){
//    char* response = (char*) malloc(sizeof(char) * 1460);
    char response[1460];
    int numBytesReceived = sl_Recv(socketID, &response[0], sizeof(response), 0);

    if(numBytesReceived < 0) {
        return numBytesReceived;  // will be an error code
    }

    response[numBytesReceived] = '\0';
    UART_PRINT(response);
    UART_PRINT("\n\r\n\r");

    return 0;
}

static int http_post(int iTLSSockID){
    char messageToSend[] = "this is a fifth message debugging";

    char* payload;
    payload = createPostPayload(messageToSend);
    UART_PRINT(payload);

    //
    // Send the packet to the server */
    //
    int status = 0;
    status = sl_Send(iTLSSockID, payload, strlen(payload), 0);
    free(payload);
    if(status < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",status);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return status;
    }

    status = getResponse(iTLSSockID);
    if(status < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r", status);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
       return status;
    }

    return 0;
}

static int http_get(int iTLSSockID){
    char* payload;
    payload = createGetPayload();
    UART_PRINT(payload);

    //
    // Send the packet to the server
    //
    int status = 0;
    status = sl_Send(iTLSSockID, payload, strlen(payload), 0);
    if(status < 0) {
        UART_PRINT("GET failed. Error Number: %i\n\r", status);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return status;
    }

    status = getResponse(iTLSSockID);
    if(status < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r", status);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
       return status;
    }

    return 0;
}
