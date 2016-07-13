/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include <stdio.h>
#include <lib/type.h>
#include <lib/error.h>
#include <lib/debug.h>
#include <exosite_api.h>
#include "platform_demo.h"
#include "app.h"
#include <platform_display.h>
#include "../../er_plat_microchip_pic32mzecsk/platform_display.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
//
//
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
static volatile unsigned char SDK_HEAP[65536];


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/
static struct exosite_class *exo;

int gCounter;

/**
 *  This function is called when the read operation is finished.
 *
 *  @param[in] status   Status of the read operation. It is ERR_SUCCESS
 *                      in case of success. Otherwise it is <0.
 *  @param[in] alias    Alias of the data resource.
 *  @param[in] value    Value of the data resource
 */
static void on_read(int status, const char *alias, const char *value)
{
    if (status == ERR_SUCCESS) {
        printf("Value read from server \"%s\" to %s\n", alias, value);
        if(!strcmp("leds", alias)) {
            int val = atoi(value);
            BSP_LEDStateSet(BSP_LED_1, (val & 1) ? BSP_LED_STATE_ON : BSP_LED_STATE_OFF);
            BSP_LEDStateSet(BSP_LED_2, (val & 2) ? BSP_LED_STATE_ON : BSP_LED_STATE_OFF);
            BSP_LEDStateSet(BSP_LED_3, (val & 4) ? BSP_LED_STATE_ON : BSP_LED_STATE_OFF);
            appData.leds_initialized = (status == ERR_SUCCESS) ? INITIALIZED : NOT_INITIALIZED;
        } else if(!strcmp("count", alias)) {
            gCounter = atoi(value);
            appData.counter_initialized = (status == ERR_SUCCESS) ? INITIALIZED : NOT_INITIALIZED;
        }
    }
}

/**
 *  This function is called when the write operation is finished.
 *
 *  @param[in] status   Status of the wrire operation. It is ERR_SUCCESS
 *                      in case of success. Otherwise it is <0.
 *  @param[in] alias    Alias of the data resource.
 *  @param[in] value    Value of the data resource
 */
static void on_write(int status, const char *alias)
{
    SYS_CONSOLE_PRINT("Write \"%s\" %s\r\n", alias, (status == ERR_SUCCESS) ? "succeeded" : "failed");
}

/**
 *  This function is called when the data resource changed on the server.
 *
 *  @param[in] status   Status of the read operation. It is ERR_SUCCESS
 *                      in case of success. Otherwise it is <0.
 *  @param[in] alias    Alias of the data resource.
 *  @param[in] value    Value of the data resource
 */
static void on_change(int status, const char *alias, const char *value)
{
    if (status == ERR_SUCCESS) {
        printf("Value changed on server \"%s\" to %s\n", alias, value);
        if (!strcmp("leds", alias)) {
            appData.leds_initialized = INITIALIZED;

            int val = atoi(value);
            BSP_LEDStateSet(BSP_LED_1, (val & 1) ? BSP_LED_STATE_ON : BSP_LED_STATE_OFF);
            BSP_LEDStateSet(BSP_LED_2, (val & 2) ? BSP_LED_STATE_ON : BSP_LED_STATE_OFF);
            BSP_LEDStateSet(BSP_LED_3, (val & 4) ? BSP_LED_STATE_ON : BSP_LED_STATE_OFF);
        }
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    int error;
    appData.state = APP_STATE_INIT;
    appData.leds_initialized = NOT_INITIALIZED;
    appData.counter_initialized = NOT_INITIALIZED;
    
    /** Platform initialization */
    error = platform_init();
    if (error)
        appData.state = APP_SDK_ERROR;

    /** SDK initialization */
    exosite_sdk_register_memory_buffer((void *)SDK_HEAP, sizeof(SDK_HEAP));
    error = exosite_sdk_init(printf, NULL);
    if (error)
        appData.state = APP_SDK_ERROR;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    SYS_STATUS          tcpipStat;
    const char          *netName, *netBiosName;
    static IPV4_ADDR    dwLastIP[2] = { {-1}, {-1} };
    TCPIP_NET_HANDLE    netH;
    IPV4_ADDR           ipAddr;
    TCPIP_MAC_ADDR*     pAdd;
    int                 i, nNets;
    char                str[32];
    int                 countDiff;
    enum exosite_device_status status;
    int error;
    char *vendor = "ta7lsaumvswz5mi";
    char *model = "ta7lsaumvswz5mi";
    char tmp[256];
        
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            /* Comment it back if you want to delete the stored CIK
             * (Only for debug purposes!!!!) */
            //platform_nvm_erase();

            tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if(tcpipStat < 0) {   /* some error occurred */
                
                SYS_CONSOLE_MESSAGE(" APP: TCP/IP stack initialization failed!\r\n");
                appData.state = APP_TCPIP_ERROR;
            }
            else if(tcpipStat == SYS_STATUS_READY) {
                
                // now that the stack is ready we can check the
                // available interfaces
                SYS_CONSOLE_MESSAGE(" APP: TCP/IP stack initialization sucessed!\r\n");
                nNets = TCPIP_STACK_NumberOfNetworksGet();
                for(i = 0; i < nNets; i++)
                {

                    netH = TCPIP_STACK_IndexToNet(i);
                    netName = TCPIP_STACK_NetNameGet(netH);
                    netBiosName = TCPIP_STACK_NetBIOSName(netH);

                    #if defined(TCPIP_STACK_USE_NBNS)
                        SYS_CONSOLE_PRINT("    Interface %s on host %s - NBNS enabled\r\n", netName, netBiosName);
                    #else
                        SYS_CONSOLE_PRINT("    Interface %s on host %s - NBNS disabled\r\n", netName, netBiosName);
                    #endif  // defined(TCPIP_STACK_USE_NBNS)

                }

                SYS_CONSOLE_MESSAGE(" APP: Waiting for IP address...\r\n");
                appData.state = APP_TCPIP_WAIT_FOR_IP;

            }
            break;
        }
        case APP_TCPIP_WAIT_FOR_IP:

            // if the IP address of an interface has changed
            // display the new value on the system console
            nNets = TCPIP_STACK_NumberOfNetworksGet();

            for (i = 0; i < nNets; i++)
            {
                netH = TCPIP_STACK_IndexToNet(i);
                ipAddr.Val = TCPIP_STACK_NetAddress(netH);
                pAdd = (TCPIP_MAC_ADDR*)TCPIP_STACK_NetAddressMac(netH);
                if(dwLastIP[i].Val != ipAddr.Val)
                {
                    dwLastIP[i].Val = ipAddr.Val;

                    if (ipAddr.v[0] != 0 && ipAddr.v[0] != 169) // Wait for a Valid IP
                    {
                        SYS_CONSOLE_PRINT("---------------------------------------------\r\n");
                        SYS_CONSOLE_PRINT(" If:          %s\r\n", TCPIP_STACK_NetNameGet(netH));
                        SYS_CONSOLE_PRINT(" MAC Address: %02x.%02x.%02x.%02x.%02x.%02x \r\n",pAdd->v[0], pAdd->v[1], pAdd->v[2], pAdd->v[3], pAdd->v[4], pAdd->v[5]);
                        SYS_CONSOLE_PRINT(" IP Address:  %d.%d.%d.%d \r\n", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
                        SYS_CONSOLE_PRINT("---------------------------------------------\r\n");
                        appData.state = APP_ER_SDK_INIT;
                    }
                }
            }
            break;

        case APP_ER_SDK_INIT:

            SYS_CONSOLE_MESSAGE(" Start Exosite Ready SDK\r\n");

            /** Create new instance */
            error = exosite_new(&exo, vendor, model, NULL, APP_PROTO_HTTP);
            if (error) {
                 appData.state = APP_SDK_ERROR;
                 break;
            }

            appData.state = APP_WAIT_FOR_ACTIVATION;
            break;

        case APP_WAIT_FOR_ACTIVATION:
            /** Wait until the activation is finished */
            status = exosite_get_status(exo);
            exosite_delay_and_poll(exo, 1000);
            if (status == DEVICE_STATUS_ACTIVATED) {
                SYS_CONSOLE_MESSAGE("The device is ACTIVATED!\r\n");
                appData.state = APP_CREATE_SUBSCRIPTIONS;
            } else if (status == DEVICE_STATUS_NOT_ACTIVATED) {
                SYS_CONSOLE_PRINT("The device is not activated. Ensure serial "
                                  "number '%02x%02x%02x%02x%02x%02x' is "
                                  "enabled for activation.\r\n",
                                  pAdd->v[0], pAdd->v[1], pAdd->v[2],
                                  pAdd->v[3], pAdd->v[4], pAdd->v[5]);
            }

            break;

        case APP_CREATE_SUBSCRIPTIONS:

            if(exosite_subscribe(exo, "leds", 0, on_change) == ERR_SUCCESS) {
                appData.leds_initialized = IN_PROGRESS;
                exosite_read(exo, "leds", on_read);
                appData.state = APP_APPLICATION;
            }
            break;

        case APP_APPLICATION:

            if (appData.leds_initialized == NOT_INITIALIZED) {
                appData.leds_initialized = IN_PROGRESS;
                exosite_read(exo, "leds", on_read);
            }
            if (appData.counter_initialized == NOT_INITIALIZED) {
                appData.counter_initialized = IN_PROGRESS;
                exosite_read(exo, "count", on_read);
            }
                    
            /** Update "count" data source */
            countDiff = get_and_clear_press_counter();
            if((appData.counter_initialized == INITIALIZED) && countDiff) {
                gCounter += countDiff;
                sprintf(str, "%d", gCounter);
                SYS_CONSOLE_PRINT("Counter: %s\r\n", str);
                exosite_write(exo, "count", str, on_write);
            }

            exosite_delay_and_poll(exo, 2000);

            break;

        case APP_PLATFORM_ERROR:
            SYS_CONSOLE_MESSAGE("Platform - Fatal Error!\r\n");
            appData.state = APP_FATAL_ERROR;
            break;

        case APP_TCPIP_ERROR:
            SYS_CONSOLE_MESSAGE("Network stack - Fatal Error!\r\n");
            appData.state = APP_FATAL_ERROR;
            break;
            
        case APP_SDK_ERROR:
            SYS_CONSOLE_MESSAGE("Exosite SDK - Fatal Error!\r\n");
            appData.state = APP_FATAL_ERROR;
            break;
            
        /* The default state should never be executed. */
        case APP_FATAL_ERROR:
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }

}


/*******************************************************************************
 End of File
 */
