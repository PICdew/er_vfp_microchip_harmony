/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    switch_control.c

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

#include "system_config.h"
#include "system_definitions.h"
#include <system_utils.h>
#include <lib/type.h>
#include <lib/error.h>
#include <lib/debug.h>
#include <exosite_api.h>
#include "switch_control.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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

SWITCH_CONTROL_DATA switch_controlData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SWITCH_CONTROL_Initialize ( void )

  Remarks:
    See prototype in switch_control.h.
 */

void SWITCH_CONTROL_Initialize ( void )
{
    int32_t status;
            
    /* Place the App state machine in its initial state. */
    switch_controlData.state = SWITCH_CONTROL_STATE_INIT;
    switch_controlData.pressCounter = 0;
    switch_controlData.swDownPressed = FALSE;
    switch_controlData.swUpPressed = FALSE;
    
    status = system_mutex_create(&switch_controlData.lock_mutex);
    if (status != ERR_SUCCESS) {
        error_log(DEBUG_PLATFORM, ("Mutex can not be created\n"), status);
    }
}


/******************************************************************************
  Function:
    void SWITCH_CONTROL_Tasks ( void )

  Remarks:
    See prototype in switch_control.h.
 */

void SWITCH_CONTROL_Tasks ( void )
{
    BOOL swUp, swDown;
    /* Check the application's current state. */
    switch ( switch_controlData.state )
    {
        /* Application's initial state. */
        case SWITCH_CONTROL_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                switch_controlData.state = SWITCH_CONTROL_STATE_SERVICE_TASKS;
            }
            break;
        }

        case SWITCH_CONTROL_STATE_SERVICE_TASKS:
        {
            swUp = !BSP_SwitchStateGet(BSP_SWITCH_1);
            swDown = !BSP_SwitchStateGet(BSP_SWITCH_2);
            
            if(swUp != switch_controlData.swUpPressed) {
                if(swUp) {
                    system_mutex_lock(switch_controlData.lock_mutex);
                    switch_controlData.pressCounter++;
                    system_mutex_unlock(switch_controlData.lock_mutex);
                }
                SYS_CONSOLE_PRINT("Button up %s\r\n", swUp ? "Pressed" : "Released");
                switch_controlData.swUpPressed = !switch_controlData.swUpPressed;
            }
            if(swDown != switch_controlData.swDownPressed) {
                if(swDown) {
                    system_mutex_lock(switch_controlData.lock_mutex);
                    switch_controlData.pressCounter--;
                    system_mutex_unlock(switch_controlData.lock_mutex);
                }
                SYS_CONSOLE_PRINT("Button down %s\r\n", swDown ? "Pressed" : "Released");
                switch_controlData.swDownPressed = !switch_controlData.swDownPressed;
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

int get_and_clear_press_counter(void) {
    
    int ret;
    system_mutex_lock(switch_controlData.lock_mutex);
    ret = switch_controlData.pressCounter;
    switch_controlData.pressCounter = 0;
    system_mutex_unlock(switch_controlData.lock_mutex);
    
    return ret;
} 

/*******************************************************************************
 End of File
 */
