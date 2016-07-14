/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    display_ctrl.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _DISPLAY_CTRL_H
#define _DISPLAY_CTRL_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <lib/type.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* In this state, the application opens the driver. */
    DISPLAY_CTRL_STATE_INIT,

    /* Application writes the data in the EEPROM. */
    DISPLAY_CTRL_STATE_1,

    /* Application writes the data in the EEPROM. */
    DISPLAY_CTRL_STATE_2,

    /* In this state, application is in IDLE state after completion. */
    DISPLAY_CTRL_STATE_IDLE,

    /* Application error state */
    DISPLAY_CTRL_STATE_ERROR,
} DISPLAY_CTRL_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef unsigned char SPI_DATA_TYPE;
#define MAX_NUM_OF_BYTES        64

#define MAX_NUM_OF_BYTES_IN_BUF (MAX_NUM_OF_BYTES + 4)

typedef struct
{
    BOOL isDisplayHWInitialized;

    /* Application current state */
    DISPLAY_CTRL_STATES     state;

    /* SPI Driver Handle  */
    DRV_HANDLE              drvSPIHandle;

    /* Write buffer handle */
    DRV_SPI_BUFFER_HANDLE   drvSPIWRBUFHandle;

    /* Read buffer handle */
    DRV_SPI_BUFFER_HANDLE   drvSPIRDBUFHandle;

    /* SPI Driver TX buffer  */
    SPI_DATA_TYPE           drvSPITXbuffer[MAX_NUM_OF_BYTES_IN_BUF];

    /* SPI Driver RX buffer  */
    SPI_DATA_TYPE           drvSPIRXbuffer[MAX_NUM_OF_BYTES_IN_BUF];
} DISPLAY_CTRL_DATA;

#define SPI_CS_PORT_ID          PORT_CHANNEL_J
#define SPI_CS_PORT_PIN         PORTS_BIT_POS_7

#define SPI_DC_PORT_ID          PORT_CHANNEL_D
#define SPI_DC_PORT_PIN         PORTS_BIT_POS_0


#define OLED_CS(a) SYS_PORTS_PinWrite ( PORTS_ID_0,SPI_CS_PORT_ID,SPI_CS_PORT_PIN,a )
#define OLED_DC(a) SYS_PORTS_PinWrite ( PORTS_ID_0,SPI_DC_PORT_ID,SPI_DC_PORT_PIN,a )
#define OLED_RST(a) SYS_PORTS_PinWrite ( PORTS_ID_0,PORT_CHANNEL_K,PORTS_BIT_POS_5,a )

//De-assert RESET on OLED display
//    SYS_PORTS_PinSet(PORTS_ID_0,PORT_CHANNEL_K,PORTS_BIT_POS_5)


#define APP_SPI_CS_SELECT()     \
                   SYS_PORTS_PinClear(PORTS_ID_0,SPI_CS_PORT_ID,SPI_CS_PORT_PIN)

#define APP_SPI_CS_DESELECT()   \
                     SYS_PORTS_PinSet(PORTS_ID_0,SPI_CS_PORT_ID,SPI_CS_PORT_PIN)



// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void DISPLAY_CTRL_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    DISPLAY_CTRL_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void DISPLAY_CTRL_Initialize ( void );

/*******************************************************************************
  Function:
    void DISPLAY_CTRL_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    DISPLAY_CTRL_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void DISPLAY_CTRL_Tasks( void );


#endif /* _DISPLAY_CTRL_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

