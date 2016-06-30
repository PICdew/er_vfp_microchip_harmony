/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    sensor.c

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

#include "sensor.h"
#include <display/SSD1306.h>
#include <lib/type.h>
#include <lib/error.h>
#include <lib/debug.h>

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

SENSOR_DATA sensorData;
uint8_t deviceAddressSlave;
DRV_I2C_BUFFER_EVENT i2cOpStatus;

static uint8_t numOfBytes;
static uint8_t TXbuffer[20];
static volatile uint8_t RXbuffer[30];
typedef int64_t BME280_S32_t;

int Temperature;
int Humidity;

#define SLAVE_ADDRESS             0x76

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
    void SENSOR_Initialize ( void )

  Remarks:
    See prototype in sensor.h.
 */

void SENSOR_Initialize ( void )
{
    int32_t status;

    /* Place the App state machine in its initial state. */
    sensorData.state = SENSOR_STATE_INIT;

    status = system_mutex_create(&sensorData.sensore_update_mutex);
    if (status != ERR_SUCCESS) {
        ASSERT(0);
    }

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


DRV_I2C_BUFFER_EVENT APP_Check_Transfer_Status(DRV_HANDLE drvOpenHandle, DRV_I2C_BUFFER_HANDLE drvBufferHandle)
{
    return (DRV_I2C_TransferStatusGet(sensorData.drvI2CHandle_Master, drvBufferHandle));
}

void I2CMasterOpStatusCb(DRV_I2C_BUFFER_EVENT event,
                         DRV_I2C_BUFFER_HANDLE bufferHandle,
                         uintptr_t context)
{
    static uint32_t successCount = 0;

    switch (event) {
        case DRV_I2C_BUFFER_EVENT_COMPLETE:
            successCount++;
            Nop();
            break;
        case DRV_I2C_BUFFER_EVENT_ERROR:
            successCount--;
            break;
        default:
            break;
    }
}

/****************************************************************************************************/
/* Returns temperature in DegC, resolution is 0.01 DegC. Output value of ?5123? equals 51.23 DegC.  */
/***************************************************************************************************/


int64_t t_fine;
BME280_S32_t dig_T1, dig_T2, dig_T3, dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6;

static BME280_S32_t BME280_Compensate_T(BME280_S32_t adc_T) {
  BME280_S32_t temp1, temp2, T;

  temp1 = ((((adc_T>>3) -((BME280_S32_t)dig_T1<<1))) * ((BME280_S32_t)dig_T2)) >> 11;
  temp2 = (((((adc_T>>4) - ((BME280_S32_t)dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)dig_T1))) >> 12) * ((BME280_S32_t)dig_T3)) >> 14;
  t_fine = temp1 + temp2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}


/************************************************************************************************************/
/* Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits). */
/* Output value of ?47445? represents 47445/1024 = 46.333 %RH */
/************************************************************************************************************/
   int64_t h1;

int BME280_Compensate_H(int sen_hum ) {
  h1 = (t_fine - ((int64_t)76800));
  h1 = (((((sen_hum << 14) - ((( int64_t)dig_H4) << 20) - (((int64_t)dig_H5) * h1)) +
    ((int64_t)16384)) >> 15) * (((((((h1 * ((int64_t)dig_H6)) >> 10) * (((h1 *
    ((int64_t)dig_H3)) >> 11) + ((int64_t)32768))) >> 10) + ((int64_t)2097152)) *
    ((int64_t)dig_H2) + 8192) >> 14));
  h1 = (h1 - (((((h1 >> 15) * (h1 >> 15)) >> 7) * ((int64_t)dig_H1)) >> 4));
  h1 = (h1 < 0 ? 0 : h1);
  h1 = (h1 > 419430400 ? 419430400 : h1);
  return (unsigned long)(h1>>12);
}


/******************************************************************************
  Function:
    void SENSOR_Tasks ( void )

  Remarks:
    See prototype in app2.h.
 */
#define BME280_DATA_FRAME_SIZE 8
#define BME280_PRESSURE_MSB_REG  0xF7  //Pressure MSB
#define BME280_CHIP_ID_REG 0xD0  //Chip ID
#define BME280_CTRL_MEAS_REG 0xF4  //Ctrl Measure
#define BME280_CTRL_HUMID 0xF2  //Ctrl Measure
#define BME280_CALIBRATION_TEMP 0x88
#define BME280_CALIBRATION_HUM 0xE1

void SENSOR_Tasks(void)
{
    static uint32_t Timer;
    volatile int tmp1;
    volatile int tmp2;

    /* Check the application's current state. */
    switch (sensorData.state) {
            /* Application's initial state. */
        case SENSOR_STATE_INIT:

            sensorData.drvI2CHandle_Master = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE);

            /* event-handler set up receive callback from DRV_I2C_Tasks */
            DRV_I2C_BufferEventHandlerSet(sensorData.drvI2CHandle_Master, I2CMasterOpStatusCb, i2cOpStatus);

            if (sensorData.drvI2CHandle_Master != (DRV_HANDLE) NULL) {
                sensorData.state = SENSOR_STATE_SERVICE_TASKS_INIT;
                TXbuffer[0] = BME280_CTRL_MEAS_REG;
                TXbuffer[1] = 0xff;
                numOfBytes = 2;
            } else {
                sensorData.state = SENSOR_STATE_ERROR;
            }
            break;

        case SENSOR_STATE_SERVICE_TASKS_INIT:

            /* address of PIC32 slave */
            deviceAddressSlave = (SLAVE_ADDRESS << 1);

            /* Read Transaction to PIC32 Slave */
            if ((sensorData.drvI2CTxRxBufferHandle[0] == (DRV_I2C_BUFFER_HANDLE) NULL) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR))
            {
                sensorData.drvI2CTxRxBufferHandle[0] = DRV_I2C_Transmit(sensorData.drvI2CHandle_Master,
                        deviceAddressSlave,
                        &TXbuffer[0],
                        numOfBytes,
                        NULL);
                sensorData.state = SENSOR_STATE_SERVICE_TASKS_INIT_WAIT;
            }
            break;

        case SENSOR_STATE_SERVICE_TASKS_INIT_WAIT:

            if ((APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR))
            {

                if (TXbuffer[0] == BME280_CTRL_MEAS_REG) {
                    TXbuffer[0] = BME280_CTRL_HUMID;
                    TXbuffer[1] = 3;
                    sensorData.state = SENSOR_STATE_SERVICE_TASKS_INIT;

                } else {
                    sensorData.state = SENSOR_STATE_SERVICE_TASKS_SET_CALIB_REG;
                    TXbuffer[0] = BME280_CALIBRATION_TEMP;
                    numOfBytes = 1;
                }
            }
            else
                sensorData.state = SENSOR_STATE_SERVICE_TASKS_INIT_WAIT;
            break;

        case SENSOR_STATE_SERVICE_TASKS_SET_CALIB_REG:

            /* address of PIC32 slave */
            deviceAddressSlave = (SLAVE_ADDRESS << 1); //Write

            /* Read Transaction to PIC32 Slave */
            if ((sensorData.drvI2CTxRxBufferHandle[0] == (DRV_I2C_BUFFER_HANDLE) NULL) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR))
            {
                sensorData.drvI2CTxRxBufferHandle[0] = DRV_I2C_Transmit(sensorData.drvI2CHandle_Master,
                        deviceAddressSlave,
                        &TXbuffer[0],
                        numOfBytes,
                        NULL);
                sensorData.state = SENSOR_STATE_SERVICE_TASKS_CALIB_WAIT;

            }
            break;

        case SENSOR_STATE_SERVICE_TASKS_CALIB_WAIT:
            if ((APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR))
                sensorData.state = SENSOR_STATE_SERVICE_TASKS_READ_CALIB;
            else
                sensorData.state = SENSOR_STATE_SERVICE_TASKS_CALIB_WAIT;
            break;


        case SENSOR_STATE_SERVICE_TASKS_READ_CALIB:

            /* address of PIC32 slave */
            deviceAddressSlave = (SLAVE_ADDRESS << 1) + 1; //READ

            /* Number of bytes to read during a WriteRead Operation -
             * -2 takes account of register address which is 1st byte of
             * transmitted data and the NULL that at end of array */
            if (TXbuffer[0] == BME280_CALIBRATION_TEMP)
                numOfBytes = 28;
            else if (TXbuffer[0] == BME280_CALIBRATION_HUM)
                numOfBytes = 8;

            /* Read Transaction to PIC32 Slave */
            if ((sensorData.drvI2CTxRxBufferHandle[0] == (DRV_I2C_BUFFER_HANDLE) NULL) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR))
            {
                sensorData.drvI2CTxRxBufferHandle[0] = DRV_I2C_Receive(sensorData.drvI2CHandle_Master,
                        deviceAddressSlave,
                        &RXbuffer[0],
                        numOfBytes,
                        NULL);
            }

            sensorData.state = SENSOR_STATE_SERVICE_TASKS_READ_CALIB_WAIT;
            break;

        case SENSOR_STATE_SERVICE_TASKS_READ_CALIB_WAIT:
            if ((APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR))
            {
                if (TXbuffer[0] == BME280_CALIBRATION_TEMP) {
                    dig_T1 = (uint16_t )((RXbuffer[1]<<8) + RXbuffer[0]);
                    dig_T2 = (int16_t )((RXbuffer[3]<<8) + RXbuffer[2]);
                    dig_T3 = (int16_t )((RXbuffer[5]<<8) + RXbuffer[4]);

                    dig_H1 = (uint8_t)RXbuffer[25];
                    TXbuffer[0] = BME280_CALIBRATION_HUM;

                    sensorData.state = SENSOR_STATE_SERVICE_TASKS_SET_CALIB_REG;

                } else if (TXbuffer[0] == BME280_CALIBRATION_HUM) {
                    dig_H2 = (int16_t )(((uint32_t)RXbuffer[1]<<8) + RXbuffer[0]);
                    dig_H3 =  (uint8_t)RXbuffer[2];
                    dig_H4 = (int16_t )(((uint32_t)RXbuffer[3]<<4) + (RXbuffer[4] & 0x0F));
                    dig_H5 =  (int16_t)(RXbuffer[4]>>4) + ((uint32_t)RXbuffer[5]<<4);
                    dig_H6 = (int8_t)RXbuffer[6];

                    sensorData.state = SENSOR_STATE_SERVICE_TASKS;
                }


            } else
                sensorData.state = SENSOR_STATE_SERVICE_TASKS_READ_CALIB_WAIT;
            break;

        case SENSOR_STATE_SERVICE_TASKS:

            /* address of PIC32 slave */
            deviceAddressSlave = (SLAVE_ADDRESS << 1); //Write

            /* Number of bytes to read during a WriteRead Operation -
             * -2 takes account of register address which is 1st byte of
             * transmitted data and the NULL that at end of array */
            TXbuffer[0] = BME280_PRESSURE_MSB_REG;
            numOfBytes = 1;

            /* Read Transaction to PIC32 Slave */
            if ((sensorData.drvI2CTxRxBufferHandle[0] == (DRV_I2C_BUFFER_HANDLE) NULL) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR))
            {
                sensorData.drvI2CTxRxBufferHandle[0] = DRV_I2C_Transmit(sensorData.drvI2CHandle_Master,
                        deviceAddressSlave,
                        &TXbuffer[0],
                        numOfBytes,
                        NULL);
                sensorData.state = SENSOR_STATE_2_SERVICE_TASKS;

            }
            break;

        case SENSOR_STATE_2_SERVICE_TASKS:

            if ((APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR))
                sensorData.state = SENSOR_STATE_3_SERVICE_TASKS;
            else
                sensorData.state = SENSOR_STATE_2_SERVICE_TASKS;
            break;

        case SENSOR_STATE_3_SERVICE_TASKS:

            /* address of PIC32 slave */
            deviceAddressSlave = (SLAVE_ADDRESS << 1) + 1; //READ

            /* We will read 8 bytes
               - First three: Pressure
               - Next three: Temperature
               - Last two: Humidity
            */
            numOfBytes = 8;

            /* Read Transaction to PIC32 Slave */
            if ((sensorData.drvI2CTxRxBufferHandle[0] == (DRV_I2C_BUFFER_HANDLE) NULL) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR))
            {
                sensorData.drvI2CTxRxBufferHandle[0] = DRV_I2C_Receive(
                        sensorData.drvI2CHandle_Master,
                        deviceAddressSlave,
                        &RXbuffer[0],
                        numOfBytes,
                        NULL);
            }

            sensorData.state = SENSOR_STATE_4_SERVICE_TASKS;
            break;

        case SENSOR_STATE_4_SERVICE_TASKS:

            if ((APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
                    (APP_Check_Transfer_Status(sensorData.drvI2CHandle_Master, sensorData.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR))
                sensorData.state = SENSOR_STATE_DONE;
            else
                sensorData.state = SENSOR_STATE_4_SERVICE_TASKS;
            break;

        case SENSOR_STATE_DONE:

            //unsigned char str[50];

            system_mutex_lock(sensorData.sensore_update_mutex);

            Temperature  = (unsigned int)RXbuffer[5] >> 4;
            Temperature |= (unsigned int)RXbuffer[4] << 4;
            Temperature |= (unsigned int)RXbuffer[3] << 12;
            Temperature = BME280_Compensate_T((BME280_S32_t)Temperature);

            tmp1 = RXbuffer[6] << 8;
            tmp2 = RXbuffer[7];
            Humidity = tmp1 + tmp2;
            //Humidity = BME280_Compensate_H(tmp2);
            //Humidity = Humidity / 1024;

            system_mutex_unlock(sensorData.sensore_update_mutex);

            /*
            if(BSP_SwitchStateGet(BSP_SWITCH_2))
                sprintf((char *) str, "Temp:  %d.%01d \272C    ", (Temperature/100),(Temperature%100)/10);
            else
                sprintf((char *) str, "Humdity:  %d %%   ", Humidity);
            SSD1306_drawString(0, LINE2, (unsigned char *) str);
            */

            Nop();
            Nop();
            Nop();

            Timer = SYS_TMR_TickCountGet();
            sensorData.state = SENSOR_STATE_IDLE;
            break;

        case SENSOR_STATE_IDLE:

            if (SYS_TMR_TickCountGet() >= Timer + SYS_TMR_TickCounterFrequencyGet() )
            {
                //I had to redo INIT  since when HUM register
                // is only enable once,  it does not work
                // So,  redoing it every time ensures it works
                //Certainly not efficient but...
                sensorData.state = SENSOR_STATE_SERVICE_TASKS_INIT;
                TXbuffer[0] = BME280_CTRL_MEAS_REG;
                TXbuffer[1] = 0xff;
                numOfBytes = 2;
            }
            break;

        case SENSOR_STATE_ERROR:
            while (1);

            /* The default state should never be executed. */
        default:
            /* TODO: Handle error in application's state machine. */
            break;
    }
}

int getTemperature(void)
{
    int ret;
    system_mutex_lock(sensorData.sensore_update_mutex);
    ret = Temperature;
    system_mutex_unlock(sensorData.sensore_update_mutex);

    return ret;
}

int getHumidity(void)
{
    int ret;
    system_mutex_lock(sensorData.sensore_update_mutex);
    ret = Humidity;
    system_mutex_unlock(sensorData.sensore_update_mutex);

    return ret;
}

/*******************************************************************************
 End of File
 */
