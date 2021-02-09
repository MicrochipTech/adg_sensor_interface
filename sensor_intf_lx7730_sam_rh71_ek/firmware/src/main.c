/*******************************************************************************
* Copyright (C) 2020 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes

// *****************************************************************************
// *****************************************************************************
// Section: Defines
// *****************************************************************************
// *****************************************************************************

/* LX7730 Function Enable Register */
#define LX7730_SPI_FUNC_EN_REG                             (0x01)
/* LX7730 Non Inverting Mux Channel Register */
#define LX7730_SPI_NON_INV_MUX_CH_REG                      (0x03)
/* LX7730 Inverting Mux Channel Register */
#define LX7730_SPI_INV_MUX_CH_REG                          (0x04)
/* LX7730 ADC Control Register */
#define LX7730_SPI_ADC_CTRL_REG                            (0x08)
/* LX7730 ADC Result MSB Register */
#define LX7730_SPI_ADC_RES_MSB_REG                         (0x09)
/* LX7730 ADC Result LSB Register */
#define LX7730_SPI_ADC_RES_LSB_REG                         (0x0A)

/* LX7730 MUX Channel Select */
#define LX7730_MUX_CH57                                    (0x07)
#define LX7730_MUX_CH58                                    (0x0F)
#define LX7730_MUX_CH59                                    (0x17)
#define LX7730_MUX_CH60                                    (0x1F)
#define LX7730_MUX_CH61                                    (0x27)
#define LX7730_MUX_CH62                                    (0x2F)
#define LX7730_MUX_CH63                                    (0x37)
#define LX7730_MUX_CH64                                    (0x3F)

/* UART interface command read LX7730 address */
#define UART_INTF_READ_ADDR                                (0x01)

/* UART interface command get ADC sensors data */
#define UART_INTF_GET_ADC_DATA                             (0x07)

// *****************************************************************************
// *****************************************************************************
// Section: Variables
// *****************************************************************************
// *****************************************************************************

volatile bool isTransferDone = false;

volatile uint8_t lastRegisterRead = 0;

volatile uint16_t appSensorLight = 0;
volatile uint16_t appSensorDistance = 0;
volatile uint16_t appSensorPressure = 0;
volatile uint16_t appSensorTempScaled = 0;
volatile uint16_t appSensorMagnScaled = 0;
volatile uint16_t appSensorAccZScaled = 0;
volatile uint16_t appSensorAccYScaled = 0;
volatile uint16_t appSensorAccXScaled = 0;

// *****************************************************************************
// *****************************************************************************
// Section: Local functions
// *****************************************************************************
// *****************************************************************************

/* This function will be called by SPI PLIB when transfer is completed */
void SPIEventHandler(uintptr_t context )
{
    isTransferDone = true;
}

// *****************************************************************************
/* Function:
    static bool LX7730_SPI_ComputeParity(uint16_t value)

   Summary:
    Compute parity value of the given frame.

   Parameters:
    value - frame to compute.

   Returns:
    Parity of the computed frame.
*/
static bool LX7730_SPI_ComputeParity(uint16_t value)
{
    bool parity = 0;
    while (value)
    {
        parity = !parity;
        value = value & (value-1);
    }
    return parity;
}

// *****************************************************************************
/* Function:
    static uint16_t LX7730_SPI_EncodeFrame(uint8_t reg, uint8_t data, bool writeData)

   Summary:
    Encode LX7730 SPI frame to read or write data to registers.

   Parameters:
    reg - Register to read or write from.
    data - if applicable, data to write to register.
    writeData - True to generate a write frame, false to generate a read frame.

   Returns:
    The encoded SPI frame.
*/
static uint16_t LX7730_SPI_EncodeFrame(uint8_t reg, uint8_t data, bool writeData)
{
    uint16_t frame = (reg << 9);

    if (writeData)
        frame |= 0x4000 | (data << 1);

    if (LX7730_SPI_ComputeParity(frame))
        frame |= 0x0001;

    return frame;
}

// *****************************************************************************
/* Function:
    static void LX7730_SPI_DecodeFrame(uint16_t frame, uint8_t* reg, uint8_t* value)

   Summary:
    Decode recieved LX7730 SPI frame.

   Parameters:
    frame - Frame to decode.
    reg - pointer to store the decoded register value.
    value - pointer to store the decoded value read in the register.

   Returns:
    None.
*/
static void LX7730_SPI_DecodeFrame(uint16_t frame, uint8_t* reg, uint8_t* value)
{
    *reg = (uint8_t)((frame >> 9) & 0x1F);
    *value = (uint8_t)((frame >> 1) & 0xFF);
}

// *****************************************************************************
/* Function:
    static void LX7730_SPI_WriteReg(uint8_t reg, uint8_t value)

   Summary:
    Write value to LX7730 register.

   Parameters:
    reg - LX7730 register to write.
    value - Value to write in the register.

   Returns:
    None.
*/
static void LX7730_SPI_WriteReg(uint8_t reg, uint8_t value)
{
    uint16_t txData;
    uint16_t rxData;

    txData = LX7730_SPI_EncodeFrame(reg, value, true);

    isTransferDone = false;
    FLEXCOM2_SPI_WriteRead(&txData, 2, &rxData, 2);

    //Wait end of transmission
    while (isTransferDone == false);
}

// *****************************************************************************
/* Function:
    static bool LX7730_SPI_ReadReg(uint8_t reg, uint8_t* readData)

   Summary:
    Write value to LX7730 register.

   Parameters:
    reg - LX7730 register to write.
    readData - Pointer to store the read value read of the register.

   Returns:
    True if SPI read without error, False otherwise.
*/
static bool LX7730_SPI_ReadReg(uint8_t reg, uint8_t* readData)
{
    bool res = true;
    uint16_t txData;
    uint16_t rxData;
    uint8_t readRegAddr = 0;

    txData = LX7730_SPI_EncodeFrame(reg, 0, false);

    // Write read register command
    isTransferDone = false;
    FLEXCOM2_SPI_WriteRead(&txData, 2, &rxData, 2);

    //Wait end of transmission
    while (isTransferDone == false);
    
    // Dummy data : read register 0
    txData = LX7730_SPI_EncodeFrame(0, 0, false);
    
    //Get last read register with dummy write
    isTransferDone = false;
    FLEXCOM2_SPI_WriteRead(&txData, 2, &rxData, 2);
    
    //Wait end of transmission
    while (isTransferDone == false);

    LX7730_SPI_DecodeFrame(rxData, &readRegAddr, readData);

    if (readRegAddr != reg)
    {
        printf("ERROR wrong reg address read=0x%02X!=0x%02X\r\n", readRegAddr, reg);
        res = false;
    }
    
    return res;
}

// *****************************************************************************
/* Function:
    static bool LX7730_init(void)

   Summary:
    Initialize SPI communication with LX7730.

   Parameters:
    None.

   Returns:
    True if initialization performed without error, False otherwise.
*/
static bool LX7730_init(void)
{
    bool res = true;
    uint8_t reg01 = 0; 

    // Reset LX7730 chip
    LX7730_RST_Clear();
    SYSTICK_DelayMs(10);
    LX7730_RST_Set();

    // Test SPI connection by reading the Function Enable register (0x01), should be equal to 0xFF
    res = LX7730_SPI_ReadReg(LX7730_SPI_FUNC_EN_REG, &reg01);

    if (res && (reg01 == 0xFF) )
    {
        // Write 0xE9 to Function Enable register (0x01)
        LX7730_SPI_WriteReg(LX7730_SPI_FUNC_EN_REG, 0xE9);

        SYSTICK_DelayMs(1);

        // Setting the SE_RTN bit of register Inverting Mux Channel
        LX7730_SPI_WriteReg(LX7730_SPI_INV_MUX_CH_REG, 0x41);

        SYSTICK_DelayMs(1);

        // Setting the AUTO_CONV bit of register ADC Control for continuous conversion of ADC
        LX7730_SPI_WriteReg(LX7730_SPI_ADC_CTRL_REG, 0x10);
    }

    return res;
}

// *****************************************************************************
/* Function:
    static void LX7730_ReadSensorData(uint8_t channelNo, uint8_t *msb_val, uint8_t *lsb_val)

   Summary:
    Read ADC values (MSB and LSB) for the given channel.

   Parameters:
    channelNo - LX7730 channel selected.
    msb_val - Pointer to store the read ADC MSB value.
    lsb_val - Pointer to store the read ADC LSB value.

   Returns:
    None.
*/
static void LX7730_ReadSensorData(uint8_t channelNo, uint8_t *msb_val, uint8_t *lsb_val)
{
    // Writing into Non-inverting channel
    LX7730_SPI_WriteReg(LX7730_SPI_NON_INV_MUX_CH_REG, channelNo);

    SYSTICK_DelayMs(1);

    LX7730_SPI_ReadReg(LX7730_SPI_ADC_RES_LSB_REG, lsb_val);

    SYSTICK_DelayMs(1);

    LX7730_SPI_ReadReg(LX7730_SPI_ADC_RES_MSB_REG, msb_val);
}

// *****************************************************************************
/* Function:
    static void UART_INTF_SendLastReadRegister(void)

   Summary:
    Send the last read register value to the UART remote interface.

   Parameters:
    None.

   Returns:
    None.
*/
static void UART_INTF_SendLastReadRegister(void)
{
    char dataOut[18] = {0xAA, 0xBB, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    dataOut[3] = lastRegisterRead;
    FLEXCOM5_USART_Write(dataOut,sizeof(dataOut));
}

// *****************************************************************************
/* Function:
    static void UART_INTF_SendLastReadRegister(void)

   Summary:
    Send the sensors read values to the UART remote interface.

   Parameters:
    None.

   Returns:
    None.
*/
static void UART_INTF_SendReadSensors(void)
{
    char dataOut[18] = {0xAA, 0xBB, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    dataOut[2] = (appSensorTempScaled >> 8) & 0xFF;
    dataOut[3] = appSensorTempScaled & 0xFF;
    dataOut[4] = (appSensorPressure >> 8) & 0xFF;
    dataOut[5] = appSensorPressure & 0xFF;
    dataOut[6] = (appSensorDistance >> 8) & 0xFF;
    dataOut[7] = appSensorDistance & 0xFF;
    dataOut[8] = (appSensorMagnScaled >> 8) & 0xFF;
    dataOut[9] = appSensorMagnScaled & 0xFF;
    dataOut[10] = (appSensorLight >> 8) & 0xFF;
    dataOut[11] = appSensorLight & 0xFF;
    dataOut[12] = (appSensorAccXScaled >> 8) & 0xFF;
    dataOut[13] = appSensorAccXScaled & 0xFF;
    dataOut[14] = (appSensorAccYScaled >> 8) & 0xFF;
    dataOut[15] = appSensorAccYScaled & 0xFF;
    dataOut[16] = (appSensorAccZScaled >> 8) & 0xFF;
    dataOut[17] = appSensorAccZScaled & 0xFF;

    FLEXCOM5_USART_Write(dataOut,sizeof(dataOut));
}

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    char dataIn[8] = {0};
    uint8_t readMsb, readLsb = 0;
    uint8_t tmpRegisterRead = 0;

    /* Initialize all modules */
    SYS_Initialize ( NULL );

    SYSTICK_TimerStart();

    FLEXCOM2_SPI_CallbackRegister(SPIEventHandler, (uintptr_t) 0);
  
    bool resInit = LX7730_init();

    if (resInit)
    {
        while ( true )
        {
            while(FLEXCOM5_USART_Read((void*)&dataIn, 8) != true);

            uint32_t data = (dataIn[4] << 24) + (dataIn[5] << 16) + (dataIn[6] << 8) + dataIn[7];

            if ( (dataIn[0] == 0xAA) && (dataIn[1] == 0xBB) )
            {
                if ( dataIn[3] == UART_INTF_READ_ADDR)
                {
                    if ( (data & 0x1) == 0x1)
                    {
                        uint8_t reg = (data >> 24) & 0x1F;
                        printf("Read SPI reg : %d\r\n", reg);

                        LX7730_SPI_ReadReg(reg, &tmpRegisterRead);
                        lastRegisterRead = tmpRegisterRead;
                    }
                    else if ( (data & 0x1) == 0)
                    {
                        //Write previous register read to UART interface
                        UART_INTF_SendLastReadRegister();
                    }
                }
                if ( dataIn[3] == UART_INTF_GET_ADC_DATA)
                {
                    if ( (data & 0x1) == 0x1)
                    {
                        printf("Read SPI SENSORS\r\n");

                        // Reading Light Sensor Data
                        LX7730_ReadSensorData(LX7730_MUX_CH57, &readMsb, &readLsb);
                        appSensorLight = ((uint16_t)readMsb << 8) | readLsb;

                        // Reading Distance Sensor Data
                        LX7730_ReadSensorData(LX7730_MUX_CH58, &readMsb, &readLsb);
                        appSensorDistance = ((uint16_t)readMsb << 8) | readLsb;

                        // Reading Pressure Sensor Data
                        LX7730_ReadSensorData(LX7730_MUX_CH59, &readMsb, &readLsb);
                        appSensorPressure = ((uint16_t)readMsb << 8) | readLsb;

                        // Reading Pressure Sensor Data
                        LX7730_ReadSensorData(LX7730_MUX_CH60, &readMsb, &readLsb);
                        uint16_t appSensorTempRaw = ((uint16_t)readMsb << 4) | readLsb;
                        appSensorTempScaled = ((appSensorTempRaw * 10000) >> 13) - 40;

                        // Reading Magnetic Sensor Data
                        LX7730_ReadSensorData(LX7730_MUX_CH61, &readMsb, &readLsb);
                        uint16_t appSensorMagnRaw = ((uint16_t)readMsb << 4) | readLsb;
                        appSensorMagnScaled = ((5 * (uint32_t)appSensorMagnRaw - 100) * 222) >> 12;

                        // Reading Z-axis Accelerometer Data
                        LX7730_ReadSensorData(LX7730_MUX_CH62, &readMsb, &readLsb);
                        uint16_t appSensorAccZRaw = ((uint16_t)readMsb << 4) | readLsb;
                        appSensorAccZScaled = ((appSensorAccZRaw - 1350) * (2250/535));

                        // Reading Y-axis Accelerometer Data
                        LX7730_ReadSensorData(LX7730_MUX_CH63, &readMsb, &readLsb);
                        uint16_t appSensorAccYRaw = ((uint16_t)readMsb << 4) | readLsb;
                        appSensorAccYScaled = ((appSensorAccYRaw - 1350) * (2250/510));

                        // Reading X-axis Accelerometer Data
                        LX7730_ReadSensorData(LX7730_MUX_CH64, &readMsb, &readLsb);
                        uint16_t appSensorAccXRaw = ((uint16_t)readMsb << 4) | readLsb;
                        appSensorAccXScaled = ((appSensorAccXRaw - 1350) * (2250/512));

                        //Write sensor read to interface
                        UART_INTF_SendReadSensors();
                    }
                }
            }
        }
    }
    else
    {
        printf("[ERROR] Failed to initialize LX7730\r\n");
    }
    
    while(1);

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

