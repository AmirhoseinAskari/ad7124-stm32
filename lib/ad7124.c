/**
 * @file    ad7124.c
 * @author  Amirhossein Askari
 * @version 1.0.0
 * @date    2025-06-15
 * @email   theamiraskarii@gmail.com
 * @see     https://github.com/AmirhoseinAskari
 * @brief   Interrupt-driven driver for the AD7124-4 24-bit Sigma-Delta ADC using STM32 HAL.
 *
 * @details
 * This driver enables communication with the AD7124-4 ADC through interrupt-driven SPI transactions.
 * The ADC's DOUT/RDY pin must be connected to an **external interrupt (EXTI)** line on the STM32.
 *
 * The external interrupt must be configured as a **pull-up input** with **falling-edge trigger**,
 * and must be properly enabled in the **NVIC** prior to using this driver.
 *
 * Configure SPI with the following settings:
 * - Clock polarity (CPOL) = 1 (idle high)
 * - Clock phase    (CPHA) = 1 (data captured on second clock edge)
 *
 * The SPI peripheral and all GPIOs (chip select, interrupt lines) must be initialized externally
 * before using the functions provided in this driver.
 *
 * @note
 * - Set bit 2 (SPI_CRC_ERR_EN) in the AD7124's `error_enable` register to enable CRC error detection.
 * - This driver assumes correct configuration of SPI and external interrupt peripherals.
 *
 * @warning
 * - Improper SPI or EXTI configuration may lead to data loss or undefined ADC behavior.
 * - Ensure the DOUT/RDY pin is correctly connected and configured to generate falling-edge interrupts.
 *
 * @section Supported_Platforms
 * STM32 microcontrollers using HAL drivers.
 *
 * @copyright
 * MIT License. See LICENSE file for details.
 */


/* ------------------------------------- Includes ------------------------------------- */

#include "ad7124.h"  /**< Header file declaring the AD7124 driver functions */



/* ------------------------------------- Variables ------------------------------------ */

/**
 * @brief Global handler for AD7124 configuration and state.
 *
 * This structure contains all configuration parameters and the current
 * operational state of the AD7124 ADC.
 *
 * @note Use @c extern AD7124_ConfigTypeDef AD7124_Handler; in main.c file to access it.
 */
AD7124_ConfigTypeDef AD7124_Handler;

/**
 * @brief Storage for ADC channel samples.
 *
 * This array holds the sampled data for each enabled ADC channel.
 * The size is defined by @c AD7124_ENABLED_CHANNELS.
 *
 * @note Use @c extern uint32_t AD7124_ChannelSamples[]; to access the data externally.
 */
uint32_t AD7124_ChannelSamples[AD7124_ENABLED_CHANNELS] = {0U};

/**
 * @brief  CRC8 Lookup Table.
 *
 * This lookup table is used to calculate the CRC8 checksum,
 * which ensures data integrity during communication.
 */
static const uint8_t CRC8_LOOKUP_TABLE[256U] = {
    0x00U, 0x07U, 0x0EU, 0x09U, 0x1CU, 0x1BU, 0x12U, 0x15U,
    0x38U, 0x3FU, 0x36U, 0x31U, 0x24U, 0x23U, 0x2AU, 0x2DU,
    0x70U, 0x77U, 0x7EU, 0x79U, 0x6CU, 0x6BU, 0x62U, 0x65U,
    0x48U, 0x4FU, 0x46U, 0x41U, 0x54U, 0x53U, 0x5AU, 0x5DU,
    0xE0U, 0xE7U, 0xEEU, 0xE9U, 0xFCU, 0xFBU, 0xF2U, 0xF5U,
    0xD8U, 0xDFU, 0xD6U, 0xD1U, 0xC4U, 0xC3U, 0xCAU, 0xCDU,
    0x90U, 0x97U, 0x9EU, 0x99U, 0x8CU, 0x8BU, 0x82U, 0x85U,
    0xA8U, 0xAFU, 0xA6U, 0xA1U, 0xB4U, 0xB3U, 0xBAU, 0xBDU,
    0xC7U, 0xC0U, 0xC9U, 0xCEU, 0xDBU, 0xDCU, 0xD5U, 0xD2U,
    0xFFU, 0xF8U, 0xF1U, 0xF6U, 0xE3U, 0xE4U, 0xEDU, 0xEAU,
    0xB7U, 0xB0U, 0xB9U, 0xBEU, 0xABU, 0xACU, 0xA5U, 0xA2U,
    0x8FU, 0x88U, 0x81U, 0x86U, 0x93U, 0x94U, 0x9DU, 0x9AU,
    0x27U, 0x20U, 0x29U, 0x2EU, 0x3BU, 0x3CU, 0x35U, 0x32U,
    0x1FU, 0x18U, 0x11U, 0x16U, 0x03U, 0x04U, 0x0DU, 0x0AU,
    0x57U, 0x50U, 0x59U, 0x5EU, 0x4BU, 0x4CU, 0x45U, 0x42U,
    0x6FU, 0x68U, 0x61U, 0x66U, 0x73U, 0x74U, 0x7DU, 0x7AU,
    0x89U, 0x8EU, 0x87U, 0x80U, 0x95U, 0x92U, 0x9BU, 0x9CU,
    0xB1U, 0xB6U, 0xBFU, 0xB8U, 0xADU, 0xAAU, 0xA3U, 0xA4U,
    0xF9U, 0xFEU, 0xF7U, 0xF0U, 0xE5U, 0xE2U, 0xEBU, 0xECU,
    0xC1U, 0xC6U, 0xCFU, 0xC8U, 0xDDU, 0xDAU, 0xD3U, 0xD4U,
    0x69U, 0x6EU, 0x67U, 0x60U, 0x75U, 0x72U, 0x7BU, 0x7CU,
    0x51U, 0x56U, 0x5FU, 0x58U, 0x4DU, 0x4AU, 0x43U, 0x44U,
    0x19U, 0x1EU, 0x17U, 0x10U, 0x05U, 0x02U, 0x0BU, 0x0CU,
    0x21U, 0x26U, 0x2FU, 0x28U, 0x3DU, 0x3AU, 0x33U, 0x34U,
    0x4EU, 0x49U, 0x40U, 0x47U, 0x52U, 0x55U, 0x5CU, 0x5BU,
    0x76U, 0x71U, 0x78U, 0x7FU, 0x6AU, 0x6DU, 0x64U, 0x63U,
    0x3EU, 0x39U, 0x30U, 0x37U, 0x22U, 0x25U, 0x2CU, 0x2BU,
    0x06U, 0x01U, 0x08U, 0x0FU, 0x1AU, 0x1DU, 0x14U, 0x13U,
    0xAEU, 0xA9U, 0xA0U, 0xA7U, 0xB2U, 0xB5U, 0xBCU, 0xBBU,
    0x96U, 0x91U, 0x98U, 0x9FU, 0x8AU, 0x8DU, 0x84U, 0x83U,
    0xDEU, 0xD9U, 0xD0U, 0xD7U, 0xC2U, 0xC5U, 0xCCU, 0xCBU,
    0xE6U, 0xE1U, 0xE8U, 0xEFU, 0xFAU, 0xFDU, 0xF4U, 0xF3U
};



/* ------------------------------------- Functions ------------------------------------ */

/**
 * @brief Compute CRC8 checksum using lookup table.
 *
 * This function calculates the CRC8 checksum over a buffer using a precomputed lookup table.
 * It is used for data integrity checks during SPI communication with the AD7124-4 ADC.
 *
 * @param[in] pBuf     Pointer to the data buffer.
 * @param[in] bufSize  Number of bytes in the buffer.
 *
 * @retval CRC8 checksum value of the input buffer.
 * @retval 0 If @p pBuf is NULL or @p bufSize is 0.
 */
static inline uint8_t AD7124_ComputeCRC8(const uint8_t *pBuf, uint8_t bufSize)
{
    uint8_t crc = 0U;
       
    if ( (pBuf != NULL) && (bufSize > 0U) )
    {
        for (uint8_t index = 0U; index < bufSize; index++)
        {
            crc = CRC8_LOOKUP_TABLE[crc ^ pBuf[index]];
        }
    }
    
    return crc; 
}

/**
 * @brief Reads a register value from the AD7124-4 via SPI.
 *
 * Performs an SPI transaction to read `dataSize` bytes from the specified
 * AD7124 register at `regAddr`. The read data is validated using CRC8 to ensure communication integrity.
 *
 * @param[in]  pADC       Pointer to the AD7124-4 configuration structure containing SPI settings and communication parameters.
 * @param[in]  regAddr    Address of the target register to read.
 * @param[in]  dataSize   Number of bytes to read from the register.
 * @param[out] pRegValue  Pointer to a variable where the read register value will be stored.
 *
 * @retval AD7124_OK      Register read completed successfully.
 * @retval AD7124_ERROR   Communication failure, CRC mismatch, or invalid parameters.
 */
AD7124_StatusTypeDef AD7124_ReadRegister(const AD7124_ConfigTypeDef *pADC, uint8_t regAddr, uint8_t dataSize, uint32_t *pRegValue)
{
    AD7124_StatusTypeDef status = AD7124_ERROR;
    uint32_t data = 0U;
    uint8_t rx_buf [8U] = {0U};
    uint8_t msg_buf[8U] = {0U};
    uint8_t *rx_ptr = NULL;
    uint8_t msg_length = 1U;
    
    // Build the Command word
    uint8_t tx_buf = (uint8_t)(0x40U | (regAddr & 0x3FU));                     
    uint8_t add_status_length = (regAddr == AD7124_DATA_REG) ? 1U : 0U;  
    uint8_t total_bytes_to_receive = (uint8_t)(2U + dataSize + add_status_length); 
           
    if ( (pADC == NULL) || (dataSize == 0U) )
    {
        status = AD7124_ERROR;  
    }
    else
    {  
        // SPI write transaction
        HAL_GPIO_WritePin(pADC->csPort, pADC->csPin, GPIO_PIN_RESET);
        status = (AD7124_StatusTypeDef) HAL_SPI_TransmitReceive(pADC->SPIx, &tx_buf, rx_buf, total_bytes_to_receive, AD7124_MAX_DELAY);
        HAL_GPIO_WritePin(pADC->csPort, pADC->csPin, GPIO_PIN_SET);
 
        if (status == AD7124_OK)
        {
            msg_buf[0U] = tx_buf;  // Store command byte for CRC calculation
            rx_ptr = &rx_buf[1U];  // Pointer to received data (skip first byte: command echo)
            
            // Copy received bytes (data + status if any) for CRC calculation
            for (uint8_t i = 0U; i < (uint8_t)(dataSize + add_status_length); i++) 
            {
                msg_buf[msg_length++] = *rx_ptr++;
            }
            
            // Compute CRC8 of received message (command + data)
            uint8_t crc_calculated = AD7124_ComputeCRC8(msg_buf, msg_length);   
            uint8_t crc_received   = rx_buf[total_bytes_to_receive - 1U];
            
            if (crc_calculated == crc_received)
            {
                rx_ptr = &rx_buf[1U];
                
                // Build the result
                for (uint8_t i = 0U; i < dataSize; i++)
                {
                    data = (data << 8U) | *rx_ptr++;
                }

                *pRegValue = data; 
            }
            else
            {
                status = AD7124_ERROR;  
            }
        }
    }
    
    return status;
}

/**
 * @brief Writes data to a specified register of the AD7124-4 ADC via SPI.
 *
 * Performs an SPI transaction to write `dataSize` bytes of `regValue` to the AD7124 register specified by `regAddr`.
 *
 * @param[in] pADC      Pointer to the AD7124-4 configuration structure containing SPI settings and communication parameters.
 * @param[in] regAddr   8-bit address of the register to write to.
 * @param[in] dataSize  Number of bytes to write to the register.
 * @param[in] regValue  Value to write into the register.
 *
 * @retval AD7124_OK     Register write completed successfully.
 * @retval AD7124_ERROR  Communication failure, CRC mismatch, or invalid parameters.
 */
AD7124_StatusTypeDef AD7124_WriteRegister(const AD7124_ConfigTypeDef *pADC, uint8_t regAddr, uint8_t dataSize, uint32_t regValue)
{
    AD7124_StatusTypeDef status = AD7124_ERROR;
    uint8_t tx_buf[8U] = {0U};
    uint8_t crc = 0U;
    
    if ( (pADC == NULL) || (dataSize == 0U) )
    {
        status = AD7124_ERROR;  
    }
    else
    {
        // Build the Command word
        tx_buf[0U] = (uint8_t)(regAddr & 0x3FU);
        
        // Fill the write buffer
        for (uint8_t i = 0U; i < dataSize; i++)
        {
            tx_buf[i + 1U] = (uint8_t)( (regValue >> (8U * (dataSize - 1U - i) ) ) & 0xFFU);
        }
        
        // Compute the CRC
        crc = AD7124_ComputeCRC8(tx_buf, (uint8_t)(dataSize + 1U));  
        tx_buf[dataSize + 1U] = crc;  
        
        // SPI write transaction
        HAL_GPIO_WritePin(pADC->csPort, pADC->csPin, GPIO_PIN_RESET);
        status = (AD7124_StatusTypeDef) HAL_SPI_Transmit(pADC->SPIx, tx_buf, (dataSize + 2U), AD7124_MAX_DELAY);
        HAL_GPIO_WritePin(pADC->csPort, pADC->csPin, GPIO_PIN_SET);
    }
    
    return status;
}

/**
 * @brief Performs a hardware reset of the AD7124-4 ADC via SPI.
 *
 * Sends the predefined reset sequence over SPI to the AD7124 ADC, restoring all registers to their default reset values.
 *
 * @param[in] pADC     Pointer to the AD7124-4 configuration structure containing SPI settings and communication parameters.
 * @param[in] timeout  Maximum time (in milliseconds) to wait for the reset operation to complete.
 *
 * @retval AD7124_OK      Reset completed successfully.
 * @retval AD7124_ERROR   Communication failure, CRC mismatch, or invalid parameters.
 * @retval AD7124_TIMEOUT The reset operation timed out before completion.
 */
AD7124_StatusTypeDef AD7124_Reset(const AD7124_ConfigTypeDef *pADC, uint32_t timeout)
{  
    AD7124_StatusTypeDef status = AD7124_ERROR;
    const uint8_t reset_sequence[8U] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};  
    uint32_t start_time   = 0U;
    uint32_t status_value = 0U;        
    
    if ( (pADC == NULL) || (timeout == 0U) )
    {
        status = AD7124_ERROR; 
    }
    else
    { 
        // SPI write transaction
        HAL_GPIO_WritePin(pADC->csPort, pADC->csPin, GPIO_PIN_RESET);
        status = (AD7124_StatusTypeDef) HAL_SPI_Transmit(pADC->SPIx, reset_sequence, 8U, AD7124_MAX_DELAY);
        HAL_GPIO_WritePin(pADC->csPort, pADC->csPin, GPIO_PIN_SET);

        if (status == AD7124_OK)
        {           
            start_time = HAL_GetTick();        
            do 
            {
                // Wait for STATUS register until RDY bit is cleared or timeout expires
                if (AD7124_ReadRegister(pADC, AD7124_STATUS_REG, 1U, &status_value) != AD7124_OK)
                {
                    status = AD7124_ERROR;
                    break;
                }
            } while ( ( (status_value & 0x10U) != 0U) && ( (HAL_GetTick() - start_time) < timeout) );

            if ( (status_value & 0x10U) != 0U)
            {
                status = AD7124_TIMEOUT; 
            }
            else
            {               
                HAL_Delay(4U);  // Allow ADC stabilization               
                status = AD7124_WriteRegister(pADC, AD7124_ERREN_REG, 3U, 0x00004U);  // Enable CRC error detection
            }
        }      
    }
    
    return status;
}

/**
 * @brief Initializes and configures the AD7124-4 ADC registers.
 *
 * Writes predefined settings to the ADC's channel mapping, configuration,
 * and filter registers to prepare the device for data acquisition.
 *
 * @param[in] pADC     Pointer to the AD7124-4 configuration structure containing SPI settings and communication parameters.
 * @param[in] pConfig  Pointer to the AD7124-4 register configuration structure specifying
 *                    the desired settings for channels, filters, and configurations.
 *
 * @retval AD7124_OK     Registers configured successfully.
 * @retval AD7124_ERROR  Communication failure, CRC mismatch, or invalid parameters.
 */
AD7124_StatusTypeDef AD7124_Config(const AD7124_ConfigTypeDef *pADC, AD7124_RegisterTypeDef *pConfig)
{
    AD7124_StatusTypeDef status = AD7124_OK;
    
    static const uint8_t channelRegs[8U] =
    {
        AD7124_CH0_MAP_REG, AD7124_CH1_MAP_REG, AD7124_CH2_MAP_REG,
        AD7124_CH3_MAP_REG, AD7124_CH4_MAP_REG, AD7124_CH5_MAP_REG,
        AD7124_CH6_MAP_REG, AD7124_CH7_MAP_REG
    };

    static const uint8_t configRegs[8U] =
    {
        AD7124_CFG0_REG, AD7124_CFG1_REG, AD7124_CFG2_REG,
        AD7124_CFG3_REG, AD7124_CFG4_REG, AD7124_CFG5_REG,
        AD7124_CFG6_REG, AD7124_CFG7_REG
    };
    
    static const uint8_t filterRegs[8U] =
    {
        AD7124_FILT0_REG, AD7124_FILT1_REG, AD7124_FILT2_REG,
        AD7124_FILT3_REG, AD7124_FILT4_REG, AD7124_FILT5_REG,
        AD7124_FILT6_REG, AD7124_FILT7_REG
    };
    
    if ( (pADC == NULL) || (pConfig == NULL) )
    {
        status = AD7124_ERROR;
    }
    else
    {
        for (int8_t i = AD7124_CHANNEL_COUNT-1; i >= 0; i--)
        {
            if (AD7124_WriteRegister(pADC, channelRegs[i], 2U, pConfig->channels[i]) != AD7124_OK ||
                AD7124_WriteRegister(pADC, configRegs[i],  2U, pConfig->configs[i])  != AD7124_OK ||
                AD7124_WriteRegister(pADC, filterRegs[i],  3U, pConfig->filters[i])  != AD7124_OK)
            {
                status = AD7124_ERROR;
                break;
            }
        }

        if (status == AD7124_OK)
        {
            if (AD7124_WriteRegister(pADC, AD7124_CONTROL_REG,  2U, pConfig->adc_control)  != AD7124_OK ||
                AD7124_WriteRegister(pADC, AD7124_IO_CTRL1_REG, 3U, pConfig->io_control_1) != AD7124_OK ||
                AD7124_WriteRegister(pADC, AD7124_IO_CTRL2_REG, 2U, pConfig->io_control_2) != AD7124_OK ||
                AD7124_WriteRegister(pADC, AD7124_ERREN_REG,    3U, pConfig->error_enable) != AD7124_OK)
            {
                status = AD7124_ERROR;
            }
        }
    }
    
    return status;
}

/**
 * @brief Reads ADC conversion data from the AD7124-4 and updates the channel samples array.
 *
 * Retrieves the latest conversion result from the AD7124-4 over SPI and stores the sample
 * value in the corresponding element of the external `AD7124_ChannelSamples` array.
 * Supports reading from multiple enabled and configured ADC channels.
 *
 * @param[in] pADC  Pointer to the AD7124 configuration structure containing SPI settings and communication parameters.
 *
 * @retval AD7124_OK     Sample data read successfully.
 * @retval AD7124_ERROR  Communication failure, CRC mismatch, or invalid parameters.
 *
 * @note The `AD7124_ChannelSamples` array must be defined and managed by the user.
 */
AD7124_StatusTypeDef AD7124_ReadSampleData(const AD7124_ConfigTypeDef *pADC)
{
    AD7124_StatusTypeDef status = AD7124_OK;
    uint32_t  sample_data  = 0U;
    uint32_t  status_reg   = 0U;
    uint8_t active_channel = 0U;
    
    if (pADC == NULL)
    {
        status = AD7124_ERROR;
    }
    else
    {
        // Read samples for the number of enabled channels
        for (uint8_t i = 0U; i < AD7124_ENABLED_CHANNELS; i++)
        {
            // Read ADC data and status registers
            if ( (AD7124_ReadRegister(pADC, AD7124_DATA_REG, 3U, &sample_data) != AD7124_OK) || 
                 (AD7124_ReadRegister(pADC, AD7124_STATUS_REG, 1U, &status_reg) != AD7124_OK) )
            {
                status = AD7124_ERROR;
                break;
            }
            
            // Extract active channel index (lower 4 bits)
            active_channel = (uint8_t)(status_reg & 0x0FU);
            
            // Store sample if channel index is valid
            if (active_channel < AD7124_ENABLED_CHANNELS)
            {
                AD7124_ChannelSamples[active_channel] = sample_data;
            }
        }
    }

    return status;
}

/**
 * @brief Checks the AD7124-4 ADC error status by reading the error register.
 *
 * Reads the error register of the AD7124-4 via SPI to detect any ADC errors.
 * The error status is stored at the location pointed to by `pErrorReg`.
 * If the read operation fails, `*pErrorReg` is set to -1 to indicate failure.
 *
 * @param[in]  pADC       Pointer to the AD7124 configuration structure containing SPI settings and communication parameters.
 * @param[out] pErrorReg  Pointer to an integer where the error register value will be stored.
 *
 * @retval AD7124_OK      Error register read successfully.
 * @retval AD7124_ERROR   Communication failure, CRC mismatch, or invalid parameters.
 */
AD7124_StatusTypeDef AD7124_ErrorCheck(const AD7124_ConfigTypeDef *pADC, int32_t *pErrorReg)
{
    AD7124_StatusTypeDef status = AD7124_ERROR;
    uint32_t data = 0U;
    
    if (pADC == NULL)
    {
        status = AD7124_ERROR;
    }
    else
    {     
        if (AD7124_ReadRegister(pADC, AD7124_ERR_REG, 3U, &data) == AD7124_OK)
        {
            *pErrorReg = (int32_t)data;
            status = AD7124_OK;
        }
        else
        {
            *pErrorReg = -1;
            status = AD7124_ERROR;
        }
    }
    
    return status;
}

/**
 * @brief Performs gain and offset calibration on the AD7124-4 ADC.
 *
 * Executes internal full-scale and zero-scale calibrations to improve accuracy of the AD7124.
 * Reads the updated gain and offset coefficients from the device and updates the provided
 * configuration structure accordingly. After calibration, the ADC settings are restored to
 * their default operational values.
 *
 * @param[in]     pADC     Pointer to the AD7124-4 configuration structure
 *                        containing SPI settings and communication parameters.
 * @param[in,out] pConfig  Pointer to the register configuration structure, which will be
 *                        updated with calibrated gain and offset values upon success.
 * @param[in]     timeout  Maximum time (in ticks) to wait for each calibration step to complete.
 *
 * @retval AD7124_OK       Calibration completed successfully.
 * @retval AD7124_ERROR    Communication failure, CRC mismatch, or invalid parameters.
 * @retval AD7124_TIMEOUT  Calibration timed out before completion.
 */
AD7124_StatusTypeDef AD7124_Calibration(const AD7124_ConfigTypeDef *pADC, AD7124_RegisterTypeDef *pConfig, uint32_t timeout)
{
    AD7124_StatusTypeDef status = AD7124_OK;
    uint16_t default_config = pConfig->adc_control; 
    uint32_t start_tick = 0U;
    uint32_t  data      = 0U;
       
    if ( (pADC == NULL) || (pConfig == NULL) || (timeout == 0U) )
    {
        status = AD7124_ERROR;
    }
    else
    {    
        // Internal full-scale (gain) calibration
        pConfig->adc_control = (pConfig->adc_control & ~AD7124_CONTROL_MODE_MASK) | AD7124_CONTROL_CAL_GAIN;       
        if (AD7124_WriteRegister(pADC, AD7124_CONTROL_REG, 2U, pConfig->adc_control) != AD7124_OK)
        {
            status = AD7124_ERROR;
        }
        else
        {
            start_tick = HAL_GetTick();           
            // Wait until calibration completes or timeout occurs
            do
            {
                if (AD7124_ReadRegister(pADC, AD7124_STATUS_REG, 1U, &data) != AD7124_OK)
                {
                    status = AD7124_ERROR;
                    break;
                }
                if ( (HAL_GetTick() - start_tick) >= timeout)
                {
                    status = AD7124_TIMEOUT;
                    break;
                }
            } while ( (data & 0x80U) != 0U);
        }

        if (status == AD7124_OK)
        {
            // Internal zero-scale (offset) calibration
            pConfig->adc_control = (pConfig->adc_control & ~AD7124_CONTROL_MODE_MASK) | AD7124_CONTROL_CAL_OFFSET;
            if (AD7124_WriteRegister(pADC, AD7124_CONTROL_REG, 2U, pConfig->adc_control) != AD7124_OK)
            {
                status = AD7124_ERROR;
            }
            else
            {
                start_tick = HAL_GetTick();              
                // Wait until calibration completes or timeout occurs
                do
                {
                    if (AD7124_ReadRegister(pADC, AD7124_STATUS_REG, 1U, &data) != AD7124_OK)
                    {
                        status = AD7124_ERROR;
                        break;
                    }
                    if ((HAL_GetTick() - start_tick) >= timeout)
                    {
                        status = AD7124_TIMEOUT;
                        break;
                    }
                } while ( (data & 0x80U) != 0U);
            }
        }

        if (status == AD7124_OK)
        {
            // Standby mode to read/write calibration coefficients
            pConfig->adc_control = (pConfig->adc_control & ~AD7124_CONTROL_MODE_MASK) | AD7124_CONTROL_STANDBY;
            if (AD7124_WriteRegister(pADC, AD7124_CONTROL_REG, 2U, pConfig->adc_control) != AD7124_OK)
            {
                status = AD7124_ERROR;
            }
        }

        if (status == AD7124_OK)
        {
            // Read gain coefficients after calibration
            for (uint8_t i = 0U; i < 8U; i++)
            {
                if (AD7124_ReadRegister(pADC, AD7124_GAIN0_REG + i, 3U, &pConfig->gains[i]) != AD7124_OK)
                {
                    status = AD7124_ERROR;
                    break;
                }
            }
        }

        if (status == AD7124_OK)
        {
            // Read offset coefficients after calibration
            for (uint8_t i = 0U; i < 8U; i++)
            {
                if (AD7124_ReadRegister(pADC, AD7124_OFFS0_REG + i, 3U, &pConfig->offsets[i]) != AD7124_OK)
                {
                    status = AD7124_ERROR;
                    break;
                }
            }
        }

        if (status == AD7124_OK)
        {
            // Write calibrated gain coefficients
            for (uint8_t i = 0U; i < 8U; i++)
            {
                if (AD7124_WriteRegister(pADC, AD7124_GAIN0_REG + i, 3U, pConfig->gains[i]) != AD7124_OK)
                {
                    status = AD7124_ERROR;
                    break;
                }
            }
        }

        if (status == AD7124_OK)
        {
            // Write calibrated offset coefficients
            for (uint8_t i = 0U; i < 8U; i++)
            {
                if (AD7124_WriteRegister(pADC, AD7124_OFFS0_REG + i, 3U, pConfig->offsets[i]) != AD7124_OK)
                {
                    status = AD7124_ERROR;
                    break;
                }
            }
        }

        if (status == AD7124_OK)
        {
            // Restore original ADC control configuration
            if (AD7124_WriteRegister(pADC, AD7124_CONTROL_REG, 2U, default_config) != AD7124_OK)
            {
                status = AD7124_ERROR;
            }
        }
    } 

    return status; 
}


/* ad7124.c */
