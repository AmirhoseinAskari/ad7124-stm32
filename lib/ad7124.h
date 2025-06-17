/**
 * @file    ad7124.h
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


#ifndef AD7124_H
#define AD7124_H


#ifdef __cplusplus
extern "C" {
#endif
    

/* ------------------------------------- Includes ------------------------------------- */

#include <stdint.h>         /**< Standard library for fixed-width integer types */
#include <stddef.h>         /**< Standard library for NULL definition */
#include "ad7124_config.h"  /**< Project-specific configuration header */


/**
 * @brief Include STM32 HAL header based on target MCU series.
 *
 * This conditional compilation block includes the correct STM32 HAL driver header
 * depending on the STM32 series macro defined in `ad7124_config.h`.
 *
 * @note Exactly one `_STM32xx` macro must be defined for proper HAL inclusion.
 */
#if defined (_STM32F0)                 
    #include "stm32f0xx_hal.h"   
#elif defined (_STM32F1)
    #include "stm32f1xx_hal.h"  
#elif defined (_STM32F2)
    #include "stm32f2xx_hal.h" 
#elif defined (_STM32F3)
    #include "stm32f3xx_hal.h"  
#elif defined (_STM32F4)
    #include "stm32f4xx_hal.h"  
#elif defined (_STM32F7)
    #include "stm32f7xx_hal.h"   
#elif defined (_STM32G0)
    #include "stm32g0xx_hal.h" 
#elif defined (_STM32G4)
    #include "stm32g4xx_hal.h"   
#elif defined (_STM32H7)
    #include "stm32h7xx_hal.h" 
#elif defined (_STM32L0)
    #include "stm32l0xx_hal.h" 
#elif defined (_STM32L1)
    #include "stm32l1xx_hal.h" 
#elif defined (_STM32L4)
    #include "stm32l4xx_hal.h"   
#elif defined (_STM32L5)
    #include "stm32l5xx_hal.h" 
#elif defined (_STM32U0)
    #include "stm32u0xx_hal.h" 
#elif defined (_STM32U5)
    #include "stm32u5xx_hal.h" 
#else                             
    #error "STM32 microcontroller not supported. Define one of the _STM32xx macros in ad7124_config.h."
#endif


/**
 * @brief Suppress 'unused function' warnings for supported compilers.
 *
 * This preprocessor directive suppresses compiler warnings about unused static
 * or inline functions.
 * 
 * Supported compilers:
 * - IAR Embedded Workbench (ICCARM)
 * - GNU Compiler Collection (GCC)
 *
 * @note
 * If an unsupported compiler is used, compilation will fail with an error.
 */
#if defined (__ICCARM__)             
    #pragma diag_suppress = Pe177                     
#elif defined (__GNUC__)   
    #pragma GCC diagnostic ignored "-Wunused-function"     
#else
    #error "Unsupported compiler for AD7124 driver"
#endif



/* ------------------------------------- Defines -------------------------------------- */

/** @brief Number of enabled ADC channels used for sampling. */
#define  AD7124_ENABLED_CHANNELS  (4U)

/** @brief Total number of available ADC channels on the AD7124 device. */
#define  AD7124_CHANNEL_COUNT  (8U)

/** @brief Maximum delay value used for SPI transactions (timeout or wait). */
#define  AD7124_MAX_DELAY  (0xFFFFFFFFU)

/** @brief Bit mask for ADC mode field in control register. */      
#define  AD7124_CONTROL_MODE_MASK  (0x3CU)
      
/** @brief ADC mode value for internal gain calibration. */
#define  AD7124_CONTROL_CAL_GAIN  (0x06U << 2U)

/** @brief ADC mode value for internal offset calibration. */
#define  AD7124_CONTROL_CAL_OFFSET  (0x05U << 2U)
      
/** @brief ADC mode value for standby operation. */
#define  AD7124_CONTROL_STANDBY  (0x02U << 2U)

/** @brief AD7124 Register Addresses */
#define  AD7124_COMM_REG      (0x00U)  /**< Communications register */
#define  AD7124_STATUS_REG    (0x00U)  /**< Status register */
#define  AD7124_CONTROL_REG   (0x01U)  /**< ADC Control Register */
#define  AD7124_DATA_REG      (0x02U)  /**< Data register */
#define  AD7124_IO_CTRL1_REG  (0x03U)  /**< IO Control 1 register */
#define  AD7124_IO_CTRL2_REG  (0x04U)  /**< IO Control 2 register */
#define  AD7124_ID_REG        (0x05U)  /**< ID register */
#define  AD7124_ERR_REG       (0x06U)  /**< Error register */
#define  AD7124_ERREN_REG     (0x07U)  /**< Error Enable register */
#define  AD7124_MCLK_REG      (0x08U)  /**< Master clock register */
#define  AD7124_CH0_MAP_REG   (0x09U)  /**< Channel 0 mapping register */
#define  AD7124_CH1_MAP_REG   (0x0AU)  /**< Channel 1 mapping register */
#define  AD7124_CH2_MAP_REG   (0x0BU)  /**< Channel 2 mapping register */
#define  AD7124_CH3_MAP_REG   (0x0CU)  /**< Channel 3 mapping register */
#define  AD7124_CH4_MAP_REG   (0x0DU)  /**< Channel 4 mapping register */
#define  AD7124_CH5_MAP_REG   (0x0EU)  /**< Channel 5 mapping register */
#define  AD7124_CH6_MAP_REG   (0x0FU)  /**< Channel 6 mapping register */
#define  AD7124_CH7_MAP_REG   (0x10U)  /**< Channel 7 mapping register */
#define  AD7124_CFG0_REG      (0x19U)  /**< Configuration register 0 */
#define  AD7124_CFG1_REG      (0x1AU)  /**< Configuration register 1 */
#define  AD7124_CFG2_REG      (0x1BU)  /**< Configuration register 2 */
#define  AD7124_CFG3_REG      (0x1CU)  /**< Configuration register 3 */
#define  AD7124_CFG4_REG      (0x1DU)  /**< Configuration register 4 */
#define  AD7124_CFG5_REG      (0x1EU)  /**< Configuration register 5 */
#define  AD7124_CFG6_REG      (0x1FU)  /**< Configuration register 6 */
#define  AD7124_CFG7_REG      (0x20U)  /**< Configuration register 7 */
#define  AD7124_FILT0_REG     (0x21U)  /**< Filter register 0 */
#define  AD7124_FILT1_REG     (0x22U)  /**< Filter register 1 */
#define  AD7124_FILT2_REG     (0x23U)  /**< Filter register 2 */
#define  AD7124_FILT3_REG     (0x24U)  /**< Filter register 3 */
#define  AD7124_FILT4_REG     (0x25U)  /**< Filter register 4 */
#define  AD7124_FILT5_REG     (0x26U)  /**< Filter register 5 */
#define  AD7124_FILT6_REG     (0x27U)  /**< Filter register 6 */
#define  AD7124_FILT7_REG     (0x28U)  /**< Filter register 7 */
#define  AD7124_OFFS0_REG     (0x29U)  /**< Offset register 0 */
#define  AD7124_OFFS1_REG     (0x2AU)  /**< Offset register 1 */
#define  AD7124_OFFS2_REG     (0x2BU)  /**< Offset register 2 */
#define  AD7124_OFFS3_REG     (0x2CU)  /**< Offset register 3 */
#define  AD7124_OFFS4_REG     (0x2DU)  /**< Offset register 4 */
#define  AD7124_OFFS5_REG     (0x2EU)  /**< Offset register 5 */
#define  AD7124_OFFS6_REG     (0x2FU)  /**< Offset register 6 */
#define  AD7124_OFFS7_REG     (0x30U)  /**< Offset register 7 */
#define  AD7124_GAIN0_REG     (0x31U)  /**< Gain register 0 */
#define  AD7124_GAIN1_REG     (0x32U)  /**< Gain register 1 */
#define  AD7124_GAIN2_REG     (0x33U)  /**< Gain register 2 */
#define  AD7124_GAIN3_REG     (0x34U)  /**< Gain register 3 */
#define  AD7124_GAIN4_REG     (0x35U)  /**< Gain register 4 */
#define  AD7124_GAIN5_REG     (0x36U)  /**< Gain register 5 */
#define  AD7124_GAIN6_REG     (0x37U)  /**< Gain register 6 */
#define  AD7124_GAIN7_REG     (0x38U)  /**< Gain register 7 */



/* -------------------------------------- Types -------------------------------------- */

/**
 * @brief Structure for storing and configuring the AD7124 ADC register values.
 *
 * This structure holds configuration and register values required for
 * operating the AD7124 ADC device.
 */
typedef struct
{
    uint16_t adc_control;    /**< ADC Control Register (0x01) */
    uint32_t io_control_1;   /**< IO Control Register 1 (0x03) */
    uint16_t io_control_2;   /**< IO Control Register 2 (0x04) */
    uint32_t error_enable;   /**< Error Enable Register (0x07) */
    uint16_t channels[8U];   /**< Channel Mapping Registers (0x09 - 0x10), 8 entries */
    uint16_t configs[8U];    /**< Configuration Registers (0x19 - 0x20), 8 entries */
    uint32_t filters[8U];    /**< Filter Registers (0x21 - 0x28), 8 entries */
    uint32_t offsets[8U];    /**< Offset Registers (0x29 - 0x30), 8 entries */
    uint32_t gains[8U];      /**< Gain Registers (0x31 - 0x38), 8 entries */
} AD7124_RegisterTypeDef; 


/**
 * @brief Structure for configuring SPI communication settings for the AD7124 ADC.
 *
 * This structure holds all necessary parameters required to initialize and manage
 * SPI communication with the AD7124 ADC device.
 */
typedef struct
{                            
    SPI_HandleTypeDef *SPIx;  /**< Pointer to SPI handle used by STM32 HAL SPI driver */ 
    IRQn_Type IRQn;           /**< Interrupt request number for SPI interrupts */       
    GPIO_TypeDef *csPort;     /**< GPIO port for chip select (CS) pin */    
    uint16_t csPin;           /**< GPIO pin number for chip select (CS) */   
} AD7124_ConfigTypeDef;


/**
 * @brief Enumeration of possible operation statuses for the AD7124 ADC.
 *
 * This enumeration defines return codes for functions interacting with the AD7124,
 * describing the outcome of an operation such as success, error, busy state, or timeout.
 */
typedef enum
{
    AD7124_OK      = 0U,  /**< Operation completed successfully */   
    AD7124_ERROR   = 1U,  /**< Operation failed due to an error */
    AD7124_BUSY    = 2U,  /**< ADC is busy with another operation */
    AD7124_TIMEOUT = 3U   /**< Operation timed out */
} AD7124_StatusTypeDef;



/* ------------------------------------ Prototype ------------------------------------ */

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
AD7124_StatusTypeDef AD7124_ReadRegister(const AD7124_ConfigTypeDef *pADC, uint8_t regAddr, uint8_t dataSize, uint32_t *pRegValue);

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
AD7124_StatusTypeDef AD7124_WriteRegister(const AD7124_ConfigTypeDef *pADC, uint8_t regAddr, uint8_t dataSize, uint32_t regValue);

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
AD7124_StatusTypeDef AD7124_Reset(const AD7124_ConfigTypeDef *pADC, uint32_t timeout);

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
AD7124_StatusTypeDef AD7124_Config(const AD7124_ConfigTypeDef *pADC, AD7124_RegisterTypeDef *pConfig);

/**
 * @brief Reads ADC conversion data from the AD7124-4 and updates the channel samples array.
 *
 * Retrieves the latest conversion result from the AD7124 over SPI and stores the sample
 * value in the corresponding element of the external `AD7124_ChannelSamples` array.
 * Supports reading from multiple enabled and configured ADC channels.
 *
 * @param[in] pADC  Pointer to the AD7124-4 configuration structure containing SPI settings and communication parameters.
 *
 * @retval AD7124_OK     Sample data read successfully.
 * @retval AD7124_ERROR  Communication failure, CRC mismatch, or invalid parameters.
 *
 * @note The `AD7124_ChannelSamples` array must be defined and managed by the user.
 */
AD7124_StatusTypeDef AD7124_ReadSampleData(const AD7124_ConfigTypeDef *pADC);

/**
 * @brief Checks the AD7124 ADC error status by reading the error register.
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
AD7124_StatusTypeDef AD7124_ErrorCheck(const AD7124_ConfigTypeDef *pADC, int32_t *pErrorReg);

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
AD7124_StatusTypeDef AD7124_Calibration(const AD7124_ConfigTypeDef *pADC, AD7124_RegisterTypeDef *pConfig, uint32_t timeout);


#ifdef __cplusplus
}
#endif


#endif  /* AD7124_H */
