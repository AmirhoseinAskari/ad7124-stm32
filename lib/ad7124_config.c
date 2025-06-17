/**
 * @file    ad7124_config.c
 * @author  Amirhossein Askari
 * @version 1.0.0
 * @date    2025-06-15
 * @email   theamiraskarii@gmail.com
 * @see     https://github.com/AmirhoseinAskari
 * @brief   Predefined configurations and register settings for the AD7124-4 ADC.
 * 
 * @details
 * This file contains predefined configuration structures for the AD7124-4 24-bit Sigma-Delta ADC,
 * including channel mappings, filter settings, gain, offset, and control registers.
 * 
 * @note
 * - SPI_CRC_ERR_EN (bit 2) in the error_enable register should be set to 1 to enable CRC error detection.
 * 
 * @warning
 * - Changing the configurations without understanding AD7124 register maps may cause unexpected ADC behavior.
 * - Always verify compatibility of settings with your hardware setup and ADC datasheet.
 * 
 * @section Supported_Platforms
 * STM32 microcontrollers (HAL Drivers).
 */


/* ------------------------------------- Includes ------------------------------------- */

#include "ad7124.h"  /**< Header file declaring the AD7124 driver functions */



/* -------------------------------------- Configs ------------------------------------- */

/**********************************************************************************
 * Config A:
 * Continuous Conversion Mode with Internal Reference
 * - IOUT0 on Ain6 (off)
 * - IOUT1 on Ain7 (off)
 * - VBIAS0 on Ain0
 * 
 * Channel 0 with setup 0: AINM on Ain0, AINP on Ain1
 * Channel 1 with setup 0: Internal Temperature Sensor
 * 
 * PGA = 1, Reference Source = Internal
 * All inputs are Buffered and Bipolar.
 * Burnout current source = off
 * Filter setting FS=240: 10 SPS on SYNC4
 **********************************************************************************/
AD7124_RegisterTypeDef configA = {
    .adc_control  =  0x0700U,  
    .io_control_1 =  0x0000FEU, 
    .io_control_2 =  0x0001U,
    .error_enable =  0x7FAFFU,
    .channels     = {0x8020U,   0x8211U,   0x0000U,   0x0000U,   0x0000U,   0x0000U,   0x0000U,   0x0000U},
    .configs      = {0x09F0U,   0x09F0U,   0x09F0U,   0x09F0U,   0x09F0U,   0x09F0U,   0x09F0U,   0x09F0U},
    .filters      = {0x0600F0U, 0x0600F0U, 0x0600F0U, 0x0600F0U, 0x0600F0U, 0x0600F0U, 0x0600F0U, 0x0600F0U},
    .offsets      = {0x800000U, 0x800000U, 0x800000U, 0x800000U, 0x800000U, 0x800000U, 0x800000U, 0x800000U},
    .gains        = {0x5000U,   0x5000U,   0x5000U,   0x5000U,   0x5000U,   0x5000U,   0x5000U,   0x5000U}
};


/* ad7124_config.c */
