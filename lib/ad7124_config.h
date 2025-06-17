/**
 * @file    ad7124_config.h
 * @author  Amirhossein Askari
 * @version 1.0.0
 * @date    2025-06-15
 * @email   theamiraskarii@gmail.com
 * @see     https://github.com/AmirhoseinAskari
 * @brief   Configuration header for AD7124 ADC.
 *
 * @details
 * This file allows selecting the STM32 MCU series. Modify the macros below to fit your specific MCU.
 *
 * @warning
 * Always ensure the selected macro reflects the correct STM32 device.
 * 
 * @par Supported Platforms
 * STM32 (F0, F1, F2, F3, F4, F7, G0, G4, H7, L0, L1, L4, L5, U0, U5 series)
 */


#ifndef AD7124_CONFIG_H
#define AD7124_CONFIG_H


#ifdef __cplusplus
extern "C" {
#endif

    
/* ------------------------------------------------------------------------------------- */
/*                               Select STM32 Series (REQUIRED)                          */
/* ------------------------------------------------------------------------------------- */

/**
 * @brief Select the STM32 microcontroller series used in your project.
 *
 * Define exactly one macro corresponding to your STM32 MCU series before
 * including any STM32 HAL headers or using the AD7124 ADC driver.
 *
 * Possible values:
 *   _STM32F0, _STM32F1, _STM32F2, _STM32F3, _STM32F4, _STM32F7,
 *   _STM32G0, _STM32G4, _STM32H7,
 *   _STM32L0, _STM32L1, _STM32L4, _STM32L5,
 *   _STM32U0, _STM32U5
 */

#define  _STM32L4
    
   
#ifdef __cplusplus
}
#endif


#endif /* AD7124_CONFIG_H */
