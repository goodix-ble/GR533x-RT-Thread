/**
 ****************************************************************************************
 *
 * @file    gr533x_ll_adc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of ADC LL library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup LL_DRIVER LL Driver
  * @{
  */

/** @defgroup LL_ADC ADC
  * @brief ADC LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR533X_LL_ADC_H__
#define __GR533X_LL_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr533x.h"

#if defined(AON_CTL)

/** @defgroup ADC_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup ADC_LL_ES_INIT ADC Exported init structures
  * @{
  */

/**
  * @brief LL ADC init Structure definition
  */
typedef struct _ll_adc_init
{
    uint32_t channel_p;     /**< Specifies the input source to ADC channel P.
                                 This parameter can be any value of @ref ADC_LL_EC_INPUT_SRC.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_channelp(). */

    uint32_t channel_n;     /**< Specifies the input source to ADC channel N.
                                 This parameter can be any value of @ref ADC_LL_EC_INPUT_SRC.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_channeln(). */

    uint32_t input_mode;    /**< Specifies the operation mode for the ADC sample.
                                 This parameter can be a value of @ref ADC_LL_EC_INPUT_MODE.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_input_mode(). */

    uint32_t ref_source;    /**< Specifies the source of the ADC reference.
                                 This parameter can be a value of @ref ADC_LL_EC_REFERENCE_SRC.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_ref().*/

    uint32_t ref_value;     /*!< Specifies the value of the ADC buffered reference.
                                 This parameter can be a value of @ref ADC_LL_EC_REFERENCE.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_ref_value().*/

    uint32_t clock;         /**< Specifies the clock of ADC.
                                 This parameter can be a value of @ref ADC_LL_EC_CLK.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_clock().*/

} ll_adc_init_t;

/** @} */

/** @} */

/**
  * @defgroup  ADC_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup ADC_LL_Exported_Constants ADC Exported Constants
  * @{
  */

/** @defgroup ADC_LL_EC_CLK ADC CLOCK
  * @{
  */
#define LL_ADC_CLK_16M              (4UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 16 MHz  */
#define LL_ADC_CLK_8M               (5UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 8 MHz  */
#define LL_ADC_CLK_4M               (6UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 4 MHz  */
#define LL_ADC_CLK_1M               (7UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 1 MHz  */
#define LL_ADC_CLK_16K              (1UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 16KHz  */
#define LL_ADC_CLK_8K               (2UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 8KHz  */
#define LL_ADC_CLK_4K               (3UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< ADC Clock = 4KHz  */
#define LL_ADC_CLK_NONE             (0UL << MCU_SUB_SNSADC_CLK_WR_Pos)   /**< No ADC Clock*/

/** @} */

/** @defgroup ADC_LL_EC_REFERENCE ADC Buffered Internal Reference Value
  * @{
  */
#define LL_ADC_REF_VALUE_0P8        (0x3UL << AON_PMU_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 0.85 V */
#define LL_ADC_REF_VALUE_1P2        (0x7UL << AON_PMU_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 1.28 V */
#define LL_ADC_REF_VALUE_1P6        (0xAUL << AON_PMU_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 1.60 V */
#define LL_ADC_REF_VALUE_2P0        (0xFUL << AON_PMU_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 2.00 V */
/** @} */

/** @defgroup ADC_LL_EC_INPUT_MODE ADC Input Mode
  * @{
  */
#define LL_ADC_INPUT_SINGLE         (1UL << AON_PMU_SNSADC_CFG_SINGLE_EN_Pos)     /**< Single ended mode */
#define LL_ADC_INPUT_DIFFERENTIAL   (0x00000000UL)                            /**< Differential mode */
/** @} */

/** @defgroup ADC_LL_EC_INPUT_SRC ADC Input Source
  * @{
  */
#define LL_ADC_INPUT_SRC_IO0        (0UL)  /**< Select MSIO0 as input       */
#define LL_ADC_INPUT_SRC_IO1        (1UL)  /**< Select MSIO1 as input       */
#define LL_ADC_INPUT_SRC_IO2        (2UL)  /**< Select MSIO2 as input       */
#define LL_ADC_INPUT_SRC_IO3        (3UL)  /**< Select MSIO3 as input       */
#define LL_ADC_INPUT_SRC_IO4        (4UL)  /**< Select MSIO4 as input       */
#define LL_ADC_INPUT_SRC_IO5        (5UL)  /**< Select MSIO5 as input       */
#define LL_ADC_INPUT_SRC_IO6        (6UL)  /**< Select MSIO6 as input       */
#define LL_ADC_INPUT_SRC_IO7        (7UL)  /**< Select MSIO7 as input       */
#define LL_ADC_INPUT_SRC_TMP        (13UL)  /**< Select temperature as input */
#define LL_ADC_INPUT_SRC_BAT        (14UL)  /**< Select Vbattery as input    */
#define LL_ADC_INPUT_SRC_REF        (15UL)  /**< Select reference as input   */

/** @} */

/** @defgroup ADC_LL_EC_REFERENCE_SRC ADC Reference Source
  * @{
  */
#define LL_ADC_REF_SRC_BUF_INT      (0x00000000UL)                            /**< Select buffered internal reference as reference   */
#define LL_ADC_REF_SRC_IO0          (3UL << AON_PMU_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO0 as reference                         */
#define LL_ADC_REF_SRC_IO1          (4UL << AON_PMU_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO1 as reference                         */
#define LL_ADC_REF_SRC_IO2          (5UL << AON_PMU_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO2 as reference                         */
#define LL_ADC_REF_SRC_IO3          (6UL << AON_PMU_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO3 as reference                         */

/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup ADC_LL_Exported_Macros ADC Exported Macros
  * @{
  */

/** @defgroup ADC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in ADC register
  * @param  __instance__ ADC instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_ADC_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG((__instance__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in ADC register
  * @param  __instance__ ADC instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_ADC_ReadReg(__instance__, __REG__) READ_REG((__instance__)->__REG__)

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup ADC_LL_Private_Macros ADC Private Macros
  * @{
  */

/** @defgroup ADC_LL_EC_DEFAULT_CONFIG InitStruct default configuartion
  * @{
  */

/**
  * @brief LL ADC InitStrcut default configuartion
  */

#define LL_ADC_DEFAULT_CONFIG                      \
{                                                  \
    .channel_p  = LL_ADC_INPUT_SRC_IO0,            \
    .channel_n  = LL_ADC_INPUT_SRC_IO1,            \
    .input_mode = LL_ADC_INPUT_DIFFERENTIAL,       \
    .ref_source = LL_ADC_REF_SRC_BUF_INT,          \
    .ref_value  = LL_ADC_REF_VALUE_1P2,            \
    .clock      = LL_ADC_CLK_16M                   \
}

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup ADC_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup ADC_LL_EF_Configuration Basic Configuration
  * @{
  */

/**
  * @brief  Enable ADC module.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable(void)
{
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_EN_Msk);
}

/**
  * @brief  Disable ADC module.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable(void)
{
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_EN_Msk);
}

/**
  * @brief  Check if ADC module is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled(void)
{
    return (READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_EN_Msk) == (AON_PMU_SNSADC_CFG_EN_Msk));
}

/**
  * @brief  Disable ADC clock.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_1 | ADC_CLK_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_clock(void)
{
    MODIFY_REG(MCU_SUB->SENSE_ADC_CLK, MCU_SUB_SNSADC_CLK_WR, MCU_SUB_SNSADC_CLK_NONE);
}

/**
  * @brief  Check if ADC clock is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_1 | ADC_CLK_EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_clock(void)
{
    return (READ_BITS(MCU_SUB->SENSE_ADC_CLK, MCU_SUB_SNSADC_CLK_RD) != 0);
}

/**
  * @brief  Set ADC clock source.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_1 | ADC_CLK_SEL
  *
  * @param  clk This parameter can be one of the following values:
  *         @arg @ref LL_ADC_CLK_16M
  *         @arg @ref LL_ADC_CLK_8M
  *         @arg @ref LL_ADC_CLK_4M
  *         @arg @ref LL_ADC_CLK_1M
  *         @arg @ref LL_ADC_CLK_16K
  *         @arg @ref LL_ADC_CLK_8K
  *         @arg @ref LL_ADC_CLK_4K
  *         @arg @ref LL_ADC_CLK_NONE
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_clock(uint32_t clk)
{
    MODIFY_REG(MCU_SUB->SENSE_ADC_CLK, MCU_SUB_SNSADC_CLK_WR, clk);
}

/**
  * @brief  Return source for ADC clock.
  *
  *  Register|BitsName
  *  --------|--------
  *  MSIO_PAD_CFG_1 | ADC_CLK_SEL
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_CLK_16M
  *         @arg @ref LL_ADC_CLK_8M
  *         @arg @ref LL_ADC_CLK_4M
  *         @arg @ref LL_ADC_CLK_1M
  *         @arg @ref LL_ADC_CLK_16K
  *         @arg @ref LL_ADC_CLK_8K
  *         @arg @ref LL_ADC_CLK_4K
  *         @arg @ref LL_ADC_CLK_NONE
  */
__STATIC_INLINE uint32_t ll_adc_get_clock(void)
{
     return (uint32_t)(READ_BITS(MCU_SUB->SENSE_ADC_CLK, MCU_SUB_SNSADC_CLK_RD) >> MCU_SUB_SNSADC_CLK_RD_Pos);
}

/**
  * @brief  Set ADC bias reference.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG1
  *
  * @param  value This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REF_VALUE_0P8
  *         @arg @ref LL_ADC_REF_VALUE_1P2
  *         @arg @ref LL_ADC_REF_VALUE_1P6
  *         @arg @ref LL_ADC_REF_VALUE_2P0
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_ref_value(uint32_t value)
{
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_VALUE_Msk, value);
}

/**
  * @brief  Return ADC bias reference.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG1
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REF_VALUE_0P8
  *         @arg @ref LL_ADC_REF_VALUE_1P2
  *         @arg @ref LL_ADC_REF_VALUE_1P6
  *         @arg @ref LL_ADC_REF_VALUE_2P0
  */
__STATIC_INLINE uint32_t ll_adc_get_ref_value(void)
{
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_VALUE_Msk) >> AON_PMU_SNSADC_CFG_REF_VALUE_Pos);
}

/**
  * @brief  Enable temperature sensor.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_temp(void)
{
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_TEMP_EN_Msk);
}

/**
  * @brief  Disable temperature sensor.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_temp(void)
{
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_TEMP_EN_Msk);
}

/**
  * @brief  Check if temperature sensor is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_temp(void)
{
    return (READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_TEMP_EN_Msk) == (AON_PMU_SNSADC_CFG_TEMP_EN_Msk));
}

/**
  * @brief  Enable Vbattery sensor.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_vbat(void)
{
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_VBAT_EN_Msk);
}

/**
  * @brief  Disable Vbattery sensor.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_vbat(void)
{
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_VBAT_EN_Msk);
}

/**
  * @brief  Check if Vbattery sensor is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_vbat(void)
{
    return (READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_VBAT_EN_Msk) == (AON_PMU_SNSADC_CFG_VBAT_EN_Msk));
}

/**
  * @brief  Set ADC input mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SINGLE
  *         @arg @ref LL_ADC_INPUT_DIFFERENTIAL
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_input_mode(uint32_t mode)
{
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_SINGLE_EN_Msk, mode);
}

/**
  * @brief  Return ADC input mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SINGLE
  *         @arg @ref LL_ADC_INPUT_DIFFERENTIAL
  */
__STATIC_INLINE uint32_t ll_adc_get_input_mode(void)
{
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_SINGLE_EN_Msk));
}

/**
  * @brief  Enable offset calibration.
  * @note   Enable offset calibration, used to swap inputs of comparator for offset
  *         calibration.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_ofs_cal(void)
{
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_OFS_CAL_EN_Msk);
}

/**
  * @brief  Disable offset calibration.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_ofs_cal(void)
{
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_OFS_CAL_EN_Msk);
}

/**
  * @brief  Check if offset calibration is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_ofs_cal(void)
{
    return (READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_OFS_CAL_EN_Msk) == (AON_PMU_SNSADC_CFG_OFS_CAL_EN_Msk));
}

/**
  * @brief  Set dynamic rang of ADC.
  * @note   When higher input signal frequencies close to Nyquist rate, you should set 1.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @param  rang This parameter can be a value between: 1 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_dynamic_rang(uint32_t rang)
{
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_DYMAMIC_Msk, (rang & 0x7) << AON_PMU_SNSADC_CFG_DYMAMIC_Pos);
}

/**
  * @brief  Return ADC dynamic rang.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG2
  *
  * @retval Returned value can be a value between: 1 ~ 7
  */
__STATIC_INLINE uint32_t ll_adc_get_dynamic_rang(void)
{
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_DYMAMIC_Msk) >> AON_PMU_SNSADC_CFG_DYMAMIC_Pos);
}

/**
  * @brief  Set source of ADC input channelP.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG3
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_channelp(uint32_t source)
{
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_CHN_P_Msk, source << AON_PMU_SNSADC_CFG_CHN_P_Pos);
}

/**
  * @brief  Return source of ADC input channelP.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG3
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  */
__STATIC_INLINE uint32_t ll_adc_get_channelp(void)
{
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_CHN_P_Msk) >> AON_PMU_SNSADC_CFG_CHN_P_Pos);
}

/**
  * @brief  Set source of ADC input channelN.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG3
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_channeln(uint32_t source)
{
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_CHN_N_Msk, source << AON_PMU_SNSADC_CFG_CHN_N_Pos);
}

/**
  * @brief  Return source of ADC input channelN.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG3
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  */
__STATIC_INLINE uint32_t ll_adc_get_channeln(void)
{
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_CHN_N_Msk) >> AON_PMU_SNSADC_CFG_CHN_N_Pos);
}

/**
  * @brief  Enable ADC MAS_RST.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_mas_rst(void)
{
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_MAS_RST_Msk);
}

/**
  * @brief  Disable ADC MAS_RST.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_mas_rst(void)
{
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_MAS_RST_Msk);
}

/**
  * @brief  Check if ADC MAS_RST is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_mas_rst(void)
{
    return (READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_MAS_RST_Msk) == (AON_PMU_SNSADC_CFG_MAS_RST_Msk));
}

/**
  * @brief  Set source of ADC reference.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REF_SRC_BUF_INT
  *         @arg @ref LL_ADC_REF_SRC_IO0
  *         @arg @ref LL_ADC_REF_SRC_IO1
  *         @arg @ref LL_ADC_REF_SRC_IO2
  *         @arg @ref LL_ADC_REF_SRC_IO3
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_ref(uint32_t source)
{
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_SEL_Msk, source);
}

/**
  * @brief  Return source of ADC reference.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REF_SRC_BUF_INT
  *         @arg @ref LL_ADC_REF_SRC_IO0
  *         @arg @ref LL_ADC_REF_SRC_IO1
  *         @arg @ref LL_ADC_REF_SRC_IO2
  *         @arg @ref LL_ADC_REF_SRC_IO3
  */
__STATIC_INLINE uint32_t ll_adc_get_ref(void)
{
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_SEL_Msk) >> AON_PMU_SNSADC_CFG_REF_SEL_Pos);
}

/**
  * @brief  Set current of ADC reference circuit.
  * @note   When samples at 100kbps, you should set 0.
  *         When samples at 1mbps, you should set 7.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @param  source This parameter can be a value between: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_ref_current(uint32_t source)
{
    MODIFY_REG(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_HP_Msk, (source & 0x7) << AON_PMU_SNSADC_CFG_REF_HP_Pos);
}

/**
  * @brief  Return current of ADC reference circuit.
  *
  *  Register|BitsName
  *  --------|--------
  *  SNSADC_CFG | REG4
  *
  * @retval Returned value can be a value between: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_adc_get_ref_current(void)
{
    return (uint32_t)(READ_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_REF_HP_Msk) >> AON_PMU_SNSADC_CFG_REF_HP_Pos);
}

/** @} */

/** @defgroup ADC_LL_EF_FIFO_Access FIFO Access
  * @{
  */

/**
  * @brief  Return samples value of ADC by reading FIFO.
  * @note   There are two value in the register, both of them is 16bits.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_FIFO | SENSE_ADC_FIFO
  *
  * @retval Smaples value of input
  */
__STATIC_INLINE uint32_t ll_adc_read_fifo(void)
{
    return (uint32_t)(READ_REG(MCU_SUB->SENSE_ADC_FIFO));
}

/**
  * @brief  Set threshold of ADC FIFO.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_FF_THRESH | SENSE_FF_THRESH
  *
  * @param  thresh This parameter can be a value between: 0 ~ 64
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_thresh(uint32_t thresh)
{
    MODIFY_REG(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_THRESH, (thresh & 0x3F) << MCU_SUB_SNSADC_FF_THRESH_Pos);
}

/**
  * @brief  Return threshold of ADC FIFO.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_FF_THRESH | SENSE_FF_THRESH
  *
  * @retval Returned value can be a value between: 0 ~ 64
  */
__STATIC_INLINE uint32_t ll_adc_get_thresh(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_THRESH) >> MCU_SUB_SNSADC_FF_THRESH_Pos);
}

/**
  * @brief  Enable ADC dma_req.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_FF_THRESH | MCU_SUB_SNSADC_FF_DMA_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_dma_req(void)
{
    SET_BITS(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_DMA_EN_Msk);
}

/**
  * @brief  Disable ADC dma_req.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_FF_THRESH | MCU_SUB_SNSADC_FF_DMA_EN
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_dma_req(void)
{
    CLEAR_BITS(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_DMA_EN_Msk);
}

/**
  * @brief  Check if dma_req is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_FF_THRESH | MCU_SUB_SNSADC_FF_DMA_EN
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_dma_req(void)
{
    return (READ_BITS(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_DMA_EN_Msk) == (MCU_SUB_SNSADC_FF_DMA_EN_Msk));
}

/**
  * @brief  Check if ADC FIFO is not empty.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_STAT | VAL
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_fifo_notempty(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->SENSE_ADC_STAT, MCU_SUB_SNSADC_STAT_VAL) == MCU_SUB_SNSADC_STAT_VAL);
}

/**
  * @brief  Return count of ADC FIFO.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_STAT | FF_COUNT
  *
  * @retval Returned value can be a value between: 0 ~ 64
  */
__STATIC_INLINE uint32_t ll_adc_get_fifo_count(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->SENSE_ADC_STAT, MCU_SUB_SNSADC_STAT_FF_COUNT) >> MCU_SUB_SNSADC_STAT_FF_COUNT_Pos);
}

/**
  * @brief  Flush ADC FIFO.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_STAT | FF_FLUSH
  *
  * @retval void
  */
__STATIC_INLINE void ll_adc_flush_fifo(void)
{
    SET_BITS(MCU_SUB->SENSE_ADC_STAT, MCU_SUB_SNSADC_STAT_FLUSH_Msk);
}

/**
  * @brief  Try to lock hw token.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_GET_TKN_HW
  *
  * @retval Returned true if hw lock adc token success; return false if hw lock adc token fail
  */
__STATIC_INLINE uint32_t ll_adc_try_lock_hw_token(void)
{
    return (uint32_t)(READ_REG(MCU_SUB->SENSE_ADC_GET_TKN_HW) == MCU_SUB_SNSADC_TKN_LOCKED_HW);
}

/**
  * @brief  Release hw token.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_RET_TKN_HW | HW_RELEASE_Msk
  *
  * @retval none
  */
__STATIC_INLINE void ll_adc_release_hw_token(void)
{
    WRITE_REG(MCU_SUB->SENSE_ADC_RET_TKN_HW, 0x00);
}

/**
  * @brief  Try to lock sw token.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_GET_TKN_SW
  *
  * @retval Returned true if sw lock adc token success; return false if sw lock adc token fail
  */
__STATIC_INLINE uint32_t ll_adc_try_lock_sw_token(void)
{
    return (uint32_t)(READ_REG(MCU_SUB->SENSE_ADC_GET_TKN_SW) == MCU_SUB_SNSADC_TKN_LOCKED_SW);
}

/**
  * @brief  Release sw token.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_RET_TKN_SW | SW_RELEASE_Msk
  *
  * @retval none
  */
__STATIC_INLINE void ll_adc_release_sw_token(void)
{
    WRITE_REG(MCU_SUB->SENSE_ADC_RET_TKN_SW, 0x00);
}

/**
  * @brief  get adc token state.
  *
  *  Register|BitsName
  *  --------|--------
  *  SENSE_ADC_TKN_STS
  *
  * @retval Returned value from SENSE_ADC_TKN_STS reg
  */
__STATIC_INLINE uint32_t ll_adc_get_token_state(void)
{
    return READ_REG(MCU_SUB->SENSE_ADC_TKN_STS);
}

/** @} */

/** @defgroup ADC_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize ADC registers (Registers restored to their default values).
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: ADC registers are de-initialized
  *          - ERROR: ADC registers are not de-initialized
  */
error_status_t ll_adc_deinit(void);

/**
  * @brief  Initialize ADC registers according to the specified.
  *         parameters in p_adc_init.
  * @param  p_adc_init Pointer to a ll_adc_init_t structure that contains the configuration
  *                             information for the specified ADC peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: ADC registers are initialized according to p_adc_init content
  *          - ERROR: Problem occurred during ADC Registers initialization
  */
error_status_t ll_adc_init(ll_adc_init_t *p_adc_init);

/**
  * @brief Set each field of a @ref ll_adc_init_t type structure to default value.
  * @param p_adc_init  Pointer to a @ref ll_adc_init_t structure
  *                             whose fields will be set to default values.
  * @retval None
  */
void ll_adc_struct_init(ll_adc_init_t *p_adc_init);

/** @} */

/** @} */

#endif /* AON */

#ifdef __cplusplus
}
#endif

#endif /* __GR533X_LL_ADC_H__ */

/** @} */

/** @} */

/** @} */
