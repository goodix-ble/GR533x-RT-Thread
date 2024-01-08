
#ifndef __DRV_PORT_I2C_H__
#define __DRV_PORT_I2C_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_porting.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************
 *
 * Define Area, Do Not Change
 *
 *******************************************/

#define DEV_I2C_0                   "i2c0"       /* Device Name for I2C0 */
#define DEV_I2C_1                   "i2c1"       /* Device Name for I2C1 */


/*
 * optional xfer mode for i2c
 */
#define I2C_XFER_MODE_POLL         0u
#define I2C_XFER_MODE_IRQ          1u
#define I2C_XFER_MODE_DMA          2u           /* Suggest that not use the DMA mode, No obvious advantages and take up DMA channel */


/*******************************************
 *
 * Configuration Area, Changed By User
 *
 *******************************************/

#define DEV_I2C_0_ENABLE            1           /* Register I2C0 Dev or Not */
#define DEV_I2C_1_ENABLE            1           /* Register I2C1 Dev or Not */


#if DEV_I2C_0_ENABLE
    #define DEV_I2C_0_SCL_IO_TYPE       APP_IO_TYPE_GPIOA
    #define DEV_I2C_0_SCL_PIN           APP_IO_PIN_4
    #define DEV_I2C_0_SCL_PINMUX        APP_IO_MUX_1
    #define DEV_I2C_0_SCL_PULL          APP_IO_PULLUP
    #define DEV_I2C_0_SDA_IO_TYPE       APP_IO_TYPE_GPIOA
    #define DEV_I2C_0_SDA_PIN           APP_IO_PIN_5
    #define DEV_I2C_0_SDA_PINMUX        APP_IO_MUX_2
    #define DEV_I2C_0_SDA_PULL          APP_IO_PULLUP
    #define DEV_I2C_0_SPEED             I2C_SPEED_400K
    #define DEV_I2C_0_XFER_MODE         I2C_XFER_MODE_IRQ           /* Suggest that not use the DMA mode */

    #if DEV_I2C_0_XFER_MODE == I2C_XFER_MODE_DMA
        #define DEV_I2C_0_TX_DMA_CHANNEL    DMA_Channel2            /* Attention to avoid the DMA Channel conflict with other module */
        #define DEV_I2C_0_RX_DMA_CHANNEL    DMA_Channel3
    #endif

#endif

#if DEV_I2C_1_ENABLE
    #define DEV_I2C_1_SCL_IO_TYPE       APP_IO_TYPE_GPIOA
    #define DEV_I2C_1_SCL_PIN           APP_IO_PIN_6
    #define DEV_I2C_1_SCL_PINMUX        APP_IO_MUX_3
    #define DEV_I2C_1_SCL_PULL          APP_IO_PULLUP
    #define DEV_I2C_1_SDA_IO_TYPE       APP_IO_TYPE_GPIOA
    #define DEV_I2C_1_SDA_PIN           APP_IO_PIN_7
    #define DEV_I2C_1_SDA_PINMUX        APP_IO_MUX_4
    #define DEV_I2C_1_SDA_PULL          APP_IO_PULLUP
    #define DEV_I2C_1_SPEED             I2C_SPEED_400K
    #define DEV_I2C_1_XFER_MODE         I2C_XFER_MODE_IRQ           /* Suggest that not use the DMA mode */

    #if DEV_I2C_1_XFER_MODE == I2C_XFER_MODE_DMA
        #define DEV_I2C_1_TX_DMA_CHANNEL    DMA_Channel2            /* Attention to avoid the DMA Channel conflict with other module */
        #define DEV_I2C_1_RX_DMA_CHANNEL    DMA_Channel3
    #endif
#endif

#define DEV_I2C_POLL_TIMEOUT          1000u       //ms


int rt_hw_i2c_init(void);


#ifdef __cplusplus
}
#endif

#endif /* __DRV_PORT_I2C_H__ */
