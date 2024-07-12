
#ifndef __DRV_PORT_UART_H__
#define __DRV_PORT_UART_H__

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

#define DEV_UART_0                  "uart0"       /* Device Name for UART0 */
#define DEV_UART_1                  "uart1"       /* Device Name for UART1 */


/*
 * optional xfer mode for uart
 */
#define UART_XFER_MODE_POLL         0u
#define UART_XFER_MODE_IRQ          1u
#define UART_XFER_MODE_DMA          2u



/*******************************************
 *
 * Configuration Area, Changed By User
 *
 *******************************************/

#define DEV_UART_0_ENABLE            1           /* Register UART0 Dev or Not */
#define DEV_UART_1_ENABLE            1           /* Register UART1 Dev or Not */
#define DEV_UART_BUF_SIZE            128u


#if DEV_UART_0_ENABLE
    /* PARAMS */
    #define DEV_UART0_CFG_BAUD_RATE         BAUD_RATE_115200
    #define DEV_UART0_CFG_DATA_BITS         DATA_BITS_8
    #define DEV_UART0_CFG_STOP_BITS         STOP_BITS_1
    #define DEV_UART0_CFG_PARITY            PARITY_NONE
    #define DEV_UART0_CFG_BIT_ORDER         BIT_ORDER_LSB
    #define DEV_UART0_CFG_INVERT            NRZ_NORMAL
    #define DEV_UART0_CFG_BUFSZ             (DEV_UART_BUF_SIZE * 2)
    #define DEV_UART0_CFG_FLOW_CTRL         RT_SERIAL_FLOWCONTROL_NONE

    /* Pin */
    #define DEV_UART0_TX_IO_TYPE            APP_IO_TYPE_AON
    #define DEV_UART0_TX_PIN                APP_IO_PIN_0
    #define DEV_UART0_TX_PINMUX             APP_IO_MUX_7
    #define DEV_UART0_TX_PULL               APP_IO_NOPULL
    #define DEV_UART0_RX_IO_TYPE            APP_IO_TYPE_AON
    #define DEV_UART0_RX_PIN                APP_IO_PIN_1
    #define DEV_UART0_RX_PINMUX             APP_IO_MUX_8
    #define DEV_UART0_RX_PULL               APP_IO_PULLUP

    /* Xfer Mode */
    #define DEV_UART0_XFER_MODE             UART_XFER_MODE_IRQ      /* Suggest that not use the DMA mode */

    #if DEV_UART0_XFER_MODE == UART_XFER_MODE_DMA
        #define DEV_UART0_TX_DMA_CHANNEL    DMA_Channel2            /* Attention to avoid the DMA Channel conflict with other module */
        #define DEV_UART0_RX_DMA_CHANNEL    DMA_Channel3

        #define DEV_UART0_FLAG              (RT_DEVICE_FLAG_RDWR   | \
                                             RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX | \
                                             RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX)
    #else
        #define DEV_UART0_FLAG              (RT_DEVICE_FLAG_RDWR   | \
                                             RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX )
    #endif

#endif


#if DEV_UART_1_ENABLE
    #define DEV_UART1_CFG_BAUD_RATE         BAUD_RATE_115200
    #define DEV_UART1_CFG_DATA_BITS         DATA_BITS_8
    #define DEV_UART1_CFG_STOP_BITS         STOP_BITS_1
    #define DEV_UART1_CFG_PARITY            PARITY_NONE
    #define DEV_UART1_CFG_BIT_ORDER         BIT_ORDER_LSB
    #define DEV_UART1_CFG_INVERT            NRZ_NORMAL
    #define DEV_UART1_CFG_BUFSZ             (DEV_UART_BUF_SIZE * 2)
    #define DEV_UART1_CFG_FLOW_CTRL         RT_SERIAL_FLOWCONTROL_NONE

    /* Pin */
    #define DEV_UART1_TX_IO_TYPE            APP_IO_TYPE_GPIOA
    #define DEV_UART1_TX_PIN                APP_IO_PIN_2
    #define DEV_UART1_TX_PINMUX             APP_IO_MUX_11
    #define DEV_UART1_TX_PULL               APP_IO_NOPULL
    #define DEV_UART1_RX_IO_TYPE            APP_IO_TYPE_GPIOA
    #define DEV_UART1_RX_PIN                APP_IO_PIN_3
    #define DEV_UART1_RX_PINMUX             APP_IO_MUX_12
    #define DEV_UART1_RX_PULL               APP_IO_PULLUP

    /* Xfer Mode */
    #define DEV_UART1_XFER_MODE             UART_XFER_MODE_IRQ      /* Suggest that not use the DMA mode */

    #if DEV_UART1_XFER_MODE == UART_XFER_MODE_DMA
        #define DEV_UART1_TX_DMA_CHANNEL    DMA_Channel2            /* Attention to avoid the DMA Channel conflict with other module */
        #define DEV_UART1_RX_DMA_CHANNEL    DMA_Channel_DISABLE

        #define DEV_UART1_FLAG              (RT_DEVICE_FLAG_RDWR   | \
                                             RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX | \
                                             RT_DEVICE_FLAG_DMA_TX)     /* remove RT_DEVICE_FLAG_DMA_RX FLAG */
    #else
        #define DEV_UART1_FLAG              (RT_DEVICE_FLAG_RDWR   | \
                                             RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX )
    #endif

#endif


int rt_hw_uart_init(void);


#ifdef __cplusplus
}
#endif

#endif /* __DRV_PORT_UART_H__ */
