
#ifndef __DRV_PORT_SPI_H__
#define __DRV_PORT_SPI_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_porting.h"

#ifdef __cplusplus
extern "C" {
#endif


#define SPI_ONCE_XFER_MAX_BEAT      4095u   /* DO NOT Change THIS */
#define DEV_SPI_0                   "spi0"  /* Bus Name */


/*
 * optional xfer mode for spi
 */
#define SPI_XFER_MODE_POLL         0u
#define SPI_XFER_MODE_IRQ          1u
#define SPI_XFER_MODE_DMA          2u


/*********************************************************
 * GPIO11 -> SPI_CS   -> PIN NO.43  (Controlled By RT-T Logic)
 * GPIO8  -> SPI_CLK  -> PIN NO.40
 * GPIO9  -> SPI_MOSI -> PIN NO.41
 * GPIO10 -> SPI_MISO -> PIN NO.42
 */
#if 0
    #define DEV_SPI_CS_IO_TYPE              APP_IO_TYPE_GPIOA
    #define DEV_SPI_CS_PIN                  APP_IO_PIN_11
    #define DEV_SPI_CS_PINMUX               APP_IO_MUX_43
    #define DEV_SPI_CS_PULL                 APP_IO_PULLUP
#else
    #define SPI_CS_PIN_NUM                  43              /* Pin Number, by RT-T Pin Driver */
#endif

#define DEV_SPI_CLK_IO_TYPE                 APP_IO_TYPE_GPIOA
#define DEV_SPI_CLK_PIN                     APP_IO_PIN_8
#define DEV_SPI_CLK_PINMUX                  APP_IO_MUX_42
#define DEV_SPI_CLK_PULL                    APP_IO_PULLUP

#define DEV_SPI_MOSI_IO_TYPE                APP_IO_TYPE_GPIOA
#define DEV_SPI_MOSI_PIN                    APP_IO_PIN_9
#define DEV_SPI_MOSI_PINMUX                 APP_IO_MUX_46
#define DEV_SPI_MOSI_PULL                   APP_IO_PULLUP

#define DEV_SPI_MISO_IO_TYPE                APP_IO_TYPE_GPIOA
#define DEV_SPI_MISO_PIN                    APP_IO_PIN_10
#define DEV_SPI_MISO_PINMUX                 APP_IO_MUX_45
#define DEV_SPI_MISO_PULL                   APP_IO_PULLUP

#define DEV_SPI_WAIT_TIMEOUT_MS             1000u
#define DEV_SPI_CLOCK_DEFAULT_PRESCALER     2u

#define DEV_SPI_XFER_MODE                   SPI_XFER_MODE_DMA

#if DEV_SPI_XFER_MODE == SPI_XFER_MODE_DMA
    #define DEV_SPI_RX_DMA_CHANNEL              DMA_Channel0    /* Attention to avoid the DMA Channel conflict with other module */
    #define DEV_SPI_TX_DMA_CHANNEL              DMA_Channel1
#endif

int rt_hw_spi_init(void);


#ifdef __cplusplus
}
#endif

#endif /* __DRV_PORT_SPI_H__ */
