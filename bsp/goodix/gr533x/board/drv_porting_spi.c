#include <rtdevice.h>
#include <rthw.h>
#include <string.h>


#ifdef RT_USING_SPI

#include "drivers/pin.h"
#include "drivers/spi.h"
#include "app_spi.h"
#include "app_spi_dma.h"
#include "drv_porting_spi.h"

/*
 * xfer direction
 */
#define XFER_DIR_NONE               0u
#define XFER_DIR_SEND_AND_RECV      1u
#define XFER_DIR_RECV_ONLY          2u
#define XFER_DIR_SEND_ONLY          3u


struct spi_udata_t {
    volatile bool           xferx_err;
    uint32_t                xfer_mode;
    uint32_t                xfer_cspin;     /* specify the cs pin */
    struct rt_semaphore     xfer_semaphore;
    struct rt_spi_bus       spi_bus;
};

struct spi_xfer_context_t {
    volatile bool           xfer_state;
    volatile bool           xfer_err;
    uint32_t                xfer_direction;
    uint32_t                xfer_total_bytes;
    uint32_t                xfer_once_max_bytes;
    uint32_t                xfer_done_bytes;
    uint32_t                xfer_this_bytes;
    uint32_t                xfer_cur_mode;
    uint8_t *               p_send;
    uint8_t *               p_recv;
} ;

static rt_err_t     _spi_configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration);
static rt_uint32_t  _spi_xfer(struct rt_spi_device *device, struct rt_spi_message *message);
static void         _spi_cs_pin_init(void);
static void         _spi_cs_pin_take(void);
static void         _spi_cs_pin_release(void);
static void         _spi_interrupt_callback(app_spi_evt_t *p_evt);
static bool         _spi_send_and_recv(void * send_buf, void * recv_buf, uint32_t total_bytes, uint32_t once_max_bytes);
static bool         _spi_send(void * send_buf, uint32_t total_bytes, uint32_t once_max_bytes);
static bool         _spi_recv(void * recv_buf, uint32_t total_bytes, uint32_t once_max_bytes);


static struct spi_udata_t _gr533x_spi_udata = {
    .xfer_mode  = DEV_SPI_XFER_MODE,
    .xfer_cspin = DEFAULT_SPI_CS_PIN_NUM,
};

static struct spi_xfer_context_t _gr533x_spi_xfer_ctx = {
    .xfer_state              = 0,
    .xfer_err                = 0,
    .xfer_direction          = XFER_DIR_NONE,
    .xfer_total_bytes        = 0,
    .xfer_once_max_bytes     = 0,
    .xfer_done_bytes         = 0,
    .xfer_this_bytes         = 0,
    .xfer_cur_mode           = 0,
    .p_send                  = NULL,
    .p_recv                  = NULL,
};

const static struct rt_spi_ops _gr533x_spi_ops = {
        .configure = _spi_configure,
        .xfer      = _spi_xfer,
    };

static struct rt_spi_bus _gr533x_spi_bus;

/* spi master parameters */
static app_spi_params_t _gr533x_spi_params = {
    .id = APP_SPI_ID_MASTER,
    .pin_cfg = {
       .cs = {
    #if 0
           .type   = DEV_SPI_CS_IO_TYPE,
           .mux    = DEV_SPI_CS_PINMUX,
           .pin    = DEV_SPI_CS_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = APP_IO_PULLUP,
    #endif
           .enable = APP_SPI_PIN_DISABLE,       /* Control CS by RT-Thread Driver Logic */
       },
       .clk  = {
           .type   = DEV_SPI_CLK_IO_TYPE,
           .mux    = DEV_SPI_CLK_PINMUX,
           .pin    = DEV_SPI_CLK_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = DEV_SPI_CLK_PULL,
           .enable = APP_SPI_PIN_ENABLE,
       },
       .mosi = {
           .type   = DEV_SPI_MOSI_IO_TYPE,
           .mux    = DEV_SPI_MOSI_PINMUX,
           .pin    = DEV_SPI_MOSI_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = DEV_SPI_MOSI_PULL,
           .enable = APP_SPI_PIN_ENABLE,
       },
       .miso = {
           .type   = DEV_SPI_MISO_IO_TYPE,
           .mux    = DEV_SPI_MISO_PINMUX,
           .pin    = DEV_SPI_MISO_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = DEV_SPI_MISO_PULL,
           .enable = APP_SPI_PIN_ENABLE,
       },
    },
#if DEV_SPI_XFER_MODE == SPI_XFER_MODE_DMA
    .dma_cfg = {
        .tx_dma_instance = DMA0,
        .rx_dma_instance = DMA0,
        .tx_dma_channel  = DEV_SPI_TX_DMA_CHANNEL,
        .rx_dma_channel  = DEV_SPI_RX_DMA_CHANNEL,
        .wait_timeout_ms = DEV_SPI_WAIT_TIMEOUT_MS,
        .extend = 0,
    },
#endif
    .init = {
        .data_size      = SPI_DATASIZE_8BIT,
        .clock_polarity = SPI_POLARITY_LOW,
        .clock_phase    = SPI_PHASE_1EDGE,
        .baudrate_prescaler = DEV_SPI_CLOCK_DEFAULT_PRESCALER,
        .ti_mode        = SPI_TIMODE_DISABLE,
        .slave_select   = SPI_SLAVE_SELECT_0,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .rx_sample_delay = 0,
#endif
    },

    .is_soft_cs = RT_TRUE,
};


static void _spi_cs_pin_init(void) {
    rt_pin_mode(_gr533x_spi_udata.xfer_cspin, PIN_MODE_OUTPUT);
}


static void _spi_cs_pin_take(void) {
    rt_pin_write(_gr533x_spi_udata.xfer_cspin, PIN_LOW);
}


static void _spi_cs_pin_release(void) {
    rt_pin_write(_gr533x_spi_udata.xfer_cspin, PIN_HIGH);
}


static void _spi_interrupt_callback(app_spi_evt_t *p_evt) {
    uint16_t spi_ret     = 0;
    uint32_t left_bytes  = 0;
    uint32_t xfer_offset = 0;

    switch(p_evt->type) {
        case APP_SPI_EVT_TX_CPLT:
        {
            RT_ASSERT(_gr533x_spi_xfer_ctx.xfer_direction == XFER_DIR_SEND_ONLY);

            _gr533x_spi_xfer_ctx.xfer_done_bytes     += _gr533x_spi_xfer_ctx.xfer_this_bytes;

            if(_gr533x_spi_xfer_ctx.xfer_done_bytes   < _gr533x_spi_xfer_ctx.xfer_total_bytes) {
                left_bytes                            = _gr533x_spi_xfer_ctx.xfer_total_bytes - _gr533x_spi_xfer_ctx.xfer_done_bytes;
                _gr533x_spi_xfer_ctx.xfer_this_bytes  = (left_bytes <= _gr533x_spi_xfer_ctx.xfer_once_max_bytes) ? left_bytes : _gr533x_spi_xfer_ctx.xfer_once_max_bytes;

                xfer_offset = _gr533x_spi_xfer_ctx.xfer_done_bytes;

                if(SPI_XFER_MODE_IRQ == _gr533x_spi_xfer_ctx.xfer_cur_mode) {
                    spi_ret = app_spi_transmit_async(_gr533x_spi_params.id, ((uint8_t*)_gr533x_spi_xfer_ctx.p_send) + xfer_offset, _gr533x_spi_xfer_ctx.xfer_this_bytes);
                } else if(SPI_XFER_MODE_DMA == _gr533x_spi_xfer_ctx.xfer_cur_mode) {
                    spi_ret = app_spi_dma_transmit_async(_gr533x_spi_params.id, ((uint8_t*)_gr533x_spi_xfer_ctx.p_send) + xfer_offset, _gr533x_spi_xfer_ctx.xfer_this_bytes);
                } else {
                    spi_ret = APP_DRV_ERR_INVALID_PARAM;
                }

                if(spi_ret != APP_DRV_SUCCESS) {
                    _gr533x_spi_xfer_ctx.xfer_err = 1;
                    rt_sem_release(&(_gr533x_spi_udata.xfer_semaphore));
                }
            } else {
                _gr533x_spi_xfer_ctx.xfer_err = 0;
                rt_sem_release(&(_gr533x_spi_udata.xfer_semaphore));
            }
        }
        break;

        case APP_SPI_EVT_RX_CPLT:
        {
            RT_ASSERT(_gr533x_spi_xfer_ctx.xfer_direction == XFER_DIR_RECV_ONLY);

            _gr533x_spi_xfer_ctx.xfer_done_bytes     += _gr533x_spi_xfer_ctx.xfer_this_bytes;

            if(_gr533x_spi_xfer_ctx.xfer_done_bytes   < _gr533x_spi_xfer_ctx.xfer_total_bytes) {
                left_bytes                            = _gr533x_spi_xfer_ctx.xfer_total_bytes - _gr533x_spi_xfer_ctx.xfer_done_bytes;
                _gr533x_spi_xfer_ctx.xfer_this_bytes  = (left_bytes <= _gr533x_spi_xfer_ctx.xfer_once_max_bytes) ? left_bytes : _gr533x_spi_xfer_ctx.xfer_once_max_bytes;

                xfer_offset = _gr533x_spi_xfer_ctx.xfer_done_bytes;

                if(SPI_XFER_MODE_IRQ == _gr533x_spi_xfer_ctx.xfer_cur_mode) {
                    spi_ret = app_spi_receive_async(_gr533x_spi_params.id, ((uint8_t*)_gr533x_spi_xfer_ctx.p_recv) + xfer_offset, _gr533x_spi_xfer_ctx.xfer_this_bytes);
                } else if(SPI_XFER_MODE_DMA == _gr533x_spi_xfer_ctx.xfer_cur_mode) {
                    spi_ret = app_spi_dma_receive_async(_gr533x_spi_params.id, ((uint8_t*)_gr533x_spi_xfer_ctx.p_recv) + xfer_offset, _gr533x_spi_xfer_ctx.xfer_this_bytes);
                } else {
                    spi_ret = APP_DRV_ERR_INVALID_PARAM;
                }

                if(spi_ret != APP_DRV_SUCCESS) {
                    _gr533x_spi_xfer_ctx.xfer_err = 1;
                    rt_sem_release(&(_gr533x_spi_udata.xfer_semaphore));
                }
            } else {
                _gr533x_spi_xfer_ctx.xfer_err = 0;
                rt_sem_release(&(_gr533x_spi_udata.xfer_semaphore));
            }
        }
        break;

        case APP_SPI_EVT_TX_RX_CPLT:
        {
            RT_ASSERT(_gr533x_spi_xfer_ctx.xfer_direction == XFER_DIR_SEND_AND_RECV);

            _gr533x_spi_xfer_ctx.xfer_done_bytes     += _gr533x_spi_xfer_ctx.xfer_this_bytes;

            if(_gr533x_spi_xfer_ctx.xfer_done_bytes   < _gr533x_spi_xfer_ctx.xfer_total_bytes) {
                left_bytes                            = _gr533x_spi_xfer_ctx.xfer_total_bytes - _gr533x_spi_xfer_ctx.xfer_done_bytes;
                _gr533x_spi_xfer_ctx.xfer_this_bytes  = (left_bytes <= _gr533x_spi_xfer_ctx.xfer_once_max_bytes) ? left_bytes : _gr533x_spi_xfer_ctx.xfer_once_max_bytes;

                xfer_offset = _gr533x_spi_xfer_ctx.xfer_done_bytes;

                if(SPI_XFER_MODE_IRQ == _gr533x_spi_xfer_ctx.xfer_cur_mode) {
                    spi_ret = app_spi_transmit_receive_async(_gr533x_spi_params.id, ((uint8_t*)_gr533x_spi_xfer_ctx.p_send) + xfer_offset, ((uint8_t*)_gr533x_spi_xfer_ctx.p_recv) + xfer_offset, _gr533x_spi_xfer_ctx.xfer_this_bytes);
                } else if(SPI_XFER_MODE_DMA == _gr533x_spi_xfer_ctx.xfer_cur_mode) {
                    spi_ret = app_spi_dma_transmit_receive_async(_gr533x_spi_params.id, ((uint8_t*)_gr533x_spi_xfer_ctx.p_send) + xfer_offset, ((uint8_t*)_gr533x_spi_xfer_ctx.p_recv) + xfer_offset, _gr533x_spi_xfer_ctx.xfer_this_bytes);
                } else {
                    spi_ret = APP_DRV_ERR_INVALID_PARAM;
                }

                if(spi_ret != APP_DRV_SUCCESS) {
                    _gr533x_spi_xfer_ctx.xfer_err = 1;
                    rt_sem_release(&(_gr533x_spi_udata.xfer_semaphore));
                }
            } else {
                _gr533x_spi_xfer_ctx.xfer_err = 0;
                rt_sem_release(&(_gr533x_spi_udata.xfer_semaphore));
            }
        }
        break;

        case APP_SPI_EVT_ABORT:
        case APP_SPI_EVT_ERROR:
        {
            _gr533x_spi_xfer_ctx.xfer_err = 1;
            rt_sem_release(&(_gr533x_spi_udata.xfer_semaphore));
        }
        break;
    }

}


static rt_err_t _spi_configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration) {
    if(!device || !configuration) {
        return RT_EINVAL;
    }

    if(configuration->mode & RT_SPI_SLAVE) {
        rt_kprintf("spi err: not support spi slave now!\r\n");
        return RT_EINVAL;
    }

    if(configuration->mode & RT_SPI_CPOL) {
        _gr533x_spi_params.init.clock_polarity = SPI_POLARITY_HIGH;
    } else {
        _gr533x_spi_params.init.clock_polarity = SPI_POLARITY_LOW;
    }

    if(configuration->mode & RT_SPI_CPHA) {
        _gr533x_spi_params.init.clock_phase = SPI_PHASE_2EDGE;
    } else {
        _gr533x_spi_params.init.clock_phase = SPI_PHASE_1EDGE;
    }

    int clock_prescale  = DEV_SPI_CLOCK_DEFAULT_PRESCALER;
    int rx_sample_delay = 0;

    if(configuration->max_hz >= 32000000u ) {
        clock_prescale  = 2;
        rx_sample_delay = 1;
    } else {
        clock_prescale = 64000000/configuration->max_hz;
        clock_prescale = (clock_prescale >> 1) << 1;
    }

    _gr533x_spi_params.init.baudrate_prescaler = clock_prescale;
    _gr533x_spi_params.init.rx_sample_delay    = rx_sample_delay;

    if(8 == configuration->data_width) {
        _gr533x_spi_params.init.data_size = SPI_DATASIZE_8BIT;
    } else if(16 == configuration->data_width) {
        _gr533x_spi_params.init.data_size = SPI_DATASIZE_16BIT;
    } else if(32 == configuration->data_width) {
        _gr533x_spi_params.init.data_size = SPI_DATASIZE_32BIT;
    }


    app_drv_err_t ret = 0;

    ret = app_spi_init(&_gr533x_spi_params, _spi_interrupt_callback);
    if (ret != 0)
    {
        rt_kprintf("SPI master initial failed! Please check the input paraments.\r\n");

        return RT_ERROR;
    }

#if DEV_SPI_XFER_MODE == SPI_XFER_MODE_DMA
    bool is_tx_dma_init = true;
    bool is_rx_dma_init = true;

    _gr533x_spi_params.dma_cfg.tx_dma_instance = DMA0;
    _gr533x_spi_params.dma_cfg.rx_dma_instance = DMA0;
    _gr533x_spi_params.dma_cfg.tx_dma_channel  = DEV_SPI_TX_DMA_CHANNEL;
    _gr533x_spi_params.dma_cfg.rx_dma_channel  = DEV_SPI_RX_DMA_CHANNEL;

    if(_gr533x_spi_params.dma_cfg.tx_dma_channel == DMA_Channel_DISABLE) {
        _gr533x_spi_params.dma_cfg.tx_dma_instance = RT_NULL;
        is_tx_dma_init = false;
    }

    if(_gr533x_spi_params.dma_cfg.rx_dma_channel == DMA_Channel_DISABLE) {
        _gr533x_spi_params.dma_cfg.rx_dma_instance = RT_NULL;
        is_rx_dma_init = false;
    }

    if(is_tx_dma_init || is_rx_dma_init) {
        ret = app_spi_dma_init(&_gr533x_spi_params);
        if (ret != 0)
        {
            rt_kprintf("SPI master dma initial failed! Please check the input paraments.\r\n");
            return RT_ERROR;
        }
    }

#endif

    _spi_cs_pin_init();

    device->config = * configuration;

    return 0;
}


static bool _spi_send_and_recv(void * send_buf, void * recv_buf, uint32_t total_bytes, uint32_t once_max_bytes) {
    bool     ret = false;
    uint16_t spi_ret = 0;
    uint32_t this_xfer_bytes = 0;
    uint32_t xfer_offset = 0;

    switch(_gr533x_spi_udata.xfer_mode) {
        case SPI_XFER_MODE_POLL:
        {
            this_xfer_bytes = 0;
            xfer_offset     = 0;

            do {
                this_xfer_bytes = (total_bytes <= once_max_bytes) ? total_bytes : once_max_bytes;
                spi_ret = app_spi_transmit_receive_sync(_gr533x_spi_params.id, ((uint8_t*)send_buf) + xfer_offset, ((uint8_t*)recv_buf) + xfer_offset, this_xfer_bytes, DEV_SPI_WAIT_TIMEOUT_MS);
                total_bytes -= this_xfer_bytes;
                xfer_offset += this_xfer_bytes;

                if(spi_ret != APP_DRV_SUCCESS) {
                    rt_kprintf("spi poll send_recv err: %d\r\n", spi_ret);
                    break;
                }
            } while(total_bytes > 0);

            if(spi_ret == APP_DRV_SUCCESS) {
                ret = true;
            }
        }
        break;

        case SPI_XFER_MODE_IRQ:
        case SPI_XFER_MODE_DMA:
        {
            if(_gr533x_spi_xfer_ctx.xfer_state != 0) {
                rt_kprintf("spi is xferring...\n");
                break;
            }

            _gr533x_spi_xfer_ctx.xfer_err = 0;

            _gr533x_spi_xfer_ctx.xfer_direction      = XFER_DIR_SEND_AND_RECV;
            _gr533x_spi_xfer_ctx.xfer_done_bytes     = 0;
            _gr533x_spi_xfer_ctx.xfer_once_max_bytes = once_max_bytes;
            _gr533x_spi_xfer_ctx.xfer_total_bytes    = total_bytes;
            _gr533x_spi_xfer_ctx.xfer_this_bytes     = (total_bytes <= once_max_bytes) ? total_bytes : once_max_bytes;
            _gr533x_spi_xfer_ctx.p_send              = (uint8_t*)send_buf;
            _gr533x_spi_xfer_ctx.p_recv              = (uint8_t*)recv_buf;
            _gr533x_spi_xfer_ctx.xfer_cur_mode       = _gr533x_spi_udata.xfer_mode;
            _gr533x_spi_xfer_ctx.xfer_state          = 1;

            this_xfer_bytes = _gr533x_spi_xfer_ctx.xfer_this_bytes;
            xfer_offset     = 0;

            if(SPI_XFER_MODE_IRQ == _gr533x_spi_udata.xfer_mode) {
                spi_ret = app_spi_transmit_receive_async(_gr533x_spi_params.id, (uint8_t*)send_buf, (uint8_t*)recv_buf, this_xfer_bytes);
            } else if(SPI_XFER_MODE_DMA == _gr533x_spi_udata.xfer_mode) {
                spi_ret = app_spi_dma_transmit_receive_async(_gr533x_spi_params.id, (uint8_t*)send_buf, (uint8_t*)recv_buf, this_xfer_bytes);
            }

            if(spi_ret == APP_DRV_SUCCESS) {
                rt_sem_take(&(_gr533x_spi_udata.xfer_semaphore), RT_WAITING_FOREVER);

                if(!_gr533x_spi_xfer_ctx.xfer_err) {
                    ret = true;
                }
                memset(&_gr533x_spi_xfer_ctx, 0, sizeof(_gr533x_spi_xfer_ctx));
            } else {
                memset(&_gr533x_spi_xfer_ctx, 0, sizeof(_gr533x_spi_xfer_ctx));
                rt_kprintf("spi send_recv err: %d\r\n", spi_ret);
            }
        }
        break;
    }

    return ret;
}

static bool _spi_send(void * send_buf, uint32_t total_bytes, uint32_t once_max_bytes) {
    bool ret         = false;
    uint16_t spi_ret = 0;
    uint32_t this_xfer_bytes = 0;
    uint32_t xfer_offset     = 0;

    switch(_gr533x_spi_udata.xfer_mode) {
        case SPI_XFER_MODE_POLL:
        {
            this_xfer_bytes = 0;
            xfer_offset     = 0;

            do {
                this_xfer_bytes = (total_bytes <= once_max_bytes) ? total_bytes : once_max_bytes;

                spi_ret = app_spi_transmit_sync(_gr533x_spi_params.id, ((uint8_t*)send_buf) + xfer_offset, this_xfer_bytes, DEV_SPI_WAIT_TIMEOUT_MS);

                total_bytes -= this_xfer_bytes;
                xfer_offset += this_xfer_bytes;

                if(spi_ret != APP_DRV_SUCCESS) {
                    rt_kprintf("spi poll send err: %d\r\n", spi_ret);
                    break;
                }
            } while(total_bytes > 0);

            if(spi_ret == APP_DRV_SUCCESS) {
                ret = true;
            }
        }
        break;

        case SPI_XFER_MODE_IRQ:
        case SPI_XFER_MODE_DMA:
        {

            if(_gr533x_spi_xfer_ctx.xfer_state != 0) {
                rt_kprintf("spi is xferring...\n");
                break;
            }

            _gr533x_spi_xfer_ctx.xfer_err = 0;

            _gr533x_spi_xfer_ctx.xfer_direction      = XFER_DIR_SEND_ONLY;
            _gr533x_spi_xfer_ctx.xfer_done_bytes     = 0;
            _gr533x_spi_xfer_ctx.xfer_once_max_bytes = once_max_bytes;
            _gr533x_spi_xfer_ctx.xfer_total_bytes    = total_bytes;
            _gr533x_spi_xfer_ctx.xfer_this_bytes     = (total_bytes <= once_max_bytes) ? total_bytes : once_max_bytes;
            _gr533x_spi_xfer_ctx.p_send              = (uint8_t*)send_buf;
            _gr533x_spi_xfer_ctx.p_recv              = (uint8_t*)NULL;
            _gr533x_spi_xfer_ctx.xfer_cur_mode       = _gr533x_spi_udata.xfer_mode;
            _gr533x_spi_xfer_ctx.xfer_state          = 1;

            this_xfer_bytes = _gr533x_spi_xfer_ctx.xfer_this_bytes;
            xfer_offset     = 0;


            if(SPI_XFER_MODE_IRQ == _gr533x_spi_udata.xfer_mode) {
                spi_ret = app_spi_transmit_async(_gr533x_spi_params.id, (uint8_t*)send_buf, this_xfer_bytes);
            } else if(SPI_XFER_MODE_DMA == _gr533x_spi_udata.xfer_mode) {
                spi_ret = app_spi_dma_transmit_async(_gr533x_spi_params.id, (uint8_t*)send_buf, this_xfer_bytes);
            }

            if(spi_ret == APP_DRV_SUCCESS) {
                rt_sem_take(&(_gr533x_spi_udata.xfer_semaphore), RT_WAITING_FOREVER);
                if(!_gr533x_spi_xfer_ctx.xfer_err) {
                    ret = true;
                }
                memset(&_gr533x_spi_xfer_ctx, 0, sizeof(_gr533x_spi_xfer_ctx));
            } else {
                memset(&_gr533x_spi_xfer_ctx, 0, sizeof(_gr533x_spi_xfer_ctx));
                rt_kprintf("spi send err: %d\r\n", spi_ret);
            }

        }
        break;

    }

    return ret;
}

static bool _spi_recv(void * recv_buf, uint32_t total_bytes, uint32_t once_max_bytes) {
    bool ret         = false;
    uint16_t spi_ret = 0;
    uint32_t this_xfer_bytes = 0;
    uint32_t xfer_offset     = 0;

    switch(_gr533x_spi_udata.xfer_mode) {
        case SPI_XFER_MODE_POLL:
        {
            this_xfer_bytes = 0;
            xfer_offset     = 0;

            do {
                this_xfer_bytes = (total_bytes <= once_max_bytes) ? total_bytes : once_max_bytes;

                spi_ret = app_spi_receive_sync(_gr533x_spi_params.id, ((uint8_t*)recv_buf) + xfer_offset, this_xfer_bytes, DEV_SPI_WAIT_TIMEOUT_MS);

                total_bytes -= this_xfer_bytes;
                xfer_offset += this_xfer_bytes;

                if(spi_ret != APP_DRV_SUCCESS) {
                    rt_kprintf("spi poll recv err: %d\r\n", spi_ret);
                    break;
                }
            } while(total_bytes > 0);

            if(spi_ret == APP_DRV_SUCCESS) {
                ret = true;
            }
        }
        break;

        case SPI_XFER_MODE_IRQ:
        case SPI_XFER_MODE_DMA:
        {
            if(_gr533x_spi_xfer_ctx.xfer_state != 0) {
                rt_kprintf("spi is xferring...\n");
                break;
            }

            _gr533x_spi_xfer_ctx.xfer_err = 0;

            _gr533x_spi_xfer_ctx.xfer_direction      = XFER_DIR_RECV_ONLY;
            _gr533x_spi_xfer_ctx.xfer_done_bytes     = 0;
            _gr533x_spi_xfer_ctx.xfer_once_max_bytes = once_max_bytes;
            _gr533x_spi_xfer_ctx.xfer_total_bytes    = total_bytes;
            _gr533x_spi_xfer_ctx.xfer_this_bytes     = (total_bytes <= once_max_bytes) ? total_bytes : once_max_bytes;
            _gr533x_spi_xfer_ctx.p_send              = (uint8_t*)NULL;
            _gr533x_spi_xfer_ctx.p_recv              = (uint8_t*)recv_buf;
            _gr533x_spi_xfer_ctx.xfer_cur_mode       = _gr533x_spi_udata.xfer_mode;
            _gr533x_spi_xfer_ctx.xfer_state          = 1;

            this_xfer_bytes = _gr533x_spi_xfer_ctx.xfer_this_bytes;
            xfer_offset     = 0;


            if(SPI_XFER_MODE_IRQ == _gr533x_spi_udata.xfer_mode) {
                spi_ret = app_spi_receive_async(_gr533x_spi_params.id, (uint8_t*)recv_buf, this_xfer_bytes);
            } else if(SPI_XFER_MODE_DMA == _gr533x_spi_udata.xfer_mode) {
                spi_ret = app_spi_dma_receive_async(_gr533x_spi_params.id, (uint8_t*)recv_buf, this_xfer_bytes);
            }

            if(spi_ret == APP_DRV_SUCCESS) {
                rt_sem_take(&(_gr533x_spi_udata.xfer_semaphore), RT_WAITING_FOREVER);
                if(!_gr533x_spi_xfer_ctx.xfer_err) {
                    ret = true;
                }
                memset(&_gr533x_spi_xfer_ctx, 0, sizeof(_gr533x_spi_xfer_ctx));
            } else {
                memset(&_gr533x_spi_xfer_ctx, 0, sizeof(_gr533x_spi_xfer_ctx));
                rt_kprintf("spi recv err: %d\r\n", spi_ret);
            }

        }
        break;
    }

    return ret;
}


static rt_uint32_t _spi_xfer(struct rt_spi_device *device, struct rt_spi_message *message) {

    if(!device || !message) {
        return RT_EINVAL;
    }

    struct rt_spi_message * cur_msg = message;
    uint16_t beat_bytes = 1;
    bool ret = 0;

    spi_handle_t * p_handle = app_spi_get_handle(_gr533x_spi_params.id);

    if(_gr533x_spi_params.init.data_size == SPI_DATASIZE_8BIT) {
        beat_bytes = 1;
    } else if(_gr533x_spi_params.init.data_size == SPI_DATASIZE_16BIT) {
        beat_bytes = 2;
    } else if(_gr533x_spi_params.init.data_size == SPI_DATASIZE_32BIT) {
        beat_bytes = 4;
    }

    do {

        //if((cur_msg->length == 0) || (cur_msg->length > SPI_ONCE_XFER_MAX_BEAT))
        if(cur_msg->length == 0)
        {
            rt_kprintf("spi xfer, error length : %d \r\n", cur_msg->length);
            //return RT_EINVAL;
        }

        /* user may just want to take cs */
        if(cur_msg->cs_take) {
            _spi_cs_pin_take();
        }

        if((cur_msg->length > 0) && /*(cur_msg->length <= SPI_ONCE_XFER_MAX_BEAT) && */
           (cur_msg->send_buf || cur_msg->recv_buf)) {

            if(cur_msg->send_buf && cur_msg->recv_buf) {
                ret = _spi_send_and_recv((uint8_t*)cur_msg->send_buf, (uint8_t*)cur_msg->recv_buf, cur_msg->length * beat_bytes, SPI_ONCE_XFER_MAX_BEAT * beat_bytes);
                if(!ret) {
                    return RT_ERROR;
                }
            } else if(!cur_msg->send_buf && cur_msg->recv_buf) {
                ret = _spi_recv((uint8_t*)cur_msg->recv_buf, cur_msg->length * beat_bytes, SPI_ONCE_XFER_MAX_BEAT * beat_bytes);
                if(!ret) {
                    return RT_ERROR;
                }
            } else if(cur_msg->send_buf && !cur_msg->recv_buf) {
                ret = _spi_send((uint8_t*)cur_msg->send_buf, cur_msg->length * beat_bytes, SPI_ONCE_XFER_MAX_BEAT * beat_bytes);
                if(!ret) {
                    return RT_ERROR;
                }
            }
        }

        /* user may just want to release cs */
        if(cur_msg->cs_release) {
            _spi_cs_pin_release();
        }

    } while(0);

    return message->length;
}

void rt_hw_spi_switch_cspin(uint32_t pin_num) {
    _gr533x_spi_udata.xfer_cspin = pin_num;

    rt_pin_mode(_gr533x_spi_udata.xfer_cspin,  PIN_MODE_OUTPUT);
    rt_pin_write(_gr533x_spi_udata.xfer_cspin, PIN_HIGH);

    return;
}


int rt_hw_spi_init(void)
{
    int result;

    result = rt_spi_bus_register(&_gr533x_spi_bus, DEV_SPI_0, &_gr533x_spi_ops);

    rt_sem_init(&_gr533x_spi_udata.xfer_semaphore, "spi0", 0, RT_IPC_FLAG_FIFO);

    return result;
}

INIT_BOARD_EXPORT(rt_hw_spi_init);


#endif /* RT_USING_SPI */
