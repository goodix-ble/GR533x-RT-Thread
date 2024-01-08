#include <rtdevice.h>
#include <rthw.h>
#include <string.h>

#ifdef RT_USING_I2C

#include "app_i2c.h"
#include "app_i2c_dma.h"
#include "drv_porting_i2c.h"


struct i2c_udata_t
{
    uint8_t             id;
    uint8_t             role;
    uint8_t             xfer_mode;
    volatile uint8_t    xfer_err;
    i2c_regs_t *        p_instance;
    void *              p_config;
    struct rt_semaphore xfer_semaphore;
};


static rt_size_t _master_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num);
static rt_size_t _slave_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num);
static rt_err_t  _i2c_bus_control(struct rt_i2c_bus_device *bus, rt_uint32_t, rt_uint32_t);
static void      _i2c_bus_init(void);

static const struct rt_i2c_bus_device_ops _gr533x_i2c_ops = {
        .master_xfer     = _master_xfer,
        .slave_xfer      = _slave_xfer,
        .i2c_bus_control = _i2c_bus_control,
    };


#if DEV_I2C_0_ENABLE

static app_i2c_params_t         _gr533x_i2c0_config = {
    .id      = APP_I2C_ID_0,
    .role    = APP_I2C_ROLE_MASTER,
    .pin_cfg = {
        .scl = {
            .type = DEV_I2C_0_SCL_IO_TYPE,
            .mux  = DEV_I2C_0_SCL_PINMUX,
            .pin  = DEV_I2C_0_SCL_PIN,
            .pull = DEV_I2C_0_SCL_PULL,
        },
        .sda = {
            .type = DEV_I2C_0_SDA_IO_TYPE,
            .mux  = DEV_I2C_0_SDA_PINMUX,
            .pin  = DEV_I2C_0_SDA_PIN,
            .pull = DEV_I2C_0_SDA_PULL,
        },
    },
    .init   = {
        .speed             = DEV_I2C_0_SPEED,
        .own_address       = 0x00,
        .addressing_mode   = I2C_ADDRESSINGMODE_7BIT,
        .general_call_mode = I2C_GENERALCALL_DISABLE,
    },
};
static struct i2c_udata_t       _gr533x_i2c0_udata = {
    .id         = APP_I2C_ID_0,
    .role       = APP_I2C_ROLE_MASTER,
    .xfer_mode  = DEV_I2C_0_XFER_MODE,
    .xfer_err   = 0,
    .p_instance = I2C0,
    .p_config   = &_gr533x_i2c0_config,
};
static struct rt_i2c_bus_device _gr533x_i2c0_bus;

#endif


#if DEV_I2C_1_ENABLE

static app_i2c_params_t         _gr533x_i2c1_config = {
    .id      = APP_I2C_ID_1,
    .role    = APP_I2C_ROLE_MASTER,
    .pin_cfg = {
        .scl = {
            .type = DEV_I2C_1_SCL_IO_TYPE,
            .mux  = DEV_I2C_1_SCL_PINMUX,
            .pin  = DEV_I2C_1_SCL_PIN,
            .pull = DEV_I2C_1_SCL_PULL,
        },
        .sda = {
            .type = DEV_I2C_1_SDA_IO_TYPE,
            .mux  = DEV_I2C_1_SDA_PINMUX,
            .pin  = DEV_I2C_1_SDA_PIN,
            .pull = DEV_I2C_1_SDA_PULL,
        },
    },
    .init   = {
        .speed             = DEV_I2C_1_SPEED,
        .own_address       = 0x00,
        .addressing_mode   = I2C_ADDRESSINGMODE_7BIT,
        .general_call_mode = I2C_GENERALCALL_DISABLE,
    },
};

static struct i2c_udata_t       _gr533x_i2c1_udata = {
    .id         = APP_I2C_ID_1,
    .role       = APP_I2C_ROLE_MASTER,
    .xfer_mode  = DEV_I2C_1_XFER_MODE,
    .xfer_err   = 0,
    .p_instance = I2C1,
    .p_config   = &_gr533x_i2c1_config,
};

static struct rt_i2c_bus_device _gr533x_i2c1_bus;

#endif

extern bool app_i2c_check_busy(app_i2c_id_t id);

static void _adjust_i2c_addr_bits(struct i2c_udata_t * p_udata, bool is_10bit) {
    while(app_i2c_check_busy((app_i2c_id_t)p_udata->id)) {
        delay_ms(2);
    }
    if(is_10bit) {
        ll_i2c_disable(p_udata->p_instance);
        ll_i2c_set_own_address(p_udata->p_instance, ((app_i2c_params_t*)p_udata->p_config)->init.own_address, LL_I2C_OWNADDRESS_10BIT);
        ll_i2c_set_master_addressing_mode(p_udata->p_instance, LL_I2C_ADDRESSING_MODE_10BIT);
    } else {
        ll_i2c_disable(p_udata->p_instance);
        ll_i2c_set_own_address(p_udata->p_instance, ((app_i2c_params_t*)p_udata->p_config)->init.own_address, LL_I2C_OWNADDRESS_7BIT);
        ll_i2c_set_master_addressing_mode(p_udata->p_instance, LL_I2C_ADDRESSING_MODE_7BIT);
    }
}


static rt_size_t _master_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num) {

    uint16_t  i      = 0;
    uint16_t  ret    = APP_DRV_SUCCESS;
    rt_size_t rt_ret = 0;
    struct i2c_udata_t * p_udata = (struct i2c_udata_t *)bus->priv;

    static bool is_10bits_set = false;
    static bool is_7bits_set  = false;

    for (i = 0; i < num; i++) {

        if(msgs[i].flags & RT_I2C_ADDR_10BIT) {
            if(!is_10bits_set) {
                _adjust_i2c_addr_bits(p_udata, true);
                is_10bits_set = true;
                is_7bits_set  = false;
            }
        } else {
            if(!is_7bits_set) {
                _adjust_i2c_addr_bits(p_udata, false);
                is_10bits_set = false;
                is_7bits_set  = true;
            }
        }

        if(msgs[i].flags & RT_I2C_RD) {
            switch(p_udata->xfer_mode) {
                case I2C_XFER_MODE_POLL:
                {
                    ret = app_i2c_receive_sync((app_i2c_id_t)p_udata->id, msgs[i].addr, msgs[i].buf, msgs[i].len, DEV_I2C_POLL_TIMEOUT);
                    if(ret == APP_DRV_SUCCESS) {
                        rt_ret = msgs[i].len;
                    }
                }
                break;

                case I2C_XFER_MODE_IRQ:
                {
                    p_udata->xfer_err = 0;
                    ret = app_i2c_receive_async((app_i2c_id_t)p_udata->id, msgs[i].addr, msgs[i].buf, msgs[i].len);

                    if(ret == APP_DRV_SUCCESS) {

                        //Take Semophare
                        rt_sem_take(&p_udata->xfer_semaphore, RT_WAITING_FOREVER);

                        if(!p_udata->xfer_err) {
                            rt_ret = msgs[i].len;
                        }
                    }
                    delay_ms(2);   //a little delay to avoid reentry
                }
                break;

                case I2C_XFER_MODE_DMA:
                {
                    p_udata->xfer_err = 0;
                    ret = app_i2c_dma_receive_async((app_i2c_id_t)p_udata->id, msgs[i].addr, msgs[i].buf, msgs[i].len);

                    if(ret == APP_DRV_SUCCESS) {

                        //Take Semophare
                        rt_sem_take(&p_udata->xfer_semaphore, RT_WAITING_FOREVER);

                        if(!p_udata->xfer_err) {
                            rt_ret = msgs[i].len;
                        }
                    }
                    delay_ms(2);
                }
                break;
            }
        } else {
            switch(p_udata->xfer_mode) {
                case I2C_XFER_MODE_POLL:
                {
                    ret = app_i2c_transmit_sync((app_i2c_id_t)p_udata->id, msgs[i].addr, msgs[i].buf, msgs[i].len, DEV_I2C_POLL_TIMEOUT);
                    if(ret == APP_DRV_SUCCESS) {
                        rt_ret = msgs[i].len;
                    }
                }
                break;

                case I2C_XFER_MODE_IRQ:
                {
                    p_udata->xfer_err = 0;
                    ret = app_i2c_transmit_async((app_i2c_id_t)p_udata->id, msgs[i].addr, msgs[i].buf, msgs[i].len);

                    if(ret == APP_DRV_SUCCESS) {

                        //Take Semophare
                        rt_sem_take(&p_udata->xfer_semaphore, RT_WAITING_FOREVER);

                        if(!p_udata->xfer_err) {
                            rt_ret = msgs[i].len;
                        }
                    }
                    delay_ms(2);
                }
                break;

                case I2C_XFER_MODE_DMA:
                {
                    p_udata->xfer_err = 0;
                    ret = app_i2c_dma_transmit_async((app_i2c_id_t)p_udata->id, msgs[i].addr, msgs[i].buf, msgs[i].len);

                    if(ret == APP_DRV_SUCCESS) {

                        //Take Semophare
                        rt_sem_take(&p_udata->xfer_semaphore, RT_WAITING_FOREVER);

                        if(!p_udata->xfer_err) {
                            rt_ret = msgs[i].len;
                        }

                    }
                    delay_ms(2);
                }
                break;
            }
        }
    }

    return rt_ret;
}


static rt_size_t _slave_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num) {
    return _master_xfer(bus, msgs, num);
}


static rt_err_t  _i2c_bus_control(struct rt_i2c_bus_device *bus, rt_uint32_t cmd, rt_uint32_t arg) {
    return RT_EOK;
}


#if DEV_I2C_0_ENABLE
static void _i2c_0_interrupt_callback(app_i2c_evt_t * p_evt) {
    switch (p_evt->type)
    {
        case APP_I2C_ABORT:
        case APP_I2C_EVT_ERROR:
            _gr533x_i2c0_udata.xfer_err = 1;
            break;

        case APP_I2C_EVT_TX_CPLT:
            _gr533x_i2c0_udata.xfer_err = 0;
            break;

        case APP_I2C_EVT_RX_DATA:
            _gr533x_i2c0_udata.xfer_err = 0;
            break;
    }

    rt_sem_release(&_gr533x_i2c0_udata.xfer_semaphore);
}
#endif

#if DEV_I2C_1_ENABLE
static void _i2c_1_interrupt_callback(app_i2c_evt_t * p_evt) {
    switch (p_evt->type)
    {
        case APP_I2C_ABORT:
        case APP_I2C_EVT_ERROR:
            _gr533x_i2c1_udata.xfer_err = 1;
            break;

        case APP_I2C_EVT_TX_CPLT:
            _gr533x_i2c1_udata.xfer_err = 0;
            break;

        case APP_I2C_EVT_RX_DATA:
            _gr533x_i2c1_udata.xfer_err = 0;
            break;
    }

    rt_sem_release(&_gr533x_i2c1_udata.xfer_semaphore);

}
#endif

static void _i2c_bus_init(void) {
    uint16_t ret = 0;

#if DEV_I2C_0_ENABLE

    _gr533x_i2c0_udata.xfer_mode   = DEV_I2C_0_XFER_MODE;
    _gr533x_i2c0_config.init.speed = DEV_I2C_0_SPEED;

    if(_gr533x_i2c0_udata.xfer_mode == I2C_XFER_MODE_POLL) {
        ret = app_i2c_init(&_gr533x_i2c0_config, NULL);
    } else {
        ret = app_i2c_init(&_gr533x_i2c0_config, _i2c_0_interrupt_callback);
    }
    if (ret != 0)
    {
        rt_kprintf("i2c0 init failed : %d \r\n", ret);
        return;
    }

    #if DEV_I2C_0_XFER_MODE  == I2C_XFER_MODE_DMA
    {
        bool is_i2c0_tx_init = true;
        bool is_i2c0_rx_init = true;

        _gr533x_i2c0_config.dma_cfg.tx_dma_instance = DMA0;
        _gr533x_i2c0_config.dma_cfg.rx_dma_instance = DMA0;
        _gr533x_i2c0_config.dma_cfg.tx_dma_channel  = DEV_I2C_0_TX_DMA_CHANNEL;
        _gr533x_i2c0_config.dma_cfg.rx_dma_channel  = DEV_I2C_0_RX_DMA_CHANNEL;

        if(DMA_Channel_DISABLE == _gr533x_i2c0_config.dma_cfg.tx_dma_channel) {
            _gr533x_i2c0_config.dma_cfg.tx_dma_instance = RT_NULL;
            is_i2c0_tx_init                             = false;
        }

        if(DMA_Channel_DISABLE == _gr533x_i2c0_config.dma_cfg.rx_dma_channel) {
            _gr533x_i2c0_config.dma_cfg.rx_dma_instance = RT_NULL;
            is_i2c0_rx_init                             = false;
        }

        if(is_i2c0_tx_init || is_i2c0_rx_init) {
            ret = app_i2c_dma_init(&_gr533x_i2c0_config);

            if (ret != 0)
            {
                rt_kprintf("i2c0 dma init failed : %d \r\n", ret);
                return;
            }
        }
    }
    #endif

    rt_sem_init(&_gr533x_i2c0_udata.xfer_semaphore, "i2c0", 1, RT_IPC_FLAG_FIFO);
    rt_sem_take(&_gr533x_i2c0_udata.xfer_semaphore, RT_WAITING_FOREVER);

#endif

#if DEV_I2C_1_ENABLE

    _gr533x_i2c1_udata.xfer_mode   = DEV_I2C_1_XFER_MODE;
    _gr533x_i2c1_config.init.speed = DEV_I2C_1_SPEED;

    if(_gr533x_i2c1_udata.xfer_mode == I2C_XFER_MODE_POLL) {
        ret = app_i2c_init(&_gr533x_i2c1_config, NULL);
    } else {
        ret = app_i2c_init(&_gr533x_i2c1_config, _i2c_1_interrupt_callback);
    }
    if (ret != 0)
    {
        rt_kprintf("i2c1 init failed : %d \r\n", ret);
        return;
    }

    #if DEV_I2C_1_XFER_MODE  == I2C_XFER_MODE_DMA
    {
        bool is_i2c1_tx_init = true;
        bool is_i2c1_rx_init = true;

        _gr533x_i2c1_config.dma_cfg.tx_dma_instance = DMA0;
        _gr533x_i2c1_config.dma_cfg.rx_dma_instance = DMA0;
        _gr533x_i2c1_config.dma_cfg.tx_dma_channel  = DEV_I2C_1_TX_DMA_CHANNEL;
        _gr533x_i2c1_config.dma_cfg.rx_dma_channel  = DEV_I2C_1_RX_DMA_CHANNEL;

        if(DMA_Channel_DISABLE == _gr533x_i2c1_config.dma_cfg.tx_dma_channel) {
            _gr533x_i2c1_config.dma_cfg.tx_dma_instance = RT_NULL;
            is_i2c1_tx_init                             = false;
        }

        if(DMA_Channel_DISABLE == _gr533x_i2c1_config.dma_cfg.rx_dma_channel) {
            _gr533x_i2c1_config.dma_cfg.rx_dma_instance = RT_NULL;
            is_i2c1_rx_init                             = false;
        }

        if(is_i2c1_tx_init || is_i2c1_rx_init) {
            ret = app_i2c_dma_init(&_gr533x_i2c1_config);

            if (ret != 0)
            {
                rt_kprintf("i2c1 dma init failed : %d \r\n", ret);
                return;
            }
        }
    }
    #endif

    rt_sem_init(&_gr533x_i2c1_udata.xfer_semaphore, "i2c1", 1, RT_IPC_FLAG_FIFO);
    rt_sem_take(&_gr533x_i2c1_udata.xfer_semaphore, RT_WAITING_FOREVER);
#endif
    return;
}


int rt_hw_i2c_init(void)
{
    int result = 0;

    _i2c_bus_init();

#if DEV_I2C_0_ENABLE
    _gr533x_i2c0_bus.ops  = &_gr533x_i2c_ops;
    _gr533x_i2c0_bus.priv = (void*)&_gr533x_i2c0_udata;
    result                = rt_i2c_bus_device_register(&_gr533x_i2c0_bus, DEV_I2C_0);
#endif

#if DEV_I2C_1_ENABLE
    _gr533x_i2c1_bus.ops  = &_gr533x_i2c_ops;
    _gr533x_i2c1_bus.priv = (void*)&_gr533x_i2c1_udata;
    result               |= rt_i2c_bus_device_register(&_gr533x_i2c1_bus, DEV_I2C_1);
#endif

    return result;
}

INIT_BOARD_EXPORT(rt_hw_i2c_init);


#endif /* RT_USING_I2C */
