#include <rtdevice.h>
#include <rthw.h>
#include <string.h>

#ifdef RT_USING_SERIAL

#include "app_uart.h"
#include "app_uart_dma.h"
#include "drv_porting_uart.h"


struct uart_udata_t
{
    uint8_t                     id;
    uint32_t                    tx_mode;                /* t-xfer mode, poll in default */
    uint32_t                    rx_mode;                /* r-xfer mode, poll in default */
    uint8_t *                   uart_tx_buffer;
    uint8_t *                   uart_rx_buffer;         /* used for buffer when recv with irq and dma */
    volatile uint16_t           buffer_size;            /* same for tx&rx buffer */
    volatile uint16_t           recv_size;              /* record recv size */
    volatile uint16_t           recv_pos;               /* record recv position */
    volatile uint16_t           xfer_err;
    volatile uint8_t            is_dma_tx_inited;
    volatile uint8_t            is_dma_rx_inited;
    rt_thread_t                 uart_rx_thread;
    struct rt_semaphore         tx_semaphore;
    struct rt_semaphore         rx_semaphore;
    app_uart_params_t           uart_init;
    rt_serial_t                 uart_bus;
};

/* state to handle the parameters transition */
enum {
    _CONV_DATABITS = 0,
    _CONV_STOPBITS,
    _CONV_PARITY,
    _CONV_FLOW_CTRL,
};


/* state to handle the rx thread for uart */
enum {
    THREAD_STARTUP = 1,
    THREAD_SUSPEND,
    THREAD_RESUME,
    THREAD_DELETE,
};


#define GET_ID(serial)  ((struct uart_udata_t *)(serial)->parent.user_data)->id


static rt_err_t     _configure(struct rt_serial_device *serial, struct serial_configure *cfg);
static rt_err_t     _control(struct rt_serial_device *serial, int cmd, void *arg);
static int          _putc(struct rt_serial_device *serial, char c);
static int          _getc(struct rt_serial_device *serial);
static rt_size_t    _dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction);
static uint32_t     _convert_serial_conf(uint32_t conv_type, uint32_t val);
//static bool         _control_uart_rx_thread(struct uart_udata_t * p_udata, uint32_t state);


static struct uart_udata_t _gr533x_uart0_udata = {
    .id          = APP_UART_ID_0,
    .tx_mode     = 0,
    .rx_mode     = 0,
    .buffer_size = 0,
    .recv_size   = 0,
    .recv_pos    = 0,
    .xfer_err    = false,
    .is_dma_tx_inited = false,
    .is_dma_rx_inited = false,
    .uart_tx_buffer   = RT_NULL,
    .uart_rx_buffer   = RT_NULL,
    .uart_rx_thread   = RT_NULL,
};

static struct uart_udata_t _gr533x_uart1_udata  = {
    .id          = APP_UART_ID_1,
    .tx_mode     = 0,
    .rx_mode     = 0,
    .buffer_size = 0,
    .recv_size   = 0,
    .recv_pos    = 0,
    .xfer_err    = false,
    .is_dma_tx_inited = false,
    .is_dma_rx_inited = false,
    .uart_tx_buffer   = RT_NULL,
    .uart_rx_buffer   = RT_NULL,
    .uart_rx_thread   = RT_NULL,
};

const static struct rt_uart_ops _gr533x_uart_ops = {
    .configure      = _configure,
    .control        = _control,
    .putc           = _putc,
    .getc           = _getc,
    .dma_transmit   = _dma_transmit,
};

const static struct serial_configure _gr533x_uart0_conf = {
    .baud_rate   = DEV_UART0_CFG_BAUD_RATE ,
    .data_bits   = DEV_UART0_CFG_DATA_BITS ,
    .stop_bits   = DEV_UART0_CFG_STOP_BITS ,
    .parity      = DEV_UART0_CFG_PARITY    ,
    .bit_order   = DEV_UART0_CFG_BIT_ORDER ,
    .invert      = DEV_UART0_CFG_INVERT    ,
    .bufsz       = DEV_UART0_CFG_BUFSZ     ,
    .flowcontrol = DEV_UART0_CFG_FLOW_CTRL ,
    .reserved    = 0 ,
};

const static struct serial_configure _gr533x_uart1_conf = {
    .baud_rate   = DEV_UART1_CFG_BAUD_RATE ,
    .data_bits   = DEV_UART1_CFG_DATA_BITS ,
    .stop_bits   = DEV_UART1_CFG_STOP_BITS ,
    .parity      = DEV_UART1_CFG_PARITY    ,
    .bit_order   = DEV_UART1_CFG_BIT_ORDER ,
    .invert      = DEV_UART1_CFG_INVERT    ,
    .bufsz       = DEV_UART1_CFG_BUFSZ     ,
    .flowcontrol = DEV_UART1_CFG_FLOW_CTRL ,
    .reserved    = 0 ,
};


static uint32_t _convert_serial_conf(uint32_t conv_type, uint32_t val) {
    uint32_t conv_val = 0;
    bool is_unsupport = false;
    char * tips = RT_NULL;

    switch(conv_type) {
        case _CONV_DATABITS : {
            switch(val) {
                case DATA_BITS_5:   conv_val = UART_DATABITS_5; break;
                case DATA_BITS_6:   conv_val = UART_DATABITS_6; break;
                case DATA_BITS_7:   conv_val = UART_DATABITS_7; break;
                case DATA_BITS_8:   conv_val = UART_DATABITS_8; break;
                default:
                {
                    conv_val     = UART_DATABITS_8; /* give a default value */
                    is_unsupport = true;
                    tips         = "DATA_BITS";
                }
                break;
            }
        }
        break;

        case _CONV_STOPBITS : {
            switch(val) {
                case STOP_BITS_1:   conv_val = UART_STOPBITS_1; break;
                case STOP_BITS_2:   conv_val = UART_STOPBITS_2; break;
                default:
                {
                    conv_val     = UART_STOPBITS_1; /* give a default value */
                    is_unsupport = true;
                    tips         = "STOP_BITS";
                }
                break;
            }
        }
        break;

        case _CONV_PARITY : {
            switch(val) {
                case PARITY_NONE:   conv_val = UART_PARITY_NONE; break;
                case PARITY_ODD :   conv_val = UART_PARITY_ODD ; break;
                case PARITY_EVEN:   conv_val = UART_PARITY_EVEN; break;
                default:
                {
                    conv_val     = UART_PARITY_NONE; /* give a default value */
                    is_unsupport = true;
                    tips         = "PARITY";
                }
                break;
            }
        }
        break;

        case _CONV_FLOW_CTRL : {
            switch(val) {
                case RT_SERIAL_FLOWCONTROL_NONE   : conv_val = UART_HWCONTROL_NONE    ; break;
                case RT_SERIAL_FLOWCONTROL_CTSRTS : conv_val = UART_HWCONTROL_RTS_CTS ; break;
                default:
                {
                    conv_val     = UART_HWCONTROL_NONE; /* give a default value */
                    is_unsupport = true;
                    tips         = "FLOW_CTRL";
                }
                break;
            }
        }
        break;

        default : {
            conv_val     = val;
            is_unsupport = true;
            tips         = "UNKWOWN";
        }
        break;
    }

    if(is_unsupport) {
        rt_kprintf("uart conv %s err, unsupport val: %d \r\n", tips, val);
    }

    return conv_val;
}

#if DEV_UART_0_ENABLE
static void _uart_0_interrupt_callback(app_uart_evt_t *p_evt) {

    uint16_t ret = APP_DRV_SUCCESS;
    struct uart_udata_t * p_udata = &_gr533x_uart0_udata;

    switch(p_evt->type) {
        case APP_UART_EVT_TX_CPLT:
        {
            p_udata->xfer_err  = false;
            rt_sem_release(&(p_udata->tx_semaphore));
        }
        break;

        case APP_UART_EVT_RX_DATA:
        {
            p_udata->recv_size = p_evt->data.size;
            p_udata->recv_pos  = 0;
            p_udata->xfer_err  = false;

            if(p_udata->rx_mode != 0) {
                rt_hw_serial_isr(&p_udata->uart_bus, RT_SERIAL_EVENT_RX_IND);
            }

            if(p_udata->rx_mode == RT_DEVICE_FLAG_INT_RX) {
                ret = app_uart_receive_async((app_uart_id_t)p_udata->id, p_udata->uart_rx_buffer, p_udata->buffer_size);
            } else if(p_udata->rx_mode == RT_DEVICE_FLAG_DMA_RX) {
                ret = app_uart_dma_receive_async((app_uart_id_t)p_udata->id, p_udata->uart_rx_buffer, p_udata->buffer_size);
            }

            if(ret != APP_DRV_SUCCESS) {
                rt_kprintf("uart rcv err: %d \n", ret);
            }
        }
        break;

        case APP_UART_EVT_ABORT_TX:
        {
            p_udata->xfer_err  = true;
            rt_sem_release(&(p_udata->tx_semaphore));
        }
        break;

        case APP_UART_EVT_ABORT_RX:
        {
            p_udata->recv_size = 0;
            p_udata->recv_pos  = 0;
            p_udata->xfer_err  = true;
        }
        break;

        default:
        {
            p_udata->recv_size = 0;
            p_udata->recv_pos  = 0;
            p_udata->xfer_err  = true;

            rt_sem_release(&(p_udata->tx_semaphore));

            if(p_udata->rx_mode == RT_DEVICE_FLAG_INT_RX) {
                ret = app_uart_receive_async((app_uart_id_t)p_udata->id, p_udata->uart_rx_buffer, p_udata->buffer_size);
            } else if(p_udata->rx_mode == RT_DEVICE_FLAG_DMA_RX) {
                ret = app_uart_dma_receive_async((app_uart_id_t)p_udata->id, p_udata->uart_rx_buffer, p_udata->buffer_size);
            }

            if(ret != APP_DRV_SUCCESS) {
                rt_kprintf("uart recv err: %d \n", ret);
            }
        }
        break;
    }
}
#endif

#if DEV_UART_1_ENABLE
static void _uart_1_interrupt_callback(app_uart_evt_t *p_evt) {

    uint16_t ret = APP_DRV_SUCCESS;
    struct uart_udata_t * p_udata = &_gr533x_uart1_udata;

    switch(p_evt->type) {
        case APP_UART_EVT_TX_CPLT:
        {
            p_udata->xfer_err  = false;
            rt_sem_release(&(p_udata->tx_semaphore));
        }
        break;

        case APP_UART_EVT_RX_DATA:
        {
            p_udata->recv_size = p_evt->data.size;
            p_udata->recv_pos  = 0;
            p_udata->xfer_err  = false;

            if(p_udata->rx_mode != 0) {
                rt_hw_serial_isr(&p_udata->uart_bus, RT_SERIAL_EVENT_RX_IND);
            }

            if(p_udata->rx_mode == RT_DEVICE_FLAG_INT_RX) {
                ret = app_uart_receive_async((app_uart_id_t)p_udata->id, p_udata->uart_rx_buffer, p_udata->buffer_size);
            } else if(p_udata->rx_mode == RT_DEVICE_FLAG_DMA_RX) {
                ret = app_uart_dma_receive_async((app_uart_id_t)p_udata->id, p_udata->uart_rx_buffer, p_udata->buffer_size);
            }

            if(ret != APP_DRV_SUCCESS) {
                rt_kprintf("uart rcv err: %d \n", ret);
            }
        }
        break;

        case APP_UART_EVT_ABORT_TX:
        {
            p_udata->xfer_err  = true;
            rt_sem_release(&(p_udata->tx_semaphore));
        }
        break;

        case APP_UART_EVT_ABORT_RX:
        {
            p_udata->recv_size = 0;
            p_udata->recv_pos  = 0;
            p_udata->xfer_err  = true;
        }
        break;

        default:
        {
            p_udata->recv_size = 0;
            p_udata->recv_pos  = 0;
            p_udata->xfer_err  = true;

            rt_sem_release(&(p_udata->tx_semaphore));

            if(p_udata->rx_mode == RT_DEVICE_FLAG_INT_RX) {
                ret = app_uart_receive_async((app_uart_id_t)p_udata->id, p_udata->uart_rx_buffer, p_udata->buffer_size);
            } else if(p_udata->rx_mode == RT_DEVICE_FLAG_DMA_RX) {
                ret = app_uart_dma_receive_async((app_uart_id_t)p_udata->id, p_udata->uart_rx_buffer, p_udata->buffer_size);
            }

            if(ret != APP_DRV_SUCCESS) {
                rt_kprintf("uart recv err: %d \n", ret);
            }
        }
        break;
    }
}
#endif


static rt_err_t _configure(struct rt_serial_device *serial, struct serial_configure *cfg) {

    bool is_init      = false;
    bool is_init_dma  = false;
    uint16_t ret      = 0;
    uint16_t buf_size = DEV_UART_BUF_SIZE;
    app_uart_params_t * p_uart_init  = RT_NULL;
    app_uart_tx_buf_t _uart_buffer;
    app_uart_evt_handler_t p_handler = RT_NULL;

    app_uart_id_t id = (app_uart_id_t)GET_ID(serial);

#if DEV_UART_0_ENABLE
    if(id == APP_UART_ID_0) {

        _gr533x_uart0_udata.tx_mode     = 0;
        _gr533x_uart0_udata.rx_mode     = 0;
        _gr533x_uart0_udata.buffer_size = 0;
        _gr533x_uart0_udata.recv_size   = 0;
        _gr533x_uart0_udata.recv_pos    = 0;
        _gr533x_uart0_udata.xfer_err    = false;

        _gr533x_uart0_udata.uart_init.id                   = id;
        _gr533x_uart0_udata.uart_init.init.baud_rate       = cfg->baud_rate;
        _gr533x_uart0_udata.uart_init.init.data_bits       = _convert_serial_conf(_CONV_DATABITS, cfg->data_bits);
        _gr533x_uart0_udata.uart_init.init.stop_bits       = _convert_serial_conf(_CONV_STOPBITS, cfg->stop_bits);
        _gr533x_uart0_udata.uart_init.init.parity          = _convert_serial_conf(_CONV_PARITY,   cfg->parity);
        _gr533x_uart0_udata.uart_init.init.hw_flow_ctrl    = _convert_serial_conf(_CONV_FLOW_CTRL,cfg->flowcontrol);
        _gr533x_uart0_udata.uart_init.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;

        _gr533x_uart0_udata.uart_init.pin_cfg.rx.type      = DEV_UART0_RX_IO_TYPE;
        _gr533x_uart0_udata.uart_init.pin_cfg.rx.pin       = DEV_UART0_RX_PIN;
        _gr533x_uart0_udata.uart_init.pin_cfg.rx.mux       = DEV_UART0_RX_PINMUX;
        _gr533x_uart0_udata.uart_init.pin_cfg.rx.pull      = DEV_UART0_RX_PULL;

        _gr533x_uart0_udata.uart_init.pin_cfg.tx.type      = DEV_UART0_TX_IO_TYPE;
        _gr533x_uart0_udata.uart_init.pin_cfg.tx.pin       = DEV_UART0_TX_PIN;
        _gr533x_uart0_udata.uart_init.pin_cfg.tx.mux       = DEV_UART0_TX_PINMUX;
        _gr533x_uart0_udata.uart_init.pin_cfg.tx.pull      = DEV_UART0_TX_PULL;

#if DEV_UART0_XFER_MODE == UART_XFER_MODE_DMA

        _gr533x_uart0_udata.is_dma_tx_inited                  = true;
        _gr533x_uart0_udata.is_dma_rx_inited                  = true;
        _gr533x_uart0_udata.uart_init.dma_cfg.tx_dma_instance = DMA0;
        _gr533x_uart0_udata.uart_init.dma_cfg.rx_dma_instance = DMA0;
        _gr533x_uart0_udata.uart_init.dma_cfg.tx_dma_channel  = DEV_UART0_TX_DMA_CHANNEL;
        _gr533x_uart0_udata.uart_init.dma_cfg.rx_dma_channel  = DEV_UART0_RX_DMA_CHANNEL;

        if(_gr533x_uart0_udata.uart_init.dma_cfg.tx_dma_channel  == DMA_Channel_DISABLE) {
            _gr533x_uart0_udata.uart_init.dma_cfg.tx_dma_instance = RT_NULL;
            _gr533x_uart0_udata.is_dma_tx_inited                  = false;
        }
        if(_gr533x_uart0_udata.uart_init.dma_cfg.rx_dma_channel  == DMA_Channel_DISABLE) {
            _gr533x_uart0_udata.uart_init.dma_cfg.rx_dma_instance = RT_NULL;
            _gr533x_uart0_udata.is_dma_rx_inited                  = false;
        }

        if(_gr533x_uart0_udata.is_dma_tx_inited ||
           _gr533x_uart0_udata.is_dma_rx_inited) {
            is_init_dma = true;
        }

#endif
        if(_gr533x_uart0_udata.uart_tx_buffer) {
            rt_free(_gr533x_uart0_udata.uart_tx_buffer);
            _gr533x_uart0_udata.uart_tx_buffer = RT_NULL;
        }
        if(_gr533x_uart0_udata.uart_rx_buffer) {
            rt_free(_gr533x_uart0_udata.uart_rx_buffer);
            _gr533x_uart0_udata.uart_rx_buffer = RT_NULL;
        }

        _gr533x_uart0_udata.uart_tx_buffer = rt_malloc(buf_size + 4); //extra 4 for safe boundry
        _gr533x_uart0_udata.uart_rx_buffer = rt_malloc(buf_size + 4);
        RT_ASSERT(_gr533x_uart0_udata.uart_tx_buffer != RT_NULL);
        RT_ASSERT(_gr533x_uart0_udata.uart_rx_buffer != RT_NULL);
        _gr533x_uart0_udata.buffer_size    = buf_size;

        _uart_buffer.tx_buf      = _gr533x_uart0_udata.uart_tx_buffer;
        _uart_buffer.tx_buf_size = buf_size;
        p_handler                = _uart_0_interrupt_callback;
        p_uart_init              = &(_gr533x_uart0_udata.uart_init);
        is_init                  = true;
    }
#endif

#if DEV_UART_1_ENABLE
    if(id == APP_UART_ID_1) {

        _gr533x_uart1_udata.tx_mode     = 0;
        _gr533x_uart1_udata.rx_mode     = 0;
        _gr533x_uart1_udata.buffer_size = 0;
        _gr533x_uart1_udata.recv_size   = 0;
        _gr533x_uart1_udata.recv_pos    = 0;
        _gr533x_uart1_udata.xfer_err    = false;

        _gr533x_uart1_udata.uart_init.id                   = id;
        _gr533x_uart1_udata.uart_init.init.baud_rate       = cfg->baud_rate;
        _gr533x_uart1_udata.uart_init.init.data_bits       = _convert_serial_conf(_CONV_DATABITS, cfg->data_bits);
        _gr533x_uart1_udata.uart_init.init.stop_bits       = _convert_serial_conf(_CONV_STOPBITS, cfg->stop_bits);
        _gr533x_uart1_udata.uart_init.init.parity          = _convert_serial_conf(_CONV_PARITY,   cfg->parity);
        _gr533x_uart1_udata.uart_init.init.hw_flow_ctrl    = _convert_serial_conf(_CONV_FLOW_CTRL,cfg->flowcontrol);
        _gr533x_uart1_udata.uart_init.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;

        _gr533x_uart1_udata.uart_init.pin_cfg.rx.type      = DEV_UART1_RX_IO_TYPE;
        _gr533x_uart1_udata.uart_init.pin_cfg.rx.pin       = DEV_UART1_RX_PIN;
        _gr533x_uart1_udata.uart_init.pin_cfg.rx.mux       = DEV_UART1_RX_PINMUX;
        _gr533x_uart1_udata.uart_init.pin_cfg.rx.pull      = DEV_UART1_RX_PULL;

        _gr533x_uart1_udata.uart_init.pin_cfg.tx.type      = DEV_UART1_TX_IO_TYPE;
        _gr533x_uart1_udata.uart_init.pin_cfg.tx.pin       = DEV_UART1_TX_PIN;
        _gr533x_uart1_udata.uart_init.pin_cfg.tx.mux       = DEV_UART1_TX_PINMUX;
        _gr533x_uart1_udata.uart_init.pin_cfg.tx.pull      = DEV_UART1_TX_PULL;

#if DEV_UART1_XFER_MODE == UART_XFER_MODE_DMA

        _gr533x_uart1_udata.is_dma_tx_inited                  = true;
        _gr533x_uart1_udata.is_dma_rx_inited                  = true;
        _gr533x_uart1_udata.uart_init.dma_cfg.tx_dma_instance = DMA0;
        _gr533x_uart1_udata.uart_init.dma_cfg.rx_dma_instance = DMA0;
        _gr533x_uart1_udata.uart_init.dma_cfg.tx_dma_channel  = DEV_UART1_TX_DMA_CHANNEL;
        _gr533x_uart1_udata.uart_init.dma_cfg.rx_dma_channel  = DEV_UART1_RX_DMA_CHANNEL;

        if(_gr533x_uart1_udata.uart_init.dma_cfg.tx_dma_channel  == DMA_Channel_DISABLE) {
            _gr533x_uart1_udata.uart_init.dma_cfg.tx_dma_instance = RT_NULL;
            _gr533x_uart1_udata.is_dma_tx_inited                  = false;
        }
        if(_gr533x_uart1_udata.uart_init.dma_cfg.rx_dma_channel  == DMA_Channel_DISABLE) {
            _gr533x_uart1_udata.uart_init.dma_cfg.rx_dma_instance = RT_NULL;
            _gr533x_uart1_udata.is_dma_rx_inited                  = false;
        }

        if(_gr533x_uart1_udata.is_dma_tx_inited ||
           _gr533x_uart1_udata.is_dma_rx_inited) {
            is_init_dma = true;
        }

#endif

        if(_gr533x_uart1_udata.uart_tx_buffer) {
            rt_free(_gr533x_uart1_udata.uart_tx_buffer);
            _gr533x_uart1_udata.uart_tx_buffer = RT_NULL;
        }
        if(_gr533x_uart1_udata.uart_rx_buffer) {
            rt_free(_gr533x_uart1_udata.uart_rx_buffer);
            _gr533x_uart1_udata.uart_rx_buffer = RT_NULL;
        }

        _gr533x_uart1_udata.uart_tx_buffer = rt_malloc(buf_size + 4); //extra 4 for safe boundry
        _gr533x_uart1_udata.uart_rx_buffer = rt_malloc(buf_size + 4);
        RT_ASSERT(_gr533x_uart1_udata.uart_tx_buffer != RT_NULL);
        RT_ASSERT(_gr533x_uart1_udata.uart_rx_buffer != RT_NULL);
        _gr533x_uart1_udata.buffer_size    = buf_size;

        _uart_buffer.tx_buf      = _gr533x_uart1_udata.uart_tx_buffer;
        _uart_buffer.tx_buf_size = buf_size;
        p_handler                = _uart_1_interrupt_callback;
        p_uart_init              = &(_gr533x_uart1_udata.uart_init);
        is_init                  = true;
    }
#endif


    if(is_init) {
        ret = app_uart_init(p_uart_init, p_handler, &_uart_buffer);
        if(ret != 0) {
            return RT_ERROR;
        }

        if(is_init_dma) {
            ret = app_uart_dma_init(p_uart_init);

            if(ret != 0) {
                return RT_ERROR;
            }
        }
    }

    return RT_EOK;
}


static rt_err_t _control(struct rt_serial_device *serial, int cmd, void *arg) {

    rt_err_t ret = RT_ERROR;

    switch(cmd) {
        case RT_DEVICE_CTRL_CONFIG:
        {
            struct serial_configure *cfg = arg;
            if(arg != RT_NULL) {
                ret = _configure(serial, cfg);
            }
        }
        break;

        case RT_DEVICE_CTRL_SET_INT:
        {
            struct uart_udata_t * p_udata = (struct uart_udata_t*)serial->parent.user_data;

            uint32_t flag = (uint32_t) arg;

            if(flag & RT_DEVICE_FLAG_INT_TX) {
                p_udata->tx_mode = RT_DEVICE_FLAG_INT_TX;
            } else if (flag & RT_DEVICE_FLAG_DMA_TX) {
                if(p_udata->is_dma_tx_inited) {
                    p_udata->tx_mode = RT_DEVICE_FLAG_DMA_TX;
                }
            }

            if(flag & RT_DEVICE_FLAG_INT_RX) {
                p_udata->rx_mode = RT_DEVICE_FLAG_INT_RX;
                ret = app_uart_receive_async((app_uart_id_t)p_udata->id, p_udata->uart_rx_buffer, p_udata->buffer_size);

            } else if (flag & RT_DEVICE_FLAG_DMA_RX) {
                if(p_udata->is_dma_rx_inited) {
                    p_udata->rx_mode = RT_DEVICE_FLAG_DMA_RX;
                    ret = app_uart_dma_receive_async((app_uart_id_t)p_udata->id, p_udata->uart_rx_buffer, p_udata->buffer_size);
                }
            }

            ret = RT_EOK;
        }
        break;

        case RT_DEVICE_CTRL_CLR_INT:
        {
            struct uart_udata_t * p_udata = (struct uart_udata_t*)serial->parent.user_data;

            uint32_t flag = (uint32_t) arg;

            if((flag & RT_DEVICE_FLAG_INT_RX) || (flag & RT_DEVICE_FLAG_DMA_RX)) {
                p_udata->rx_mode        = 0;
            }

            ret = RT_EOK;
        }
        break;

        case RT_DEVICE_CTRL_CLOSE:
        {
            struct uart_udata_t * p_udata = (struct uart_udata_t*)serial->parent.user_data;

            app_uart_id_t id = (app_uart_id_t)p_udata->id;

            if(p_udata->is_dma_tx_inited || p_udata->is_dma_rx_inited) {
                app_uart_dma_deinit(id);
                p_udata->is_dma_tx_inited = false;
                p_udata->is_dma_rx_inited = false;
            }

            app_uart_deinit(id);

            if(p_udata->uart_tx_buffer) {
                rt_free(p_udata->uart_tx_buffer);
                p_udata->uart_tx_buffer = RT_NULL;
            }
            if(p_udata->uart_rx_buffer) {
                rt_free(p_udata->uart_rx_buffer);
                p_udata->uart_rx_buffer = RT_NULL;
            }

            p_udata->tx_mode     = 0;
            p_udata->rx_mode     = 0;
            p_udata->buffer_size = 0;
            p_udata->recv_size   = 0;
            p_udata->recv_pos    = 0;
            p_udata->xfer_err    = false;
            ret = RT_EOK;
        }
        break;
    }

    return ret;
}


static int _putc(struct rt_serial_device *serial, char c) {
    uint16_t ret = 0;
    app_uart_id_t id = (app_uart_id_t)GET_ID(serial);

    struct uart_udata_t * p_udata = (struct uart_udata_t*)serial->parent.user_data;

    if(p_udata->tx_mode == RT_DEVICE_FLAG_DMA_TX) {

    } else if(p_udata->tx_mode == RT_DEVICE_FLAG_INT_TX) {

        p_udata->xfer_err = false;
        ret = app_uart_transmit_async(id, (uint8_t *)&c, 1);

        if(0 == ret) {
            rt_sem_take(&(p_udata->tx_semaphore), RT_WAITING_FOREVER);
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DONE);
            if(p_udata->xfer_err) {
                ret = 0xFF;     //error flag
            }
        }
    } else {
        ret = app_uart_transmit_sync(id, (uint8_t *)&c, 1, 300);
    }

    return (ret == 0) ? 0 : -1;
}


static int _getc(struct rt_serial_device *serial) {
    char c           = 0;
    uint16_t ret     = 0;
    app_uart_id_t id = (app_uart_id_t)GET_ID(serial);

    struct uart_udata_t * p_udata = (struct uart_udata_t*)serial->parent.user_data;

    if((p_udata->rx_mode == RT_DEVICE_FLAG_DMA_RX) ||
       (p_udata->rx_mode == RT_DEVICE_FLAG_INT_RX)) {
        if(p_udata->recv_pos < p_udata->recv_size) {
            c = p_udata->uart_rx_buffer[p_udata->recv_pos];
            p_udata->recv_pos++;
            ret = 0;
        } else {
            ret = 0xff; /* error flag */
        }

    } else {
        ret = app_uart_receive_sync(id, (uint8_t *)&c, 1, 1000);
    }

    return (ret == 0) ? c : -1;
}


static rt_size_t _dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction) {

    uint16_t ret = 0;
    app_uart_id_t id = (app_uart_id_t)GET_ID(serial);
    struct uart_udata_t * p_udata = (struct uart_udata_t *) serial->parent.user_data;

    if(size > 4095u) {
        rt_kprintf("DMA xfer Size is big for uart%d", id);
        return 0;
    }

    p_udata->xfer_err = false;
    ret = app_uart_dma_transmit_async(id, buf, size);

    if(ret != 0) {
        return 0;
    }

    rt_sem_take(&(p_udata->tx_semaphore), RT_WAITING_FOREVER);
    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE);

    if(p_udata->xfer_err) {
        return 0;
    }

    return size;
}


int rt_hw_uart_init(void)
{
    int result = 0;

#if DEV_UART_0_ENABLE
    _gr533x_uart0_udata.uart_bus.ops    = &_gr533x_uart_ops;
    _gr533x_uart0_udata.uart_bus.config = _gr533x_uart0_conf;
    result   = rt_hw_serial_register(&_gr533x_uart0_udata.uart_bus, DEV_UART_0, DEV_UART0_FLAG, (void*) &_gr533x_uart0_udata);
    rt_sem_init(&_gr533x_uart0_udata.tx_semaphore, "uart0_tx", 0, RT_IPC_FLAG_FIFO);
#endif

#if DEV_UART_1_ENABLE
    _gr533x_uart1_udata.uart_bus.ops    = &_gr533x_uart_ops;
    _gr533x_uart1_udata.uart_bus.config = _gr533x_uart1_conf;
    result  |= rt_hw_serial_register(&_gr533x_uart1_udata.uart_bus, DEV_UART_1, DEV_UART1_FLAG, (void*) &_gr533x_uart1_udata);
    rt_sem_init(&_gr533x_uart1_udata.tx_semaphore, "uart1_tx", 0, RT_IPC_FLAG_FIFO);
#endif

    return result;
}
INIT_BOARD_EXPORT(rt_hw_uart_init);

#endif /* RT_USING_SERIAL */
