#include <rtdevice.h>
#include <rthw.h>
#include <string.h>

#ifdef RT_USING_PIN

#include "drv_porting_pin.h"
#include "app_io.h"


#define SIZEOF(items) sizeof(items) / sizeof(items[0])

static void         _pin_mode(struct rt_device *device, rt_base_t pin, rt_base_t mode);
static void         _pin_write(struct rt_device *device, rt_base_t pin, rt_base_t value);
static int          _pin_read(struct rt_device *device, rt_base_t pin);
static rt_err_t     _pin_attach_irq(struct rt_device *device, rt_int32_t pin, rt_uint32_t mode, void (*hdr)(void *args), void *args);
static rt_err_t     _pin_detach_irq(struct rt_device *device, rt_int32_t pin);
static rt_err_t     _pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint32_t enabled);
static rt_base_t    _pin_get(const char *name);
static void         _pin_irq_callback(app_io_evt_t *p_evt);
static const pin_index_t * _get_pin_index(rt_base_t pin);


static const struct rt_pin_ops _gr533x_pin_ops =
{
    .pin_mode       = _pin_mode       ,
    .pin_write      = _pin_write      ,
    .pin_read       = _pin_read       ,
    .pin_attach_irq = _pin_attach_irq ,
    .pin_detach_irq = _pin_detach_irq ,
    .pin_irq_enable = _pin_irq_enable ,
    .pin_get        = _pin_get        ,
};

static const pin_index_t _pin_index[] = {
#if GR533x_PACKAGE_TYPE == 0        /* QFN32 */

    GR533x_PIN_DEFAULT,             /* Reserved 0,  */
    GR533x_PIN_DEFAULT,             /* PIN 1 */
    GR533x_PIN_DEFAULT,             /* PIN 2 */
    GR533x_PIN_DEFAULT,             /* PIN ... */
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    {8, APP_IO_TYPE_GPIOA, APP_IO_PIN_0},
    {9, APP_IO_TYPE_GPIOA, APP_IO_PIN_1},
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    {16, APP_IO_TYPE_MSIO, APP_IO_PIN_7},
    {17, APP_IO_TYPE_MSIO, APP_IO_PIN_6},
    {18, APP_IO_TYPE_MSIO, APP_IO_PIN_5},
    {19, APP_IO_TYPE_MSIO, APP_IO_PIN_4},
    {20, APP_IO_TYPE_MSIO, APP_IO_PIN_3},
    {21, APP_IO_TYPE_MSIO, APP_IO_PIN_9},
    {22, APP_IO_TYPE_MSIO, APP_IO_PIN_8},
    {23, APP_IO_TYPE_AON,  APP_IO_PIN_0},
    {24, APP_IO_TYPE_AON,  APP_IO_PIN_1},
    GR533x_PIN_DEFAULT,
    {26, APP_IO_TYPE_AON,  APP_IO_PIN_2},
    {27, APP_IO_TYPE_AON,  APP_IO_PIN_3},
    {28, APP_IO_TYPE_AON,  APP_IO_PIN_4},
    {29, APP_IO_TYPE_AON,  APP_IO_PIN_5},
    {30, APP_IO_TYPE_AON,  APP_IO_PIN_6},
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,

#elif GR533x_PACKAGE_TYPE == 1      /* QFN48 */

    GR533x_PIN_DEFAULT,             /* Reserved 0,  */
    GR533x_PIN_DEFAULT,             /* PIN 1 */
    GR533x_PIN_DEFAULT,             /* PIN 2 */
    GR533x_PIN_DEFAULT,             /* PIN ... */
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    {6,  APP_IO_TYPE_GPIOA, APP_IO_PIN_0},
    {7,  APP_IO_TYPE_GPIOA, APP_IO_PIN_1},
    {8,  APP_IO_TYPE_GPIOA, APP_IO_PIN_2},
    {9,  APP_IO_TYPE_GPIOA, APP_IO_PIN_3},
    {10, APP_IO_TYPE_GPIOA, APP_IO_PIN_4},
    {11, APP_IO_TYPE_GPIOA, APP_IO_PIN_5},
    {12, APP_IO_TYPE_GPIOA, APP_IO_PIN_6},
    {13, APP_IO_TYPE_GPIOA, APP_IO_PIN_7},
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    {21, APP_IO_TYPE_MSIO, APP_IO_PIN_7},
    {22, APP_IO_TYPE_MSIO, APP_IO_PIN_6},
    {23, APP_IO_TYPE_MSIO, APP_IO_PIN_5},
    {24, APP_IO_TYPE_MSIO, APP_IO_PIN_4},
    {25, APP_IO_TYPE_MSIO, APP_IO_PIN_3},
    {26, APP_IO_TYPE_MSIO, APP_IO_PIN_2},
    {27, APP_IO_TYPE_MSIO, APP_IO_PIN_1},
    {28, APP_IO_TYPE_MSIO, APP_IO_PIN_0},
    {29, APP_IO_TYPE_MSIO, APP_IO_PIN_9},
    {30, APP_IO_TYPE_MSIO, APP_IO_PIN_8},
    {31, APP_IO_TYPE_AON,  APP_IO_PIN_0},
    {32, APP_IO_TYPE_AON,  APP_IO_PIN_1},
    GR533x_PIN_DEFAULT,
    {34, APP_IO_TYPE_AON,  APP_IO_PIN_2},
    {35, APP_IO_TYPE_AON,  APP_IO_PIN_3},
    {36, APP_IO_TYPE_AON,  APP_IO_PIN_4},
    {37, APP_IO_TYPE_AON,  APP_IO_PIN_5},
    {38, APP_IO_TYPE_AON,  APP_IO_PIN_6},
    {39, APP_IO_TYPE_AON,  APP_IO_PIN_7},
    {40, APP_IO_TYPE_GPIOA, APP_IO_PIN_8},
    {41, APP_IO_TYPE_GPIOA, APP_IO_PIN_9},
    {42, APP_IO_TYPE_GPIOA, APP_IO_PIN_10},
    {43, APP_IO_TYPE_GPIOA, APP_IO_PIN_11},
    {44, APP_IO_TYPE_GPIOA, APP_IO_PIN_12},
    {45, APP_IO_TYPE_GPIOA, APP_IO_PIN_13},
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
    GR533x_PIN_DEFAULT,
#endif
};


const static uint8_t _gpioa_pin_order[] = {
#if GR533x_PACKAGE_TYPE == 0        /* QFN32 */
    8, 9,
#elif GR533x_PACKAGE_TYPE == 1      /* QFN48 */
    6,7,8,9,10,11,12,13,
    40,41,42,43,44,45
#endif
};

const static uint8_t _msio_pin_order[] = {
#if GR533x_PACKAGE_TYPE == 0        /* QFN32 */
    -1, -1, -1,
    20,19,18,17,16,22,21,

#elif GR533x_PACKAGE_TYPE == 1      /* QFN48 */
    28,27,26,25,24,23,22,21,30,29,
#endif
};

const static uint8_t _aon_pin_order[] = {
#if GR533x_PACKAGE_TYPE == 0        /* QFN32 */
    23,24,26,27,26,29,30,
#elif GR533x_PACKAGE_TYPE == 1      /* QFN48 */
    31,32,34,35,36,37,38,39,
#endif
};


static pin_irq_map_t _pin_irq_map[SIZEOF(_pin_index)];


const static pin_index_t * _get_pin_index(rt_base_t pin)
{
    const pin_index_t * p_index;

    if (pin < SIZEOF(_pin_index))
    {
        p_index = &_pin_index[pin];

        if (p_index->index == -1) {
            p_index = RT_NULL;
        }
    }
    else
    {
        p_index = RT_NULL;
    }

    return p_index;
};


static void _pin_mode(struct rt_device *device, rt_base_t pin, rt_base_t mode) {

    const pin_index_t * p_index = _get_pin_index(pin);

    if(p_index == RT_NULL) {
        return;
    }

    app_io_type_t io_type = (app_io_type_t) p_index->io_type;
    app_io_init_t io_init;

    switch(mode)
    {
        case PIN_MODE_OUTPUT:
        case PIN_MODE_OUTPUT_OD:
        {
            io_init.mode = APP_IO_MODE_OUTPUT;
            io_init.pin  = p_index->io_pin;
            io_init.pull = APP_IO_NOPULL;
            io_init.mux  = APP_IO_MUX;
        }
        break;

        case PIN_MODE_INPUT:
        {
            io_init.mode = APP_IO_MODE_INPUT;
            io_init.pin  = p_index->io_pin;
            io_init.pull = APP_IO_NOPULL;
            io_init.mux  = APP_IO_MUX;
        }
        break;

        case PIN_MODE_INPUT_PULLUP:
        {
            io_init.mode = APP_IO_MODE_INPUT;
            io_init.pin  = p_index->io_pin;
            io_init.pull = APP_IO_PULLUP;
            io_init.mux  = APP_IO_MUX;
        }
        break;

        case PIN_MODE_INPUT_PULLDOWN:
        {
            io_init.mode = APP_IO_MODE_INPUT;
            io_init.pin  = p_index->io_pin;
            io_init.pull = APP_IO_PULLDOWN;
            io_init.mux  = APP_IO_MUX;
        }
        break;

        default:
        {}
        return;
    }

    _pin_irq_map[p_index->index].pull_mode = io_init.pull;
    app_io_init(io_type, &io_init);

    return;
}


static void _pin_write(struct rt_device *device, rt_base_t pin, rt_base_t value) {
    const pin_index_t * p_index = _get_pin_index(pin);

    if(p_index == RT_NULL) {
        return;
    }

    app_io_type_t io_type = (app_io_type_t) p_index->io_type;
    app_io_write_pin(io_type, (uint32_t) p_index->io_pin, value ? APP_IO_PIN_SET : APP_IO_PIN_RESET);
    return;
}


static int _pin_read(struct rt_device *device, rt_base_t pin) {
    const pin_index_t * p_index = _get_pin_index(pin);

    if(p_index == RT_NULL) {
        return PIN_LOW;
    }
    app_io_pin_state_t io_state;
    app_io_type_t      io_type;

    io_type  = (app_io_type_t) p_index->io_type;
    io_state = app_io_read_pin(io_type, (uint32_t) p_index->io_pin);

    return (io_state == APP_IO_PIN_RESET) ? PIN_LOW : PIN_HIGH;
}


static void _pin_irq_callback(app_io_evt_t *p_evt) {
    int index = 0;
    if(p_evt) {
        for(int  i = 0; i < SIZEOF(_pin_index); i++) {
            if((p_evt->type == _pin_index[i].io_type) && (p_evt->pin == _pin_index[i].io_pin)) {
                index = i;
                if(_pin_irq_map[index].irq_attach &&
                   _pin_irq_map[index].irq_cb &&
                   _pin_irq_map[index].irq_en) {
                    _pin_irq_map[index].irq_cb(p_evt->arg);
                }
                break;
            }
        }
    }

    return;
}


static rt_err_t _pin_attach_irq(struct rt_device *device, rt_int32_t pin, rt_uint32_t mode, void (*hdr)(void *args), void *args) {
    const pin_index_t * p_index = _get_pin_index(pin);

    if(p_index == RT_NULL) {
        return RT_EINVAL;
    }

    if((mode > PIN_IRQ_MODE_LOW_LEVEL) || (hdr == RT_NULL)) {
        return RT_EINVAL;
    }

    if(p_index->io_type == APP_IO_TYPE_MSIO) {
        return RT_EINVAL;
    }

    int32_t index = p_index->index;

    _pin_irq_map[index].irq_mode   = mode;
    _pin_irq_map[index].irq_cb     = hdr;
    _pin_irq_map[index].irq_args   = args;
    _pin_irq_map[index].irq_attach = RT_TRUE;
    _pin_irq_map[index].irq_en     = RT_FALSE;

    return RT_EOK;
}


static rt_err_t _pin_detach_irq(struct rt_device *device, rt_int32_t pin) {
    const pin_index_t * p_index = _get_pin_index(pin);

    if(p_index == RT_NULL) {
        return RT_EINVAL;
    }

    if(p_index->io_type == APP_IO_TYPE_MSIO) {
        return RT_EINVAL;
    }

    _pin_irq_enable(device, pin, RT_FALSE);

    int32_t index = p_index->index;

    _pin_irq_map[index].irq_mode   = 0xFF;
    _pin_irq_map[index].irq_cb     = RT_NULL;
    _pin_irq_map[index].irq_args   = RT_NULL;
    _pin_irq_map[index].irq_attach = RT_FALSE;
    _pin_irq_map[index].irq_en     = RT_FALSE;
    _pin_irq_map[index].pull_mode  = 0;

    return RT_TRUE;
}


static rt_err_t _pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint32_t enabled) {

    const pin_index_t * p_index = _get_pin_index(pin);

    if(p_index == RT_NULL) {
        return RT_EINVAL;
    }

    if(p_index->io_type == APP_IO_TYPE_MSIO) {
        return RT_EINVAL;
    }

    uint16_t ret  = 0;
    int32_t index = p_index->index;
    app_io_type_t io_type = (app_io_type_t)p_index->io_type;
    app_io_init_t io_init;

    if((_pin_irq_map[index].irq_attach == RT_TRUE) &&
       (_pin_irq_map[index].irq_cb     != RT_NULL) &&
       (_pin_irq_map[index].irq_mode   <= PIN_IRQ_MODE_LOW_LEVEL)) {

        if(enabled) {
            if(_pin_irq_map[index].irq_en) {
                return RT_EOK;
            } else {

                io_init.mux  = APP_IO_MUX;
                io_init.pin  = p_index->io_pin;
                io_init.pull = (app_io_pull_t)_pin_irq_map[index].pull_mode;

                switch(_pin_irq_map[index].irq_mode) {
                    case PIN_IRQ_MODE_RISING:           io_init.mode = APP_IO_MODE_IT_RISING;       break;
                    case PIN_IRQ_MODE_FALLING:          io_init.mode = APP_IO_MODE_IT_FALLING;      break;
                    case PIN_IRQ_MODE_RISING_FALLING:   io_init.mode = APP_IO_MODE_IT_BOTH_EDGE;    break;
                    case PIN_IRQ_MODE_HIGH_LEVEL:       io_init.mode = APP_IO_MODE_IT_HIGH;         break;
                    case PIN_IRQ_MODE_LOW_LEVEL:        io_init.mode = APP_IO_MODE_IT_LOW;          break;
                    default:                            io_init.mode = APP_IO_MODE_INPUT;           break;
                }

                ret = app_io_event_register_cb(io_type, &io_init, _pin_irq_callback, _pin_irq_map[index].irq_args);
                if(APP_DRV_SUCCESS != ret) {
                    return RT_ERROR;
                }

                _pin_irq_map[index].irq_en = RT_TRUE;

                return RT_EOK;
            }
        } else {
            if(_pin_irq_map[index].irq_en) {

                app_io_event_unregister(io_type, p_index->io_pin);

                io_init.mux  = APP_IO_MUX;
                io_init.pin  = p_index->io_pin;
                io_init.pull = (app_io_pull_t)_pin_irq_map[index].pull_mode;
                io_init.mode = APP_IO_MODE_INPUT;
                app_io_init(io_type, &io_init);

                _pin_irq_map[index].irq_en = RT_FALSE;

                return RT_EOK;
            } else {
                return RT_EOK;
            }
        }
    }

    return RT_EINVAL;
}


/*
 *      Map between name and IO
 *
 * GPIO0     ~ GPIOx     : PA.0 ~ PA.x
 * MSIO0     ~ MSIOx     : PM.0 ~ PM.x
 * AON_GPIO0 ~ AON_GPIOx : PO.0 ~ PO.x
 *
 */

static rt_base_t _pin_get(const char *name) {

    rt_base_t pin   = 0;
    int hw_pin_num  = 0;
    int name_len    = 0;
    uint8_t buff[4] = {0,0,0,0};

    name_len = rt_strlen(name);

    if ((name_len < 4) || (name_len >= 6))
    {
        return -RT_EINVAL;
    }
    if ((name[0] != 'P') || (name[2] != '.'))
    {
        return -RT_EINVAL;
    }

    memcpy(&buff[0], &name[3], name_len - 3);
    hw_pin_num = atoi((const char *)&buff[0]);

    if(name[1] == 'A') {
        pin = _gpioa_pin_order[hw_pin_num];
    } else if(name[1] == 'M') {
        pin = _msio_pin_order[hw_pin_num];
    } else if(name[1] == 'O') {
        pin = _aon_pin_order[hw_pin_num];
    } else {
        return -RT_EINVAL;
    }

    return pin;
}


int rt_hw_pin_init(void)
{
    int result;

    memset(&_pin_irq_map[0], 0, sizeof(_pin_irq_map));

    result = rt_device_pin_register("pin", &_gr533x_pin_ops, RT_NULL);

    return result;
}
INIT_BOARD_EXPORT(rt_hw_pin_init);


#endif /* RT_USING_PIN */
