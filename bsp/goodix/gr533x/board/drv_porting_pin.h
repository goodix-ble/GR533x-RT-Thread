
#ifndef __DRV_PORT_PIN_H__
#define __DRV_PORT_PIN_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *      Map between name and IO
 *
 * GPIO0     ~ GPIOx     : PA.0 ~ PA.x
 * MSIO0     ~ MSIOx     : PM.0 ~ PM.x
 * AON_GPIO0 ~ AON_GPIOx : PO.0 ~ PO.x
 *
 */

#define GR533x_PACKAGE_TYPE             1           /* package type for GR533x, 0: QFN32; 1 : DFN48 */

#define GR533x_PIN_DEFAULT              {-1, 0, 0}

/*
 * Pin Index, correspond to Chip PAD Number
 */
typedef struct
{
    rt_int32_t  index;
    rt_uint32_t io_type;
    rt_uint32_t io_pin;
} pin_index_t;


/*
 * I/O Interrupt MAP, correspond to PIN Index Table
 */
typedef struct {

    void            (* irq_cb)(void *args);     /* I/O Interrupt Callback */
    void *          irq_args;                   /* User arguments for IRQ Callback */
    rt_uint8_t      irq_mode;                   /* I/O Interrupt Trigger Mode */
    rt_uint8_t      pull_mode;                  /* Record the I/O Pull Mode */
    rt_uint8_t      irq_attach;                 /* Attachh the I/O Interrupt or Not */
    rt_uint8_t      irq_en;                     /* Enable the I/O Interrupt or Not */
} pin_irq_map_t ;


int rt_hw_pin_init(void);


#ifdef __cplusplus
}
#endif

#endif /* __DRV_PORT_PIN_H__ */

