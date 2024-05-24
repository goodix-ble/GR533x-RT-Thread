#include <rtdevice.h>
#include <rthw.h>
#include "stdint.h"
#include "dfu_port.h"


#define DFU_FW_SAVE_ADDR       (FLASH_START_ADDR + 0x40000)


static struct rt_semaphore      _dfu_semaphore;
static rt_thread_t              _dfu_thread;
static bool                     b_start_dfu_task_flag = false;


static void dfu_program_start_callback(void);
static void dfu_programing_callback(uint8_t pro);
static void dfu_program_end_callback(uint8_t status);

static dfu_pro_callback_t dfu_pro_call =
{
    .dfu_program_start_callback = dfu_program_start_callback,
    .dfu_programing_callback    = dfu_programing_callback,
    .dfu_program_end_callback   = dfu_program_end_callback,
};

static void dfu_program_start_callback(void)
{
    rt_kprintf("Dfu start program\r\n");
}

static void dfu_programing_callback(uint8_t pro)
{
    rt_kprintf("Dfu programing---%d%%\r\n", pro);
}

static void dfu_program_end_callback(uint8_t status)
{
    rt_kprintf("Dfu program end\r\n");
    if (0x01 == status)
    {
        rt_kprintf("status: successful\r\n");
    }
    else
    {
        rt_kprintf("status: error\r\n");
    }
}


static void dfu_schedule_trigger(void)
{
    if (!b_start_dfu_task_flag)
    {
        b_start_dfu_task_flag = true;
        rt_kprintf("DFU Start Schedule ...\r\n");
        rt_sem_release(&_dfu_semaphore);
        //dfu_timer_start();
    }
}

static void dfu_thread_entry(void * p_arg) {
    while (1)
    {
        if (!b_start_dfu_task_flag)
        {
            rt_sem_take(&_dfu_semaphore, RT_WAITING_FOREVER);
        }
        dfu_schedule();
    }
}

void dfu_ble_service_init(void) {
    extern void dfu_service_init(dfu_enter_callback dfu_enter);

    dfu_service_init(dfu_schedule_trigger);
}

void dfu_task_start(void) {

    dfu_port_init(NULL, DFU_FW_SAVE_ADDR, &dfu_pro_call);
    rt_sem_init(&_dfu_semaphore, "dfu_sema", 0, RT_IPC_FLAG_FIFO);
    _dfu_thread = rt_thread_create( "dfu_task", dfu_thread_entry, NULL, 1024, RT_THREAD_PRIORITY_MAX - 10, 8);
    
    if (_dfu_thread != RT_NULL)
    {
        rt_thread_startup(_dfu_thread);
    }
    else
    {
        rt_kprintf("dfu thread create fail \r\n");
    }
}
