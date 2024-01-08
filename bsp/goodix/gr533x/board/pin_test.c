
/***************************************************************************************
 *                 PIN 功能测试实验
 *
 * 1. PIN 35/36/45 配置为输出模式, 周期性拉高拉低
 * 2. GR533x SK 上, PIN 35/36 连接LED, 会看到LED 周期性闪烁
 * 3. PIN.34 配置为输入中断模式, 触发方式为上升沿
 * 4. 将 PIN.45和PIN.34 短接, 这样PIN.45 翻转, 就能周期性触发 PIN.34 的中断函数, 打印 '+'
 *
 ***************************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include "string.h"
#include "drivers/pin.h"



static void pin_irq_on(void * args) {
    rt_kprintf("+");
}


int pin_test_entry(void)
{
    rt_err_t ret = RT_EOK;

    rt_pin_mode(45, PIN_MODE_OUTPUT);
    rt_pin_mode(35, PIN_MODE_OUTPUT);
    rt_pin_mode(36, PIN_MODE_OUTPUT);

    /*设置 pin.34 输入中断, 上边沿检测, 中断响应为打印字符+ */
    rt_pin_mode(34, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(34, PIN_IRQ_MODE_RISING, pin_irq_on, RT_NULL);
    rt_pin_irq_enable(34, PIN_IRQ_ENABLE);

    rt_uint8_t cnt = 10;

    while(cnt --)
    {

        rt_pin_write(45, PIN_HIGH);
        rt_pin_write(35, PIN_HIGH);
        rt_pin_write(36, PIN_HIGH);

        rt_thread_delay(500);

        rt_pin_write(45, PIN_LOW);
        rt_pin_write(35, PIN_LOW);
        rt_pin_write(36, PIN_LOW);

        rt_thread_delay(500);
    }

    return ret;
}

