
/***************************************************************************************
 *                 UART 功能测试实验
 *
 * 1. 默认UART0 配置为Console, 作日志打印; UART1配置为互联通信口, 可以通过另一个串口转接到电脑, 用于交互测试
 * 2. 程序开始运行, 日志通过 UART0 打印
 * 3. UART1 可以进行回环测试, 对端发过来的字符, 向后偏移1后再发给对端
 * 4. 可以尝试配置 POLL、IRQ、DMA等多种模式, 以及尝试修改 UART 参数进行各种测试
 *
 ***************************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include "string.h"
#include "drivers/serial.h"


#define SAMPLE_UART_NAME       "uart1"


/* 用于接收消息的信号量 */
static struct rt_semaphore  rx_sem;
static rt_device_t          serial;


/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

static rt_err_t uart_ouput(rt_device_t dev, void *buffer)
{
    rt_kprintf("send done!");

    return RT_EOK;
}

static void serial_thread_entry(void *parameter)
{
    char ch;

    while (1)
    {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(serial, -1, &ch, 1) != 1)
        {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        /* 读取到的数据通过串口错位输出 */
        ch = ch + 1;
        rt_device_write(serial, 0, &ch, 1);
    }
}

int serial_test_entry(void)
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    char str[] = "hello RT-Thread!\r\n";

    rt_strncpy(uart_name, SAMPLE_UART_NAME, RT_NAME_MAX);

    /* 查找系统中的串口设备 */
    serial = rt_device_find(uart_name);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    /* 初始化信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(serial, 0);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);
    /* 发送字符串 */
    rt_device_write(serial, 0, str, (sizeof(str) - 1));


    rt_device_close(serial);


    rt_device_open(serial, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);

    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);

    rt_device_set_tx_complete(serial, uart_ouput);

    rt_device_write(serial, 0, "Hello world", (sizeof("Hello world") - 1));


    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }

    return ret;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(uart_sample, uart device sample);

