
/***************************************************************************************
 *                 SPI 功能测试实验
 *
 * 1. 使用逻辑分析器连接 SPI 的引脚
 * 2. 分别配置 SPI 工作在POLL、IRQ、DMA 模式, 运行下面测试函数
 * 3. 通过逻辑分析器 解析时序正确性
 * 4. 可以尝试修改频率、数据宽度等配置, 重复测试各种可能性配置
 *
 ***************************************************************************************/

#include <rtdevice.h>
#include <rthw.h>
#include <string.h>
#include <stdint.h>
#include  "drivers/spi.h"

#define SPI_BUS_NAME        "spi0"
#define SPI_DEVICE_NAME     "spi00"


uint8_t buff1[8000]  = {0x01, 0x02, 0x03, 0x04};
uint8_t buff2[16000] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,0x0C,0x0D,0x0E,0x0F};


int spi_test_entry(void) {

    struct rt_spi_device * spi_device = RT_NULL;

    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));

    if(RT_NULL == spi_device)
    {
        rt_kprintf("Failed to malloc the spi device.\r\n");
        return -RT_ENOMEM;
    }

    if (RT_EOK != rt_spi_bus_attach_device(spi_device, SPI_DEVICE_NAME, SPI_BUS_NAME, RT_NULL))
    {
        rt_kprintf("Failed to attach the spi device.");
        return -RT_ERROR;
    }

    struct rt_spi_configuration cfg;
    cfg.data_width = 16;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.max_hz = 16 * 1000 *1000;                           /* 8M */

    if (RT_EOK != rt_spi_configure(spi_device, &cfg)) {
        rt_kprintf("Failed to Configure the spi device.");
        return -RT_ERROR;
    }

    struct rt_spi_message msg1, msg2;

    msg1.send_buf   = &buff1;
    msg1.recv_buf   = RT_NULL;
    msg1.length     = 4000;
    msg1.cs_take    = 1;
    msg1.cs_release = 0;
    msg1.next       = &msg2;

    msg2.send_buf   = &buff2;
    msg2.recv_buf   = RT_NULL;
    msg2.length     = 8000;
    msg2.cs_take    = 0;
    msg2.cs_release = 1;
    msg2.next       = RT_NULL;

    while(1) {

        rt_spi_transfer_message(spi_device, &msg1);

        rt_thread_delay(1000);
        rt_kprintf(".");
    }

    //return RT_EOK;
}

