
/***************************************************************************************
 *                 I2C 功能测试实验
 *
 * 1. 当前测试程序, 是进行一个 g-sensor 的初始化及测试访问
 * 2. 由于测试用户难于找到相同的测试传感器, 故需要根据可以获得的I2C设备,自行修改测试程序进行
 *
 ***************************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include "string.h"

#include "drivers/i2c.h"

#include "gr533x_hal.h"

#define QMC5883_ADDRESS         0x0D
#define QMC5883_REG_CHIP_ID     0x0D
#define QMC5883_REG_X_LSB       0x00

#define I2C0_NAME               "i2c1"


typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} magmeter_rawdata_t;


/* 写传感器寄存器 */
static rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t *data)
{
    rt_uint8_t buf[3];
    struct rt_i2c_msg msgs;
    rt_uint32_t buf_size = 1;

    buf[0] = reg; //cmd
    if (data != RT_NULL)
    {
        buf[1] = data[0];
        //buf[2] = data[1];
        //buf_size = 3;
        buf_size = 2;
    }

    msgs.addr  = QMC5883_ADDRESS;
    msgs.flags = RT_I2C_WR;
    msgs.buf   = buf;
    msgs.len   = buf_size;

    /* 调用I2C设备接口传输数据 */
    rt_i2c_transfer(bus, &msgs, 1);

    return RT_EOK;
}


/* 读传感器寄存器数据 */
static rt_err_t read_regs(struct rt_i2c_bus_device *bus, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs;

    msgs.addr  = QMC5883_ADDRESS;
    msgs.flags = RT_I2C_RD;
    msgs.buf   = buf;
    msgs.len   = len;

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

uint8_t read_byte(struct rt_i2c_bus_device *bus, rt_uint8_t reg) {
    uint8_t val = 0;

    write_reg(bus, reg, RT_NULL);

    read_regs(bus, 1, &val);

    return val;
}

void magmeter_reset(struct rt_i2c_bus_device *bus)
{
    /**
     * Reset then set recommended value
     */
    uint8_t val = 0;

    val = 0x80;
    write_reg(bus, 0x0A, &val);
    val = 0x01;
    write_reg(bus, 0x0B, &val);
    val = 0x40;
    write_reg(bus, 0x20, &val);
    val = 0x01;
    write_reg(bus, 0x21, &val);
    val = 0x00;
    write_reg(bus, 0x09, &val);

}


void magmeter_read(struct rt_i2c_bus_device *bus, magmeter_rawdata_t *out)
{
    uint8_t buffer[6];

    memset(buffer, 0, 6);

    write_reg(bus, QMC5883_REG_X_LSB, RT_NULL);
    read_regs(bus, 6, buffer);

    out->x = *((int16_t *)(&buffer[0]));
    out->y = *((int16_t *)(&buffer[2]));
    out->z = *((int16_t *)(&buffer[4]));
}


static void qmc5883_selftest(struct rt_i2c_bus_device *bus)

{
    uint8_t val = 0;

    val = 0x40;
    write_reg(bus, 0x20, &val);

    val = 0x01;
    write_reg(bus, 0x21, &val);

    val = 0x01;
    write_reg(bus, 0x0B, &val);

    val = 0x1D;
    write_reg(bus, 0x09, &val);

    rt_thread_mdelay(10);

    val = 0x1C;
    write_reg(bus, 0x09, &val);

    val = 0x00;
    write_reg(bus, 0x0B, &val);

    val = 0x12;
    write_reg(bus, 0x09, &val);

    do
    {
        rt_thread_mdelay(10);
        rt_kprintf("+");
    } while (read_byte(bus, 0x09) != 0x10);

    val = 0x1D;
    write_reg(bus, 0x09, &val);


    magmeter_rawdata_t st_a;
    magmeter_read(bus, &st_a);
    rt_kprintf("SELF_TEST_A: (%d,%d,%d)\n", st_a.x, st_a.y, st_a.z);

    val = 0x13;
    write_reg(bus, 0x09, &val);

    do
    {
        rt_thread_mdelay(10);
        rt_kprintf("=");
    } while (read_byte(bus, 0x09) != 0x10);

    val = 0x1D;
    write_reg(bus, 0x09, &val);


    magmeter_rawdata_t st_b;
    magmeter_read(bus, &st_b);
    rt_kprintf("SELF_TEST_B: (%d,%d,%d)\n", st_b.x, st_b.y, st_b.z);
    rt_kprintf("Center Point: (%d,%d,%d)\n", st_a.x + st_b.x, st_a.y + st_b.y, st_a.z + st_b.z);

    return;
}


static struct rt_i2c_bus_device *i2c_bus = RT_NULL;     /* I2C总线设备句柄 */

int i2c_test_entry(void) {

    uint8_t buff[2] = {0,0};

    /* 查找I2C总线设备，获取I2C总线设备句柄 */
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(I2C0_NAME);

    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", I2C0_NAME);
    }

    write_reg(i2c_bus, QMC5883_REG_CHIP_ID, RT_NULL);

    read_regs(i2c_bus, 1, buff);

    rt_kprintf("QMC5883 Chip ID: 0x%02X\n", buff[0]);

    magmeter_reset(i2c_bus);

    rt_thread_mdelay(100);

    qmc5883_selftest(i2c_bus);

    return 0;
}


