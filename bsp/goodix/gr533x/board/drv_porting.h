#ifndef __DRV_PORTING_H__
#define __DRV_PORTING_H__


/*********************************************
 *         Porting Note for pin
 * 
 * 1. PIN 编号是用的芯片的PAD 编号, QFN32封装是 1-32; QFN48封装是1-48, 0占位用; 封装选择用宏 GR533x_PACKAGE_TYPE 控制
 * 2. 暂时未支持 rt_pin_get 接口
 * 3. 芯片不支持 PIN_MODE_OUTPUT_OD 模式, PIN_MODE_OUTPUT_OD 实际设置为 PIN_MODE_OUTPUT 模式
 *
 *
 *************************************************/


/*********************************************
 *         Porting Note for SPI
 * 
 * 1. RT-T 没有框架层的 接口配置, 使用时候, 需要先根据 SPI 引脚分配情况, 在 drv_porting_spi.c 配置各pin 的使用情况
 * 2. SPI 的cs pin, 用的 RT-T 的框架接口控制, 因此使用时候,需要指定下其 引脚编号
 * 3. 当前版本只支持了 SPI Master, 暂未支持 SPI Slave
 * 4. SPIM 的频率支持主频的偶数分频, 分频范围 [2, 65535), 配置时候请注意, 如果没有按照此原则设置, 会配置为跟用户参数最接近的频率
 * 5. RT-T SPI驱动框架的设计, 不利于发挥SPI 的高吞吐率, 时序上存在不少的浪费. 高吞吐场景需要注意
 * 6. RT-T SPI 驱动目前支持的 DMA传输方式
 * 
 *************************************************/



/*********************************************
 *         Porting Note for I2C
 * 
 * 1. 目前不支持 RT_I2C_NO_START、RT_I2C_IGNORE_NACK、RT_I2C_NO_READ_ACK、RT_I2C_NO_STOP 几种操作模式
 * 2. 由于RT-Thread 驱动框架没有提供初始化/配置入口 (control 方式实现不完善), 目前模块初始化随 bus 注册一起完成, 需要用户再头文件根据实际进行宏配置
 * 3. i2c_bus_control 暂时空实现
 * 4. I2C 模块建议优先用轮询或中断方式访问, 不推荐DMA方式
 * 5. 如果要使用 10bit 地址模式, 请通过 rt_i2c_msg 的 flags 标记传递 RT_I2C_ADDR_10BIT
 * 
 *************************************************/



/*********************************************
 *
 *         移植中存在的 [可能异常逻辑(Possible Execption Logic)], 通过编号搜索,可能存在Bug隐患
 *
 * 1) NO.1001 : putc 返回值-1, 会等完成, 然后continue:
 *              如果-1表示发送没有完成,rt_completion_wait 后使用continue, 直接从循环开始,
 *              导致数据会再发一次; 如果-1表示传输错误, 那 rt_completion_wait 在等什么呢?
 *
 * 2) NO.1002 : serial->ops->control(serial, RT_DEVICE_CTRL_CONFIG, (void *)..) 
 *              在串口设备代码中RT_DEVICE_CTRL_CONFIG既搭配 传输标志使用, 又搭配参数配置使用, 导致移植层存在歧义
 *
 *************************************************/


#define DMA_Channel_DISABLE     DMA_Channel_NUM_MAX


#endif /* __DRV_PORTING_H__ */
