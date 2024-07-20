# mcp2515-driver
> 未经过严格测试，目前可以正常使用，欢迎提出问题进行改进。

# 使用示例

```c
#include "bsp_mcp2515.h"
#inlcude "spi.h"

bsp_mcp2515_t spi_can;
mcp2515_can_msg_t spi_can_rxmsg;
mcp2515_can_msg_t spi_can_txmsg;

uint8_t spi_can_init(void)
{
    /* 8M晶振, SPI接口函数(SPI初始化、片选、读写一个字节) */
    if(mcp2515_init(&spi_can, MCP2515_OSC_8M, spi_init, spi_cs, spi_read_write_byte))
        return 1; // 初始化失败
    /* 设置CAN波特率250K， 设置后直接进入正常模式 */
    if(spi_can.set_baudrate(&spi_can, MCP2515_CAN_250K))
        return 2; // 设置波特率失败
 	
    /* 过滤器使用示例，复位后会重置过滤器接收全部消息 */
    /* 设置过滤器, 仅接收标准帧0x1c1 */
    // spi_can.set_filter_mask(&spi_can, MCP2515_MASK0, STANDARD_FRAME, 0x7FF);
    // spi_can.set_filter_mask(&spi_can, MCP2515_MASK1, STANDARD_FRAME, 0x7FF);
    // spi_can.set_filter(&spi_can, MCP2515_RXF0, STANDARD_FRAME, 0x1c1);
    // spi_can.set_filter(&spi_can, MCP2515_RXF1, STANDARD_FRAME, 0x1c1);
    // spi_can.set_filter(&spi_can, MCP2515_RXF2, STANDARD_FRAME, 0x1c1);
    // spi_can.set_filter(&spi_can, MCP2515_RXF3, STANDARD_FRAME, 0x1c1);
    // spi_can.set_filter(&spi_can, MCP2515_RXF4, STANDARD_FRAME, 0x1c1);
    // spi_can.set_filter(&spi_can, MCP2515_RXF5, STANDARD_FRAME, 0x1c1);
    /* 设置过滤器，仅接收标准帧0x1c1和0x1c2 */
    // spi_can.set_filter_mask(&spi_can, MCP2515_MASK0, STANDARD_FRAME, 0x7FF);
    // spi_can.set_filter_mask(&spi_can, MCP2515_MASK1, STANDARD_FRAME, 0x7FF);
    // spi_can.set_filter(&spi_can, MCP2515_RXF0, STANDARD_FRAME, 0x1c1);
    // spi_can.set_filter(&spi_can, MCP2515_RXF1, STANDARD_FRAME, 0x1c2);
    // spi_can.set_filter(&spi_can, MCP2515_RXF2, STANDARD_FRAME, 0x1c1);
    // spi_can.set_filter(&spi_can, MCP2515_RXF3, STANDARD_FRAME, 0x1c2);
    // spi_can.set_filter(&spi_can, MCP2515_RXF4, STANDARD_FRAME, 0x1c2);
    // spi_can.set_filter(&spi_can, MCP2515_RXF5, STANDARD_FRAME, 0x1c2);
    /* 设置过滤器，仅接收标准帧0x1c1、0x1c2、0x1c3、0x1c4 */
    // spi_can.set_filter_mask(&spi_can, MCP2515_MASK0, STANDARD_FRAME, 0x7FF);
    // spi_can.set_filter_mask(&spi_can, MCP2515_MASK1, STANDARD_FRAME, 0x7FF);
    // spi_can.set_filter(&spi_can, MCP2515_RXF0, STANDARD_FRAME, 0x1c1);
    // spi_can.set_filter(&spi_can, MCP2515_RXF1, STANDARD_FRAME, 0x1c2);
    // spi_can.set_filter(&spi_can, MCP2515_RXF2, STANDARD_FRAME, 0x1c1);
    // spi_can.set_filter(&spi_can, MCP2515_RXF3, STANDARD_FRAME, 0x1c2);
    // spi_can.set_filter(&spi_can, MCP2515_RXF4, STANDARD_FRAME, 0x1c3);
    // spi_can.set_filter(&spi_can, MCP2515_RXF5, STANDARD_FRAME, 0x1c4);
    /* 设置过滤器，仅接收标准帧0x1c0~0x1cf */
    // spi_can.set_filter_mask(&spi_can, MCP2515_MASK0, STANDARD_FRAME, 0x7F0);
    // spi_can.set_filter_mask(&spi_can, MCP2515_MASK1, STANDARD_FRAME, 0x7F0);
    // spi_can.set_filter(&spi_can, MCP2515_RXF0, STANDARD_FRAME, 0x1c0);
    // spi_can.set_filter(&spi_can, MCP2515_RXF1, STANDARD_FRAME, 0x1c0);
    // spi_can.set_filter(&spi_can, MCP2515_RXF2, STANDARD_FRAME, 0x1c0);
    // spi_can.set_filter(&spi_can, MCP2515_RXF3, STANDARD_FRAME, 0x1c0);
    // spi_can.set_filter(&spi_can, MCP2515_RXF4, STANDARD_FRAME, 0x1c0);
    // spi_can.set_filter(&spi_can, MCP2515_RXF5, STANDARD_FRAME, 0x1c0);
    
    /* RXB0&RXB1接收中断，如果用到中断模式接收则开启 */
    // spi_can.enable_interrupt(&spi_can, RX_BUF0_FULL_INT);
    // spi_can.enable_interrupt(&spi_can, RX_BUF1_FULL_INT);
    /* 唤醒中断(进入睡眠模式函数中已设置)，进入睡眠模式后，可触发配置的外部中断以唤醒MCU */
    // spi_can.enable_interrupt(&spi_can, WAKEUP_INT);
    return 0;
}

int main(void)
{
    spi_can_init();
    
    while(1)
    {
        // 轮询接收消息 中断接收参考代码注释，与之类似
        if(spi_can.receive(&spi_can, &spi_can_rxmsg))
    	{
            // 处理消息
            spi_can_txmsg = spi_can_rxmsg;
            spi_can.transmit(&spi_can, &spi_can_txmsg);
        }
    }
}
```
