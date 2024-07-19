# mcp2515-driver
> 未经过严格测试，目前可以正常使用，欢迎提出问题进行改进。

# 简单使用示例

```c
bsp_mcp2515_t spi_can;

uint8_t spi_can_init(void)
{
    /* 8M晶振, SPI接口函数(SPI初始化、片选、读写一个字节) */
    if(mcp2515_init(&spi_can, MCP2515_OSC_8M, spi_init, spi_cs, spi_read_write_byte))
        return 1; // 初始化失败
    /* 设置CAN波特率250K， 设置后直接进入正常模式 */
    if(spi_can.set_baudrate(&spi_can, MCP2515_CAN_250K))
        return 2; // 设置波特率失败
    // spi_can.enable_interrupt(&spi_can, RX_BUF0_FULL_INT); // 使能中断，如果用到开启
    // spi_can.enable_interrupt(&spi_can, RX_BUF0_FULL_INT);
    return 0;
}

mcp2515_can_msg_t spi_can_rxmsg;
mcp2515_can_msg_t spi_can_txmsg;

int main(void)
{
    spi_can_init();
    
    while(1)
    {
        // 轮询接收消息 中断接收参考注释，与之类似
        if(spi_can.receive(&spi_can, &spi_can_rxmsg))
    	{
            // 处理消息
            spi_can_txmsg = spi_can_rxmsg;
            spi_can.transmit(&spi_can, &spi_can_txmsg)
        }
    }
}
```
