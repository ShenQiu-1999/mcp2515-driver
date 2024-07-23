#include <string.h>
#include "bsp_mcp2515.h"

static uint8_t mcp2515_read_byte(bsp_mcp2515_t* handle, uint8_t address);
static void mcp2515_read_bytes(bsp_mcp2515_t* handle, uint8_t start_address, uint8_t* data, uint8_t length);
static void mcp2515_write_byte(bsp_mcp2515_t* handle, uint8_t address, uint8_t data);
static void mcp2515_write_bytes(bsp_mcp2515_t* handle, uint8_t start_address, uint8_t* data, uint8_t length);
static void mcp2515_bit_modify(bsp_mcp2515_t* handle, uint8_t address, uint8_t mask, uint8_t data);
static uint8_t mcp2515_read_status(bsp_mcp2515_t* handle);
static uint8_t mcp2515_get_rx_status(bsp_mcp2515_t* handle);
static void mcp2515_request_to_send(bsp_mcp2515_t* handle, uint8_t instruction);
static uint32_t convert_reg_to_exid(uint8_t RXBnEID8, uint8_t RXBnEID0, uint8_t RXBnSIDH, uint8_t RXBnSIDL);
static uint32_t convert_reg_to_stid(uint8_t RXBnSIDH, uint8_t RXBnSIDL);
static void convert_id_to_reg(uint32_t id, uint8_t id_typ, id_reg_t *id_reg);
static uint8_t mcp2515_set_mode(bsp_mcp2515_t* handle, uint8_t mode);
static uint8_t mcp2515_set_config_mode(bsp_mcp2515_t* handle);
static uint8_t mcp2515_set_normal_mode(bsp_mcp2515_t* handle);
static uint8_t mcp2515_set_sleep_mode(bsp_mcp2515_t* handle);
static uint8_t mcp2515_set_loopback_mode(bsp_mcp2515_t* handle);
static uint8_t mcp2515_set_listenonly_mode(bsp_mcp2515_t* handle);

static uint8_t mcp2515_reset(bsp_mcp2515_t* handle);
static uint8_t mcp2515_set_baudrate(bsp_mcp2515_t* handle, uint32_t baudrate);
static uint8_t mcp2515_enter_sleep_mode(bsp_mcp2515_t* handle);
static uint8_t mcp2515_transmit(bsp_mcp2515_t* handle, mcp2515_can_msg_t* mcp2515_can_msg);
static uint8_t mcp2515_receive(bsp_mcp2515_t* handle, mcp2515_can_msg_t* mcp2515_can_msg);
static uint8_t mcp2515_receive_isr(bsp_mcp2515_t* handle, uint8_t rxbuf, mcp2515_can_msg_t* mcp2515_can_msg);
static uint8_t mcp2515_is_bus_off(bsp_mcp2515_t* handle);
static uint8_t mcp2515_is_rx_error(bsp_mcp2515_t* handle);
static uint8_t mcp2515_is_tx_error(bsp_mcp2515_t* handle);
static uint8_t mcp2515_get_rx_error_count(bsp_mcp2515_t* handle);
static uint8_t mcp2515_get_tx_error_count(bsp_mcp2515_t* handle);
static uint8_t set_filter_mask(bsp_mcp2515_t* handle, uint8_t mask, uint8_t ide, uint32_t data);
static uint8_t set_filter(bsp_mcp2515_t* handle, uint8_t rxf, uint8_t ide, uint32_t data);
static void enable_interrupt(bsp_mcp2515_t* handle, uint8_t interrupt);
static void disable_interrupt(bsp_mcp2515_t* handle, uint8_t interrupt);
static uint8_t get_interrupt_flag(bsp_mcp2515_t* handle, uint8_t interrupt);
static void clear_interrupt_flag(bsp_mcp2515_t* handle, uint8_t interrupt);

/**
 * @brief	
 * @param	
 * @retval	0: success; 1: failed
 */
uint8_t mcp2515_init(bsp_mcp2515_t* handle,
                     uint32_t osc_freq,
                     void (*spi_init)(void),
                     void (*spi_cs)(uint8_t),
                     uint8_t (*spi_read_write_byte)(uint8_t))
{
    /* check params */
    if(NULL == handle || NULL == spi_init || NULL == spi_cs || NULL == spi_read_write_byte)
    {
        return 1;
    }
    memset(handle, 0, sizeof(bsp_mcp2515_t));
    handle->osc_freq = osc_freq;
    handle->spi_interface.spi_init = spi_init;
    handle->spi_interface.spi_cs = spi_cs;
    handle->spi_interface.spi_read_write_byte = spi_read_write_byte;

    handle->reset = mcp2515_reset;
    handle->set_baudrate = mcp2515_set_baudrate;
    handle->receive = mcp2515_receive;
    handle->receive_isr = mcp2515_receive_isr;
    handle->transmit = mcp2515_transmit;
    handle->is_bus_off = mcp2515_is_bus_off;
    handle->is_rx_error = mcp2515_is_rx_error;
    handle->is_tx_error = mcp2515_is_tx_error;
    handle->get_rx_error_count = mcp2515_get_rx_error_count;
    handle->get_tx_error_count = mcp2515_get_tx_error_count;
    handle->set_filter_mask = set_filter_mask;
    handle->set_filter = set_filter;
    handle->enter_sleep_mode = mcp2515_enter_sleep_mode;
    handle->set_loopback_mode = mcp2515_set_loopback_mode;
    handle->set_listenonly_mode = mcp2515_set_listenonly_mode;
    handle->enable_interrupt = enable_interrupt;
    handle->disable_interrupt = disable_interrupt;
    handle->get_interrupt_flag = get_interrupt_flag;
    handle->clear_interrupt_flag = clear_interrupt_flag;

    handle->spi_interface.spi_init();
    handle->spi_interface.spi_cs(1);
    
    if(mcp2515_reset(handle))
    {
        return 1;
    }

    return 0;
}

static uint8_t mcp2515_read_byte(bsp_mcp2515_t* handle, uint8_t address)
{
    uint8_t data;
    handle->spi_interface.spi_cs(0);
    handle->spi_interface.spi_read_write_byte(MCP2515_READ);
    handle->spi_interface.spi_read_write_byte(address);
    data = handle->spi_interface.spi_read_write_byte(0xFF); /* don't care 0xFF */
    handle->spi_interface.spi_cs(1);
    return data;
}

static void mcp2515_read_bytes(bsp_mcp2515_t* handle, uint8_t start_address, uint8_t* data, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = mcp2515_read_byte(handle, start_address++);
    }
}

static void mcp2515_write_byte(bsp_mcp2515_t* handle, uint8_t address, uint8_t data)
{
    handle->spi_interface.spi_cs(0);
    handle->spi_interface.spi_read_write_byte(MCP2515_WRITE);
    handle->spi_interface.spi_read_write_byte(address);
    data = handle->spi_interface.spi_read_write_byte(data);
    handle->spi_interface.spi_cs(1);
}

static void mcp2515_write_bytes(bsp_mcp2515_t* handle, uint8_t start_address, uint8_t* data, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        mcp2515_write_byte(handle, start_address++, data[i]);
    }
}

/**
 * @brief	  只修改寄存器中的某些位
 * @param	  address: 寄存器地址  
 * @param   mask   : 屏蔽字, 1表示该位需要修改
 * @param   data   : 数据字节
 * @retval	None
 */
static void mcp2515_bit_modify(bsp_mcp2515_t* handle, uint8_t address, uint8_t mask, uint8_t data)
{
    handle->spi_interface.spi_cs(0);
    handle->spi_interface.spi_read_write_byte(MCP2515_BIT_MODIFY);
    handle->spi_interface.spi_read_write_byte(address);
    handle->spi_interface.spi_read_write_byte(mask);
    handle->spi_interface.spi_read_write_byte(data);
    handle->spi_interface.spi_cs(1);
}

/* read status */
static uint8_t mcp2515_read_status(bsp_mcp2515_t* handle)
{
    uint8_t status;
    handle->spi_interface.spi_cs(0);
    handle->spi_interface.spi_read_write_byte(MCP2515_READ_STATUS);
    status = handle->spi_interface.spi_read_write_byte(0xFF);
    handle->spi_interface.spi_cs(1);
    return status;
}

/* read RX STATUS register */
static uint8_t mcp2515_get_rx_status(bsp_mcp2515_t* handle)
{
    uint8_t status;
    handle->spi_interface.spi_cs(0);
    handle->spi_interface.spi_read_write_byte(MCP2515_RX_STATUS);
    status = handle->spi_interface.spi_read_write_byte(0xFF);
    handle->spi_interface.spi_cs(1);
    return status;
}

static void mcp2515_request_to_send(bsp_mcp2515_t* handle, uint8_t instruction)
{
    handle->spi_interface.spi_cs(0);
    handle->spi_interface.spi_read_write_byte(instruction);
    handle->spi_interface.spi_cs(1);
}

/* convert register value to extended CAN ID */
static uint32_t convert_reg_to_exid(uint8_t RXBnEID8, uint8_t RXBnEID0, uint8_t RXBnSIDH, uint8_t RXBnSIDL)
{
    uint32_t converted_id;
    uint8_t RXBnSIDL_low_2bits;
    uint8_t RXBnSIDL_high_3bits;
    
    RXBnSIDL_low_2bits = (RXBnSIDL & 0x03);
    RXBnSIDL_high_3bits = (RXBnSIDL >> 5);

    converted_id = RXBnEID0 + ((uint32_t)RXBnEID8 << 8) + ((uint32_t)RXBnSIDL_low_2bits << 16) 
                    + ((uint32_t)RXBnSIDL_high_3bits << 18) + ((uint32_t)RXBnSIDH << 21);

    return converted_id;
}

/* convert register value to standard CAN ID */
static uint32_t convert_reg_to_stid(uint8_t RXBnSIDH, uint8_t RXBnSIDL)
{
    uint32_t converted_id;

    converted_id = ((uint32_t)RXBnSIDH << 3) + ((uint32_t)RXBnSIDL >> 5);
    return converted_id;
}

/* convert CAN ID to register value */
static void convert_id_to_reg(uint32_t id, uint8_t id_typ, id_reg_t *id_reg)
{
    uint32_t exid_high_11bits;
    uint32_t exid_low_18bits;

    if (id_typ == EXTENDED_FRAME) 
    {
        exid_high_11bits = ((id >> 18) & 0x7FF); /* 扩展ID高11位 */
        exid_low_18bits = (id & 0x03FFFF); /* 扩展ID低18位 */

        //EID0
        id_reg->EID0 = (uint8_t)exid_low_18bits;

        //EID8
        id_reg->EID8 = (uint8_t)(exid_low_18bits >> 8);

        //SIDL
        id_reg->SIDL = (uint8_t)((exid_high_11bits << 5) & 0xE0) | 0x08 | (uint8_t)((exid_low_18bits >> 16) & 0x03);

        //SIDH
        id_reg->SIDH = (uint8_t)(exid_high_11bits >> 3);
    } 
    else
    {
        id_reg->EID8 = 0;
        id_reg->EID0 = 0;

        id_reg->SIDH = (uint8_t)(id >> 3); /* 标准ID高8位 */
        id_reg->SIDL = (uint8_t)((id << 5) & 0xE0); /* 标准ID低3位 */
    }
}

static uint8_t mcp2515_set_mode(bsp_mcp2515_t* handle, uint8_t mode)
{
    /* configure CANCTRL Register */
    mcp2515_bit_modify(handle, MCP2515_CANCTRL, 0xE0, mode);
    
    uint32_t loop = 0xFFFFF;
    
    do {
        /* confirm mode configuration */
        if((mcp2515_read_byte(handle, MCP2515_CANSTAT) & 0xE0) == mode)
            return 0;
        
    } while(loop--);
      
    return 1;
}

/* change mode as configuration mode */
static uint8_t mcp2515_set_config_mode(bsp_mcp2515_t* handle)
{
    return mcp2515_set_mode(handle, 0x80);
}

/* change mode as normal mode */
static uint8_t mcp2515_set_normal_mode(bsp_mcp2515_t* handle)
{
    return mcp2515_set_mode(handle, 0x00);
}

static uint8_t mcp2515_set_sleep_mode(bsp_mcp2515_t* handle)
{
    return mcp2515_set_mode(handle, 0x20);
}

uint8_t mcp2515_set_loopback_mode(bsp_mcp2515_t* handle)
{
    return mcp2515_set_mode(handle, 0x40);
}

uint8_t mcp2515_set_listenonly_mode(bsp_mcp2515_t* handle)
{
    return mcp2515_set_mode(handle, 0x60);
}

/* MCP2515 Reset */
uint8_t mcp2515_reset(bsp_mcp2515_t* handle)
{
    RXMn RXM0, RXM1;
    RXFn RXF0, RXF1;

    handle->spi_interface.spi_cs(0);
    handle->spi_interface.spi_read_write_byte(MCP2515_RESET);
    handle->spi_interface.spi_cs(1);

    uint32_t loop = 0xFFFFF;
    
    do {
        /* confirm mode configuration */
        if((mcp2515_read_byte(handle, MCP2515_CANSTAT) & 0xE0) == 0x80)
            break;
        
    } while(loop--);

    if(loop == 0) return 1; // 未进入初始化模式, 无法修改mask&filter寄存器, 返回失败

    mcp2515_bit_modify(handle, MCP2515_CANCTRL, 0x04, 0x00); // disable clkout pin

    /* Intialize Rx Mask values */
    RXM0.RXMnSIDH = 0x00;
    RXM0.RXMnSIDL = 0x00; 
    RXM0.RXMnEID8 = 0x00;
    RXM0.RXMnEID0 = 0x00;

    RXM1.RXMnSIDH = 0x00;
    RXM1.RXMnSIDL = 0x00; 
    RXM1.RXMnEID8 = 0x00;
    RXM1.RXMnEID0 = 0x00;
    
    /* Intialize Rx Filter values */
    RXF0.RXFnSIDH = 0x00;
    RXF0.RXFnSIDL = 0x00; // Starndard Filter
    RXF0.RXFnEID8 = 0x00;
    RXF0.RXFnEID0 = 0x00;

    RXF1.RXFnSIDH = 0x00;
    RXF1.RXFnSIDL = 0x08; // Exntended Filter
    RXF1.RXFnEID8 = 0x00;
    RXF1.RXFnEID0 = 0x00;

    /* Configure filter & mask */
    mcp2515_write_bytes(handle, MCP2515_RXM0SIDH, &(RXM0.RXMnSIDH), 4);
    mcp2515_write_bytes(handle, MCP2515_RXM1SIDH, &(RXM1.RXMnSIDH), 4);
    mcp2515_write_bytes(handle, MCP2515_RXF0SIDH, &(RXF0.RXFnSIDH), 4);
    mcp2515_write_bytes(handle, MCP2515_RXF1SIDH, &(RXF1.RXFnSIDH), 4);
    mcp2515_write_bytes(handle, MCP2515_RXF2SIDH, &(RXF0.RXFnSIDH), 4);
    mcp2515_write_bytes(handle, MCP2515_RXF3SIDH, &(RXF0.RXFnSIDH), 4);
    mcp2515_write_bytes(handle, MCP2515_RXF4SIDH, &(RXF0.RXFnSIDH), 4);
    mcp2515_write_bytes(handle, MCP2515_RXF5SIDH, &(RXF0.RXFnSIDH), 4);

    /* Accept All (Standard + Extended) */
    // mcp2515_write_byte(handle, MCP2515_RXB0CTRL, 0x04); // Enable BUKT, Accept Filter 0
    // mcp2515_write_byte(handle, MCP2515_RXB1CTRL, 0x01); // Accept Filter 1

    mcp2515_write_byte(handle, MCP2515_RXB0CTRL, 0x04); // RXB0 message will roll over and be written to RXB1 if RXB0 is full
    mcp2515_write_byte(handle, MCP2515_RXB1CTRL, 0x00);

    return 0;
}

uint8_t mcp2515_set_clkout(bsp_mcp2515_t* handle, uint8_t divisor)
{
    if(divisor == CLKOUT_DISABLE)
    {
        /* Turn off CLKEN */
        mcp2515_bit_modify(handle, MCP2515_CANCTRL, 0x04, 0x00);

        /* Turn on CLKOUT for SOF */
        // mcp2515_bit_modify(handle, MCP2515_CNF3, 0x80, 0x80); // must in configuration mode
        return 0;
    }
    
    /* Set the prescaler (CLKPRE) */
    mcp2515_bit_modify(handle, MCP2515_CANCTRL, 0x03, divisor);

    /* Turn on CLKEN */
    mcp2515_bit_modify(handle, MCP2515_CANCTRL, 0x04, 0x04);


    /* Turn off CLKOUT for SOF */
    // mcp2515_bit_modify(handle, MCP2515_CNF3, 0x80, 0x00);
    return 0;
}

static int abs(int x) {
    return (x < 0) ? -x : x;
}

/*
 *  PropSeg + PS1 ≥ PS2
 *  PropSeg + PS1 ≥ TDELAY (typically, the TDELAY is 1-2 TQs)
 *  PS2 ≥ SJW
 *  Minimum valid setting for PS2 is 2 TQs.
 */
static void calculate_baudrate(bsp_mcp2515_t* handle, uint32_t baudrate, uint32_t osc_freq)
{
    uint8_t brp, propSeg, ps1, ps2;
    float tq;
    uint32_t bestError = 1000000000;
    uint32_t error;
    uint8_t bestBrp = 0, bestPropSeg = 0, bestPs1 = 0, bestPs2 = 0;
    
    for (brp = 0; brp < 64; brp++) {
        tq = (float)(2 * (brp + 1)) / osc_freq;
        for (propSeg = 1; propSeg <= 8; propSeg++) {
            for (ps1 = 1; ps1 <= 8; ps1++) {
                for (ps2 = 2; ps2 <= 8; ps2++) {
                    // 确保满足约束条件
                    if ((propSeg + ps1 >= ps2) && (propSeg + ps1 >= 2) && (ps2 >= 1)) { // 假设SJW = 1
                        uint32_t totalTq = 1 + propSeg + ps1 + ps2;
                        uint32_t baud = (uint32_t)(1.0 / (tq * totalTq));
                        error = abs(baudrate - baud);
                        if (error < bestError) {
                            bestError = error;
                            bestBrp = brp;
                            bestPropSeg = propSeg;
                            bestPs1 = ps1;
                            bestPs2 = ps2;
                        }
                    }
                }
            }
        }
    }
    mcp2515_write_byte(handle, MCP2515_CNF1, (bestBrp & 0x3F));
    mcp2515_write_byte(handle, MCP2515_CNF2, (0xC0 | ((bestPs1 - 1) & 0x07) << 3 | ((bestPropSeg - 1) & 0x07)));
    mcp2515_write_byte(handle, MCP2515_CNF3, (bestPs2 - 1) & 0x07);

    // printf("bestBrp: %d, bestPropSeg: %d, bestPs1: %d, bestPs2: %d", bestBrp ,bestPropSeg, bestPs1, bestPs2);
}

/**
 * @brief	
 *  @note   不使用宏定义, 则自动计算波特率参数, 传入参数示例: 500000, 8000000(500K,8M晶振); 250000, 20000000(250K,20M晶振)
 *          晶振频率在初始化中传入, 使用到没有宏定义的CAN波特率, 则直接传入相应频率即可, 单位: bps
 *          设置波特率后直接进入正常模式, 如需进入其他模式, 则再调用其他函数
 * @param	
 * @retval	0: success; 1: failed
 */
uint8_t mcp2515_set_baudrate(bsp_mcp2515_t* handle, uint32_t baudrate)
{
    /* Change mode as configuration mode */
    if(mcp2515_set_config_mode(handle))
    {
        return 1;
    }
    /* Accept All (Standard + Extended) */
    // mcp2515_write_byte(handle, MCP2515_RXB0CTRL, 0x64); // don't use mask & filter
    // mcp2515_write_byte(handle, MCP2515_RXB1CTRL, 0x60);

    /*  rules:
     *  PropSeg + PS1 ≥ PS2
     *  PropSeg + PS1 ≥ TDELAY typically, the TDELAY is 1-2 TQs
     *  PS2 ≥ SJW
     */
    switch (handle->osc_freq)
    {
        case MCP2515_OSC_8M:
            /* 
            *  tq = 2 * (brp(0) + 1) / 8000000 = 0.25us
            *  tbit = (SYNC_SEG(1 fixed) + PROP_SEG + PS1 + PS2)
            *  tbit = 1tq + 3tq + 6tq + 6tq = 16tq
            *  16tq = 4us = 250kbps
            */
            if(baudrate == MCP2515_CAN_250K)
            {
                mcp2515_write_byte(handle, MCP2515_CNF1, 0x00);
                mcp2515_write_byte(handle, MCP2515_CNF2, 0xEA);
                mcp2515_write_byte(handle, MCP2515_CNF3, 0x05); // Minimum valid setting for PS2 is 2 TQs.
            }
            else if(baudrate == MCP2515_CAN_500K)
            {
                /* 
                 *  tq = 2 * (brp(0) + 1) / 8000000 = 0.25us
                 *  tbit = (SYNC_SEG(1 fixed) + PROP_SEG + PS1 + PS2)
                 *  tbit = 1tq + 1tq + 4tq + 2tq = 8tq
                 *  8tq = 2us = 500kbps
                 */
                mcp2515_write_byte(handle, MCP2515_CNF1, 0x00);
                mcp2515_write_byte(handle, MCP2515_CNF2, 0xD8);
                mcp2515_write_byte(handle, MCP2515_CNF3, 0x01);
            }
            else
            {
                calculate_baudrate(handle, baudrate, 8000000);
            }
            break;

        case MCP2515_OSC_16M:
            /* 
            *  tq = 2 * (brp(0) + 1) / 16000000 = 0.5us
            *  tbit = (SYNC_SEG(1 fixed) + PROP_SEG + PS1 + PS2)
            *  tbit = 1tq + 1tq + 3tq + 3tq = 8tq
            *  16tq = 4us = 250kbps
            */
            if(baudrate == MCP2515_CAN_250K)
            {
                mcp2515_write_byte(handle, MCP2515_CNF1, 0x03);
                mcp2515_write_byte(handle, MCP2515_CNF2, 0xD0);
                mcp2515_write_byte(handle, MCP2515_CNF3, 0x02); // Minimum valid setting for PS2 is 2 TQs.
            }
            else if(baudrate == MCP2515_CAN_500K)
            {
                /* 
                 *  tq = 2 * (brp(0) + 1) / 16000000 = 0.25us
                 *  tbit = (SYNC_SEG(1 fixed) + PROP_SEG + PS1 + PS2)
                 *  tbit = 1tq + 1tq + 3tq + 3tq = 8tq
                 *  8tq = 2us = 500kbps
                 */
                mcp2515_write_byte(handle, MCP2515_CNF1, 0x01);
                mcp2515_write_byte(handle, MCP2515_CNF2, 0xD0);
                mcp2515_write_byte(handle, MCP2515_CNF3, 0x02);
            }
            else
            {
                calculate_baudrate(handle, baudrate, 16000000);
            }
            break;

        case MCP2515_OSC_20M:
            if(baudrate == MCP2515_CAN_250K)
            {
                /* 
                 *  tq = 2 * (brp(0) + 1) / 20000000 = 0.4us
                 *  tbit = (SYNC_SEG(1 fixed) + PROP_SEG + PS1 + PS2)
                 *  tbit = 1tq + 2tq + 3tq + 4tq = 10tq
                 *  10tq = 4us = 250kbps
                 */
                mcp2515_write_byte(handle, MCP2515_CNF1, 0x03);
                mcp2515_write_byte(handle, MCP2515_CNF2, 0xD1);
                mcp2515_write_byte(handle, MCP2515_CNF3, 0x03);
            }
            else if(baudrate == MCP2515_CAN_500K)
            {
                /* 
                 *  tq = 2 * (brp(0) + 1) / 20000000 = 0.2us
                 *  tbit = (SYNC_SEG(1 fixed) + PROP_SEG + PS1 + PS2)
                 *  tbit = 1tq + 1tq + 4tq + 3tq = 10tq
                 *  10tq = 2us = 500kbps
                 */
                mcp2515_write_byte(handle, MCP2515_CNF1, 0x01);
                mcp2515_write_byte(handle, MCP2515_CNF2, 0xD8);
                mcp2515_write_byte(handle, MCP2515_CNF3, 0x02);
            }
            else
            {
                calculate_baudrate(handle, baudrate, 20000000);
            }
            break;

        default:
            if(baudrate == MCP2515_CAN_250K)
                calculate_baudrate(handle, 250000, handle->osc_freq);
            else if(baudrate == MCP2515_CAN_500K)
                calculate_baudrate(handle, 500000, handle->osc_freq);
            else
                calculate_baudrate(handle, baudrate, handle->osc_freq);
            break;
    }

    /* Normal Mode */
    if(mcp2515_set_normal_mode(handle))
    {
        return 1;
    }

    return 0;
}

/* Entering Sleep Mode */
uint8_t mcp2515_enter_sleep_mode(bsp_mcp2515_t* handle)
{
    /* Clear CAN bus wakeup interrupt */
    mcp2515_bit_modify(handle, MCP2515_CANINTF, 0x40, 0x00);
      
    /* Enable CAN bus activity wakeup */
    mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x40, 0x40);
  
    return mcp2515_set_sleep_mode(handle); /* 开启唤醒中断, MCU也可进入休眠模式, 接收到消息也可以(外部中断)唤醒MCU */
}

/**
 * @brief	  Transmit CAN message
 * @param	
 * @retval	0: transmit success; 1: all txbuf busy; 2: transmit failed
 */
uint8_t mcp2515_transmit(bsp_mcp2515_t* handle, mcp2515_can_msg_t* mcp2515_can_msg)
{
    ctrl_status_t status;
    id_reg_t id_reg;

    status.ctrl_status = mcp2515_read_status(handle);

    /* Finding empty buffer */
    if (status.TXB0REQ != 1)
    {
        /* convert CAN ID for register */
        convert_id_to_reg(mcp2515_can_msg->id, mcp2515_can_msg->id_type, &id_reg);

        /* Load data to Tx Buffer */
        mcp2515_write_bytes(handle, MCP2515_TXB0SIDH, &id_reg.SIDH, 4);
        mcp2515_write_byte(handle, MCP2515_TXB0DLC, mcp2515_can_msg->dlc & 0x0F); // default: RTR=0,Transmitted message will be a data frame
        mcp2515_write_bytes(handle, MCP2515_TXB0D0, mcp2515_can_msg->data, 8);

        /* Request to transmit */
        mcp2515_request_to_send(handle, MCP2515_RTS_TX0);

        // uint8_t ctrl = mcp2515_read_byte(handle, MCP2515_TXB0CTRL);
        // if((ctrl & 0x70) != 0)
        //     return 2;
        // else
            return 0;
    }
    else if (status.TXB1REQ != 1)
    {
        convert_id_to_reg(mcp2515_can_msg->id, mcp2515_can_msg->id_type, &id_reg);

        mcp2515_write_bytes(handle, MCP2515_TXB1SIDH, &id_reg.SIDH, 4);
        mcp2515_write_byte(handle, MCP2515_TXB1DLC, mcp2515_can_msg->dlc & 0x0F);
        mcp2515_write_bytes(handle, MCP2515_TXB1D0, mcp2515_can_msg->data, 8);

        mcp2515_request_to_send(handle, MCP2515_RTS_TX1);

        // uint8_t ctrl = mcp2515_read_byte(handle, MCP2515_TXB1CTRL);
        // if((ctrl & 0x70) != 0)
        //     return 2;
        // else
            return 0;
    }
    else if (status.TXB2REQ != 1)
    {
        convert_id_to_reg(mcp2515_can_msg->id, mcp2515_can_msg->id_type, &id_reg);

        mcp2515_write_bytes(handle, MCP2515_TXB2SIDH, &id_reg.SIDH, 4);
        mcp2515_write_byte(handle, MCP2515_TXB2DLC, mcp2515_can_msg->dlc & 0x0F);
        mcp2515_write_bytes(handle, MCP2515_TXB2D0, mcp2515_can_msg->data, 8);

        mcp2515_request_to_send(handle, MCP2515_RTS_TX2);

        // uint8_t ctrl = mcp2515_read_byte(handle, MCP2515_TXB2CTRL);
        // if((ctrl & 0x70) != 0)
        //     return 2;
        // else
            return 0;
    }

    return 1;
}

/* You can choose one of two method to receive: interrupt-based and polling */

/**
 * @brief	  Received in polling mode
 * @param	
 * @retval	0: not received message; 1: received message
 */
uint8_t mcp2515_receive(bsp_mcp2515_t* handle, mcp2515_can_msg_t* mcp2515_can_msg)
{
    rx_reg_t rx_reg;
    ctrl_rx_status_t rx_status;

    rx_status.ctrl_rx_status = mcp2515_get_rx_status(handle);

    /* Check receive buffer */
    if (rx_status.rx_buffer != 0)
    {
        /* finding buffer which has a message */
        if ((rx_status.rx_buffer == MESSAGE_IN_RXB0) || (rx_status.rx_buffer == MESSAGE_IN_BOTH))
        {
            mcp2515_read_bytes(handle, MCP2515_RXB0SIDH, rx_reg.rx_reg_array, sizeof(rx_reg.rx_reg_array));
            clear_interrupt_flag(handle, RX_BUF0_FULL_INT);
        }
        else if (rx_status.rx_buffer == MESSAGE_IN_RXB1)
        {
            mcp2515_read_bytes(handle, MCP2515_RXB1SIDH, rx_reg.rx_reg_array, sizeof(rx_reg.rx_reg_array));
            clear_interrupt_flag(handle, RX_BUF1_FULL_INT);
        }

        /* if the message is extended CAN type */
        if (rx_status.msg_type == EXTENDED_DATA_FRAME)
        {
            mcp2515_can_msg->id_type = 1;
            mcp2515_can_msg->id = convert_reg_to_exid(rx_reg.RXBnEID8, rx_reg.RXBnEID0, rx_reg.RXBnSIDH, rx_reg.RXBnSIDL);
        }
        else
        {
            /* Standard type */
            mcp2515_can_msg->id_type = 0;
            mcp2515_can_msg->id = convert_reg_to_stid(rx_reg.RXBnSIDH, rx_reg.RXBnSIDL);
        }
        mcp2515_can_msg->dlc = rx_reg.RXBnDLC.DLC;
        memcpy(mcp2515_can_msg->data, rx_reg.RXBnDm, 8);

        return 1;
    }
    return 0;
}

/**
 * @brief	
 *  @note   在中断标记, 在主程序调用该函数读取数据 
 *          volatile bool interrupt_flag = false; 
 *          void irqHandler() {interrupt_flag = true;}
 *          if (interrupt_flag) {
 *              interrupt_flag = false;
 *              if (mcp2515.receive_isr(&mcp2515, MCP2515_RXB0, &rx_can_msg)) {
                    // rx_can_msg contains received from RXB0 message
                }
                if (mcp2515.receive_isr(&mcp2515, MCP2515_RXB1, &rx_can_msg)) {
                    // rx_can_msg contains received from RXB1 message
                }
            }
 * @param	  rxbuf: only one parameter can be selected which is shown as below:
    \arg        MCP2515_RXB0
    \arg        MCP2515_RXB1
 * @retval	0: not received message; 1: received message
 */
uint8_t mcp2515_receive_isr(bsp_mcp2515_t* handle, uint8_t rxbuf, mcp2515_can_msg_t* mcp2515_can_msg)
{
    rx_reg_t rx_reg;
    if(rxbuf == MCP2515_RXB0 && get_interrupt_flag(handle, RX_BUF0_FULL_INT))
    {
        mcp2515_read_bytes(handle, MCP2515_RXB0SIDH, rx_reg.rx_reg_array, sizeof(rx_reg.rx_reg_array));
        if(rx_reg.RXBnSIDL & 0x08)
        {
            mcp2515_can_msg->id_type = 1;
            mcp2515_can_msg->id = convert_reg_to_exid(rx_reg.RXBnEID8, rx_reg.RXBnEID0, rx_reg.RXBnSIDH, rx_reg.RXBnSIDL);
        }
        else
        {
            mcp2515_can_msg->id_type = 0;
            mcp2515_can_msg->id = convert_reg_to_stid(rx_reg.RXBnSIDH, rx_reg.RXBnSIDL);
        }
        mcp2515_can_msg->dlc = rx_reg.RXBnDLC.DLC;
        memcpy(mcp2515_can_msg->data, rx_reg.RXBnDm, 8);

        clear_interrupt_flag(handle, RX_BUF0_FULL_INT);
        return 1;
    }
    else if (rxbuf == MCP2515_RXB1 && get_interrupt_flag(handle, RX_BUF1_FULL_INT))
    {
        mcp2515_read_bytes(handle, MCP2515_RXB1SIDH, rx_reg.rx_reg_array, sizeof(rx_reg.rx_reg_array));
        if(rx_reg.RXBnSIDL & 0x08)
        {
            mcp2515_can_msg->id_type = 1;
            mcp2515_can_msg->id = convert_reg_to_exid(rx_reg.RXBnEID8, rx_reg.RXBnEID0, rx_reg.RXBnSIDH, rx_reg.RXBnSIDL);
        }
        else
        {
            mcp2515_can_msg->id_type = 0;
            mcp2515_can_msg->id = convert_reg_to_stid(rx_reg.RXBnSIDH, rx_reg.RXBnSIDL);
        }
        mcp2515_can_msg->dlc = rx_reg.RXBnDLC.DLC;
        memcpy(mcp2515_can_msg->data, rx_reg.RXBnDm, 8);

        clear_interrupt_flag(handle, RX_BUF1_FULL_INT);
        return 1;
    }
    return 0;
}

/* check BUS off */
uint8_t mcp2515_is_bus_off(bsp_mcp2515_t* handle)
{
    uint8_t error = 0;
    ctrl_error_status_t error_status;

    error_status.error_flag_reg = mcp2515_read_byte(handle, MCP2515_EFLG);

    if(error_status.TXBO == 1)
    {
        error = 1;
    }

    return error;
}

/* check Rx Passive Error */
uint8_t mcp2515_is_rx_error(bsp_mcp2515_t* handle)
{
    uint8_t error = 0;
    ctrl_error_status_t error_status;

    error_status.error_flag_reg = mcp2515_read_byte(handle, MCP2515_EFLG);

    if(error_status.RXEP == 1)
    {
        error = 1;
    }

    return error;
}

/* check Tx Passive Error */
uint8_t mcp2515_is_tx_error(bsp_mcp2515_t* handle)
{
    uint8_t error = 0;
    ctrl_error_status_t error_status;

    error_status.error_flag_reg = mcp2515_read_byte(handle, MCP2515_EFLG);

    if(error_status.TXEP == 1)
    {
        error = 1;
    }

    return error;
}

/* Receive Error Count bits */
uint8_t mcp2515_get_rx_error_count(bsp_mcp2515_t* handle)
{
    return mcp2515_read_byte(handle, MCP2515_REC);
}

/* Transmit Error Count bits */
uint8_t mcp2515_get_tx_error_count(bsp_mcp2515_t* handle)
{
    return mcp2515_read_byte(handle, MCP2515_TEC);
}

/*
    mask & filter usage example:
    mcp2515.set_filter_mask(&mcp2515, MCP2515_MASK0, STANDARD_FRAME, 0x7FF);
    mcp2515.set_filter_mask(&mcp2515, MCP2515_MASK1, STANDARD_FRAME, 0x7FF);
    mcp2515.set_filter(&mcp2515, MCP2515_RXF0...MCP2515_RXF5, STANDARD_FRAME, 0x1c1); // only receive STANDARD_FRAME id=0x1c1 
    mcp2515.set_filter(&mcp2515, MCP2515_RXF0, STANDARD_FRAME, 0x1c1);
    mcp2515.set_filter(&mcp2515, MCP2515_RXF1...MCP2515_RXF5, STANDARD_FRAME, 0x1c2); // only receive STANDARD_FRAME id=0x1c1 & 0x1c2
    如果需要接收指定区间的id则根据需要更改mask
*/
/**
 * @brief	
 *  @note   MASK0-->RXB0; MASK1-->RXB1
 * @param	  mask: only one parameter can be selected which is shown as below:
    \arg        MCP2515_MASK0
    \arg        MCP2515_MASK1
 * @param	  ext : only one parameter can be selected which is shown as below:
    \arg        STANDARD_FRAME
    \arg        EXTENDED_FRAME
 * @param   data: bitx(0...28) 1: 该位进行比较; 0: 该位不进行比较(无论filter的位是0还是1都接收)
 * @retval	
 */
uint8_t set_filter_mask(bsp_mcp2515_t* handle, uint8_t mask, uint8_t ide, uint32_t data)
{
    id_reg_t id_reg;
    if(mcp2515_set_config_mode(handle))
        return 1;

    convert_id_to_reg(data, ide, &id_reg);
    switch (mask)
    {
        case MCP2515_MASK0:
            mcp2515_write_bytes(handle, MCP2515_RXM0SIDH, &id_reg.SIDH, 4);
            break;
        case MCP2515_MASK1:
            mcp2515_write_bytes(handle, MCP2515_RXM1SIDH, &id_reg.SIDH, 4);
            break;
        default:
            mcp2515_write_bytes(handle, MCP2515_RXM0SIDH, &id_reg.SIDH, 4);
            break;
    }
    if(mcp2515_set_normal_mode(handle))
        return 1;
    return 0;
}

/**
 * @brief	
 *  @note   RXF0,RXF1-->RXB0; RXF2,RXF3,RXF4,RXF5-->RXB1
 * @param	  rxf: only one parameter can be selected which is shown as below:
    \arg        MCP2515_RXF0 to MCP2515_RXF5
 * @param	  ext: only one parameter can be selected which is shown as below:
    \arg        STANDARD_FRAME
    \arg        EXTENDED_FRAME
 * @retval	
 */
uint8_t set_filter(bsp_mcp2515_t* handle, uint8_t rxf, uint8_t ide, uint32_t data)
{
    id_reg_t id_reg;
    if(mcp2515_set_config_mode(handle))
        return 1;
    convert_id_to_reg(data, ide, &id_reg);
    switch (rxf)
    {
        case MCP2515_RXF0:
            mcp2515_write_bytes(handle, MCP2515_RXF0SIDH, &id_reg.SIDH, 4);
            break;
        case MCP2515_RXF1:
            mcp2515_write_bytes(handle, MCP2515_RXF1SIDH, &id_reg.SIDH, 4);
            break;
        case MCP2515_RXF2:
            mcp2515_write_bytes(handle, MCP2515_RXF2SIDH, &id_reg.SIDH, 4);
            break;
        case MCP2515_RXF3:
            mcp2515_write_bytes(handle, MCP2515_RXF3SIDH, &id_reg.SIDH, 4);
            break;
        case MCP2515_RXF4:
            mcp2515_write_bytes(handle, MCP2515_RXF4SIDH, &id_reg.SIDH, 4);
            break;
        case MCP2515_RXF5:
            mcp2515_write_bytes(handle, MCP2515_RXF5SIDH, &id_reg.SIDH, 4);
            break;
        default:
            mcp2515_write_bytes(handle, MCP2515_RXF0SIDH, &id_reg.SIDH, 4);
            break;
    }
    if(mcp2515_set_normal_mode(handle))
        return 1;
    return 0;
}

static void enable_interrupt(bsp_mcp2515_t* handle, uint8_t interrupt)
{
    switch (interrupt)
    {
        case MESSAGE_ERROR_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x80, 0x80);
            break;
        case WAKEUP_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x40, 0x40);
            break;
        case ERROR_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x20, 0x20);
            break;
        case TX_BUF2_EMPTY_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x10, 0x10);
            break;
        case TX_BUF1_EMPTY_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x08, 0x08);
            break;
        case TX_BUF0_EMPTY_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x04, 0x04);
            break;
        case RX_BUF1_FULL_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x02, 0x02);
            break;
        case RX_BUF0_FULL_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x01, 0x01);
            break;
        default:
            break;
    }
}

static void disable_interrupt(bsp_mcp2515_t* handle, uint8_t interrupt)
{
    switch (interrupt)
    {
        case MESSAGE_ERROR_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x80, 0x00);
            break;
        case WAKEUP_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x40, 0x00);
            break;
        case ERROR_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x20, 0x00);
            break;
        case TX_BUF2_EMPTY_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x10, 0x00);
            break;
        case TX_BUF1_EMPTY_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x08, 0x00);
            break;
        case TX_BUF0_EMPTY_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x04, 0x00);
            break;
        case RX_BUF1_FULL_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x02, 0x00);
            break;
        case RX_BUF0_FULL_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTE, 0x01, 0x00);
            break;
        default:
            break;
    }
}

static uint8_t get_interrupt_flag(bsp_mcp2515_t* handle, uint8_t interrupt)
{
    uint8_t flag_sta;
    ctrl_interrupt_flag_t interrupt_flag;
    interrupt_flag.ctrl_int_flag_reg = mcp2515_read_byte(handle, MCP2515_CANINTF);

    switch (interrupt)
    {
        case MESSAGE_ERROR_INT:
            flag_sta = interrupt_flag.MERRF;
            break;
        case WAKEUP_INT:
            flag_sta = interrupt_flag.WAKIF;
            break;
        case ERROR_INT:
            flag_sta = interrupt_flag.ERRIF;
            break;
        case TX_BUF2_EMPTY_INT:
            flag_sta = interrupt_flag.TX2IF;
            break;
        case TX_BUF1_EMPTY_INT:
            flag_sta = interrupt_flag.TX1IF;
            break;
        case TX_BUF0_EMPTY_INT:
            flag_sta = interrupt_flag.TX0IF;
            break;
        case RX_BUF1_FULL_INT:
            flag_sta = interrupt_flag.RX1IF;
            break;
        case RX_BUF0_FULL_INT:
            flag_sta = interrupt_flag.RX0IF;
            break;
        default:
            flag_sta = 0;
            break;
    }
    return flag_sta;
}

static void clear_interrupt_flag(bsp_mcp2515_t* handle, uint8_t interrupt)
{
    switch (interrupt)
    {
        case MESSAGE_ERROR_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTF, 0x80, 0x00);
            break;
        case WAKEUP_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTF, 0x40, 0x00);
            break;
        case ERROR_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTF, 0x20, 0x00);
            break;
        case TX_BUF2_EMPTY_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTF, 0x10, 0x00);
            break;
        case TX_BUF1_EMPTY_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTF, 0x08, 0x00);
            break;
        case TX_BUF0_EMPTY_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTF, 0x04, 0x00);
            break;
        case RX_BUF1_FULL_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTF, 0x02, 0x00);
            break;
        case RX_BUF0_FULL_INT:
            mcp2515_bit_modify(handle, MCP2515_CANINTF, 0x01, 0x00);
            break;
        default:
            break;
    }
}
