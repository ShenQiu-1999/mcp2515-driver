#ifndef _BSP_MCP2515_H
#define _BSP_MCP2515_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#if defined(__CC_ARM)
#pragma anon_unions
#endif

#define MCP2515_RESET           0xC0    // datasheet P67
#define MCP2515_READ            0x03
#define MCP2515_WRITE           0x02
#define MCP2515_BIT_MODIFY      0x05

#define MCP2515_CANSTAT         0x0E    // datasheet P61
#define MCP2515_CANCTRL         0x0F    // datasheet P60

#define MCP2515_READ_STATUS     0xA0    // datasheet P67
#define MCP2515_RX_STATUS       0xB0

#define MCP2515_TEC             0x1C    // datasheet P49
#define MCP2515_REC             0x1D
#define MCP2515_EFLG            0x2D    // datasheet P50

#define MCP2515_RXB0CTRL        0x60    // datasheet P27
#define MCP2515_RXB1CTRL        0x70

#define MCP2515_RXB0SIDH        0x61    // datasheet P30
#define MCP2515_RXB0D0          0x66
#define MCP2515_RXB1SIDH        0x71
#define MCP2515_RXB1D0          0x76

#define MCP2515_TXB0CTRL        0x30    // datasheet P18
#define MCP2515_TXB1CTRL        0x40
#define MCP2515_TXB2CTRL        0x50

#define MCP2515_TXB0SIDH        0x31    // datasheet P20
#define MCP2515_TXB0DLC         0x35
#define MCP2515_TXB0D0          0x36
#define MCP2515_TXB1SIDH        0x41
#define MCP2515_TXB1DLC         0x45
#define MCP2515_TXB1D0          0x46
#define MCP2515_TXB2SIDH        0x51
#define MCP2515_TXB2DLC         0x55
#define MCP2515_TXB2D0          0x56

#define MCP2515_RTS_TX0         0x81    // datasheet P67
#define MCP2515_RTS_TX1         0x82
#define MCP2515_RTS_TX2         0x84

#define MCP2515_CANINTE         0x2B    // datasheet P53
#define MCP2515_CANINTF         0x2C    // datasheet P54

#define MCP2515_RXF0SIDH        0x00    // datasheet P35
#define MCP2515_RXF1SIDH        0x04
#define MCP2515_RXF2SIDH        0x08
#define MCP2515_RXF3SIDH        0x10
#define MCP2515_RXF4SIDH        0x14
#define MCP2515_RXF5SIDH        0x18

#define MCP2515_RXM0SIDH        0x20    // datasheet P37
#define MCP2515_RXM1SIDH        0x24

#define MCP2515_CNF1            0x2A    // datasheet P44
#define MCP2515_CNF2            0x29
#define MCP2515_CNF3            0x28

/* internal use */
#define MESSAGE_IN_RXB0         0x01
#define MESSAGE_IN_RXB1         0x02
#define MESSAGE_IN_BOTH         0x03

#define STANDARD_DATA_FRAME     0x00
#define STANDARD_REMOTE_FRAME   0x01
#define EXTENDED_DATA_FRAME     0x02
#define EXTENDED_REMOTE_FRAME   0x03

/* user params */
#define MCP2515_CAN_250K        0x00
#define MCP2515_CAN_500K        0x01

#define MCP2515_OSC_8M          0x00
#define MCP2515_OSC_16M         0x01
#define MCP2515_OSC_20M         0x02

#define STANDARD_FRAME          0x00
#define EXTENDED_FRAME          0x01

#define MCP2515_RXB0            0x00
#define MCP2515_RXB1            0x01

#define MCP2515_MASK0           0x00
#define MCP2515_MASK1           0x01
#define MCP2515_RXF0            0x00
#define MCP2515_RXF1            0x01
#define MCP2515_RXF2            0x02
#define MCP2515_RXF3            0x03
#define MCP2515_RXF4            0x04
#define MCP2515_RXF5            0x05

typedef union {
    struct{
        uint8_t RX0IF      : 1;
        uint8_t RX1IF      : 1;
        uint8_t TX0IF      : 1;
        uint8_t TX1IF      : 1;
        uint8_t TX2IF      : 1;
        uint8_t ERRIF      : 1;
        uint8_t WAKIF      : 1;
        uint8_t MERRF      : 1;
    };
    uint8_t ctrl_int_flag_reg;
} ctrl_interrupt_flag_t;

typedef union {
    struct{
        uint8_t RX0IF      : 1;
        uint8_t RX1IF      : 1;
        uint8_t TXB0REQ    : 1;
        uint8_t TX0IF      : 1;
        uint8_t TXB1REQ    : 1;
        uint8_t TX1IF      : 1;
        uint8_t TXB2REQ    : 1;
        uint8_t TX2IF      : 1;
    };
    uint8_t ctrl_status;
} ctrl_status_t; // MCP2515_READ_STATUS  datasheet P70

typedef union {
    struct{
        uint8_t filter     : 3; // datasheet P70
        uint8_t msg_type   : 2; // 0: Standard data frame; 2: Extended data frame
        // uint8_t rtr        : 1; // 0: data frame; 1: remote frame TO-DO: 没用到远程帧,未实现发送接收的判断
        // uint8_t ide        : 1; // 0: Standard frame; 1: Extended frame
        uint8_t            : 1;
        uint8_t rx_buffer  : 2; // 0: No RX message; 1: Message in RXB0; 2: Message in RXB1; 3: Messages in both buffers
    };
    uint8_t ctrl_rx_status;
} ctrl_rx_status_t; // MCP2515_RX_STATUS

typedef union {
    struct{
        uint8_t EWARN      : 1;
        uint8_t RXWAR      : 1;
        uint8_t TXWAR      : 1;
        uint8_t RXEP       : 1;
        uint8_t TXEP       : 1;
        uint8_t TXBO       : 1;
        uint8_t RX0OVR     : 1;
        uint8_t RX1OVR     : 1;  
    };
    uint8_t error_flag_reg;
} ctrl_error_status_t; // MCP2515_ERROR_FLAG  datasheet P50

typedef union {
    struct{
        uint8_t RXBnSIDH;
        uint8_t RXBnSIDL;
        uint8_t RXBnEID8;
        uint8_t RXBnEID0;
        // uint8_t RXBnDLC;
        union {
            struct {
                uint8_t DLC : 4;
                uint8_t     : 2;
                uint8_t RTR : 1;
                uint8_t     : 1;
            };
            // uint8_t RXBnDLC;
        } RXBnDLC;
        uint8_t RXBnDm[8];
    };
    uint8_t rx_reg_array[13];
} rx_reg_t; // datasheet P30

typedef struct {
    uint8_t SIDH;
    uint8_t SIDL;
    uint8_t EID8;
    uint8_t EID0;
} id_reg_t;

typedef struct {
    uint8_t RXFnSIDH;
    uint8_t RXFnSIDL;
    uint8_t RXFnEID8;
    uint8_t RXFnEID0;
} RXFn; // datasheet P35

typedef struct{
    uint8_t RXMnSIDH;
    uint8_t RXMnSIDL;
    uint8_t RXMnEID8;
    uint8_t RXMnEID0;
} RXMn;

typedef enum {
    MESSAGE_ERROR_INT,
    WAKEUP_INT,
    ERROR_INT,
    TX_BUF2_EMPTY_INT,
    TX_BUF1_EMPTY_INT,
    TX_BUF0_EMPTY_INT,
    RX_BUF1_FULL_INT,
    RX_BUF0_FULL_INT,
} mcp2515_interrupt_t;

typedef enum {
    CLKOUT_DIV1,
    CLKOUT_DIV2,
    CLKOUT_DIV4,
    CLKOUT_DIV8,
    CLKOUT_DISABLE,
} mcp2515_clkout_t;

typedef struct
{
    uint8_t id_type;    // 0: STANDARD_FRAME; 1: EXTENDED_FRAME
    uint32_t id;
    // uint8_t ide;
    // uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];
} mcp2515_can_msg_t;

typedef struct 
{
    void (*spi_init)(void);
    void (*spi_cs)(uint8_t);
    uint8_t (*spi_read_write_byte)(uint8_t);
} mcp2515_spi_interface_t;

typedef struct bsp_mcp2515_tag bsp_mcp2515_t;

struct bsp_mcp2515_tag
{
    /* private: */
    mcp2515_spi_interface_t spi_interface;
    uint32_t osc_freq; // 8/16/20MHz

    /* public: */
    uint8_t (*reset)(bsp_mcp2515_t* handle);
    uint8_t (*set_baudrate)(bsp_mcp2515_t* handle, uint32_t baudrate); // baudrate: macro definition MCP2515_CAN_250K/MCP2515_CAN_500K
    uint8_t (*transmit)(bsp_mcp2515_t* handle, mcp2515_can_msg_t* mcp2515_can_msg);
    uint8_t (*receive)(bsp_mcp2515_t* handle, mcp2515_can_msg_t* mcp2515_can_msg);
    uint8_t (*receive_isr)(bsp_mcp2515_t* handle, uint8_t rxbuf, mcp2515_can_msg_t* mcp2515_can_msg);
    uint8_t (*is_bus_off)(bsp_mcp2515_t* handle);
    uint8_t (*is_rx_error)(bsp_mcp2515_t* handle);
    uint8_t (*is_tx_error)(bsp_mcp2515_t* handle);
    uint8_t (*get_rx_error_count)(bsp_mcp2515_t* handle);
    uint8_t (*get_tx_error_count)(bsp_mcp2515_t* handle);
    uint8_t (*set_filter_mask)(bsp_mcp2515_t* handle, uint8_t mask, uint8_t ide, uint32_t data); // ide: macro definition STANDARD_FRAME/EXTENDED_FRAME
    uint8_t (*set_filter)(bsp_mcp2515_t* handle, uint8_t rxf, uint8_t ide, uint32_t data);
    uint8_t (*enter_sleep_mode)(bsp_mcp2515_t* handle); // datasheet P59
    uint8_t (*set_loopback_mode)(bsp_mcp2515_t* handle);
    uint8_t (*set_listenonly_mode)(bsp_mcp2515_t* handle);
    void (*enable_interrupt)(bsp_mcp2515_t* handle, uint8_t interrupt); // interrupt: enum definition mcp2515_interrupt_t
    void (*disable_interrupt)(bsp_mcp2515_t* handle, uint8_t interrupt);
    uint8_t (*get_interrupt_flag)(bsp_mcp2515_t* handle, uint8_t interrupt);
    void (*clear_interrupt_flag)(bsp_mcp2515_t* handle, uint8_t interrupt);
};


uint8_t mcp2515_init(bsp_mcp2515_t* handle,
                     uint32_t osc_freq,         // macro definition MCP2515_OSC_8M/MCP2515_OSC_16M/MCP2515_OSC_20M
                     void (*spi_init)(void),
                     void (*spi_cs)(uint8_t),
                     uint8_t (*spi_read_write_byte)(uint8_t));


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _BSP_MCP2515_H */
