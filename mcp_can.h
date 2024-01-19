/**
 * @file mcp_can.h
 * @brief MCP2515 header file.
 * @author Halit Cetin (halitcetin@live.com)
 * @version 0.1
 * @date 19-01-2024
 *
 * @copyright Copyright (c) 2024
 */

#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"

#define MAX_CHAR_IN_MESSAGE 8

/**
 * @class MCP_CAN
 * @brief MCP2515 CAN Controller Class
 */
class MCP_CAN
{
private:
    INT8U m_nExtFlg;                   /**< Identifier Type: Extended (29 bit) or Standard (11 bit) */
    INT32U m_nID;                      /**< CAN ID */
    INT8U m_nDlc;                      /**< Data Length Code */
    INT8U m_nDta[MAX_CHAR_IN_MESSAGE]; /**< Data array */
    INT8U m_nRtr;                      /**< Remote request flag */
    INT8U m_nfilhit;                   /**< The number of the filter that matched the message */
    SPI *mcpSPI;                       /**< The SPI-Device used */
    DigitalOut *MCPCS;                 /**< Chip Select pin number */
    INT8U mcpMode;                     /**< Mode to return to after configurations are performed. */

private:
    /**
     * @brief Soft Reset MCP2515
     */
    void mcp2515_reset(void);

    /**
     * @brief Read MCP2515 register
     * @param address Register address
     * @return Register value
     */
    INT8U mcp2515_readRegister(const INT8U address);

    /**
     * @brief Read MCP2515 successive registers
     * @param address Start address
     * @param values Array to store register values
     * @param n Number of registers to read
     */
    void mcp2515_readRegisterS(const INT8U address, INT8U values[], const INT8U n);

    /**
     * @brief Set MCP2515 register
     * @param address Register address
     * @param value Value to set
     */
    void mcp2515_setRegister(const INT8U address, const INT8U value);

    /**
     * @brief Set MCP2515 successive registers
     * @param address Start address
     * @param values Array of values to set
     * @param n Number of registers to set
     */
    void mcp2515_setRegisterS(const INT8U address, const INT8U values[], const INT8U n);

    /**
     * @brief Initialize CAN buffers
     */
    void mcp2515_initCANBuffers(void);

    /**
     * @brief Set specific bit(s) of a register
     * @param address Register address
     * @param mask Bitmask to modify
     * @param data Data to set
     */
    void mcp2515_modifyRegister(const INT8U address, const INT8U mask, const INT8U data);

    /**
     * @brief Read MCP2515 Status
     * @return Status value
     */
    INT8U mcp2515_readStatus(void);

    /**
     * @brief Set CAN control mode
     * @param newmode New mode to set
     * @return Result code
     */
    INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode);

    /**
     * @brief Set control mode
     * @param newmode New mode to set
     * @return Result code
     */
    INT8U mcp2515_requestNewMode(const INT8U newmode);

    /**
     * @brief Set baudrate
     * @param canSpeed CAN speed
     * @param canClock CAN clock
     * @return Result code
     */
    INT8U mcp2515_configRate(const INT8U canSpeed, const INT8U canClock);

    /**
     * @brief Initialize controller
     * @param canIDMode CAN ID mode
     * @param canSpeed CAN speed
     * @param canClock CAN clock
     * @return Result code
     */
    INT8U mcp2515_init(const INT8U canIDMode, const INT8U canSpeed, const INT8U canClock);

    /**
     * @brief Write CAN Mask or Filter
     * @param mcp_addr Address
     * @param ext Extended flag
     * @param id CAN ID
     */
    void mcp2515_write_mf(const INT8U mcp_addr, const INT8U ext, const INT32U id);

    /**
     * @brief Write CAN ID
     * @param mcp_addr Address
     * @param ext Extended flag
     * @param id CAN ID
     */
    void mcp2515_write_id(const INT8U mcp_addr, const INT8U ext, const INT32U id);

    /**
     * @brief Read CAN ID
     * @param mcp_addr Address
     * @param ext Pointer to store Extended flag
     * @param id Pointer to store CAN ID
     */
    void mcp2515_read_id(const INT8U mcp_addr, INT8U *ext, INT32U *id);

    /**
     * @brief Write CAN message
     * @param buffer_sidh_addr Buffer SIDH address
     */
    void mcp2515_write_canMsg(const INT8U buffer_sidh_addr);

    /**
     * @brief Read CAN message
     * @param buffer_sidh_addr Buffer SIDH address
     */
    void mcp2515_read_canMsg(const INT8U buffer_sidh_addr);

    /**
     * @brief Find empty transmit buffer
     * @param txbuf_n Pointer to store transmit buffer number
     * @return Result code
     */
    INT8U mcp2515_getNextFreeTXBuf(INT8U *txbuf_n);

    /**
     * @brief Set message
     * @param id CAN ID
     * @param rtr Remote request flag
     * @param ext Extended flag
     * @param len Data length code
     * @param pData Pointer to data array
     * @return Result code
     */
    INT8U setMsg(INT32U id, INT8U rtr, INT8U ext, INT8U len, INT8U *pData);

    /**
     * @brief Clear all message to zero
     * @return Result code
     */
    INT8U clearMsg();

    /**
     * @brief Read message
     * @return Result code
     */
    INT8U readMsg();

    /**
     * @brief Send message
     * @return Result code
     */
    INT8U sendMsg();

public:
    /**
     * @brief Constructor
     * @param _SPI Pointer to SPI device
     * @param _CS Pointer to Chip Select pin
     */
    MCP_CAN(SPI *_SPI, DigitalOut *_CS);

    /**
     * @brief Initialize controller parameters
     * @param idmodeset ID mode
     * @param speedset CAN speed
     * @param clockset CAN clock
     * @return Result code
     */
    INT8U begin(INT8U idmodeset, INT8U speedset, INT8U clockset);

    /**
     * @brief Initialize Mask(s)
     * @param num Mask number
     * @param ext Extended flag
     * @param ulData Data for initialization
     * @return Result code
     */
    INT8U init_Mask(INT8U num, INT8U ext, INT32U ulData);

    /**
     * @brief Initialize Mask(s)
     * @param num Mask number
     * @param ulData Data for initialization
     * @return Result code
     */
    INT8U init_Mask(INT8U num, INT32U ulData);

    /**
     * @brief Initialize Filter(s)
     * @param num Filter number
     * @param ext Extended flag
     * @param ulData Data for initialization
     * @return Result code
     */
    INT8U init_Filt(INT8U num, INT8U ext, INT32U ulData);

    /**
     * @brief Initialize Filter(s)
     * @param num Filter number
     * @param ulData Data for initialization
     * @return Result code
     */
    INT8U init_Filt(INT8U num, INT32U ulData);

    /**
     * @brief Enable or disable the wake up interrupt
     * @param enable Enable (1) or disable (0)
     */
    void setSleepWakeup(INT8U enable);

    /**
     * @brief Set operational mode
     * @param opMode Operational mode
     * @return Result code
     */
    INT8U setMode(INT8U opMode);

    /**
     * @brief Send message to transmit buffer
     * @param id CAN ID
     * @param ext Extended flag
     * @param len Data length code
     * @param buf Pointer to data buffer
     * @return Result code
     */
    INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);

    /**
     * @brief Send message to transmit buffer
     * @param id CAN ID
     * @param len Data length code
     * @param buf Pointer to data buffer
     * @return Result code
     */
    INT8U sendMsgBuf(INT32U id, INT8U len, INT8U *buf);

    /**
     * @brief Read message from receive buffer
     * @param id Pointer to store CAN ID
     * @param ext Pointer to store Extended flag
     * @param len Pointer to store Data length code
     * @param buf Pointer to store data buffer
     * @return Result code
     */
    INT8U readMsgBuf(INT32U *id, INT8U *ext, INT8U *len, INT8U *buf);

    /**
     * @brief Read message from receive buffer
     * @param id Pointer to store CAN ID
     * @param len Pointer to store Data length code
     * @param buf Pointer to store data buffer
     * @return Result code
     */
    INT8U readMsgBuf(INT32U *id, INT8U *len, INT8U *buf);

    /**
     * @brief Check for received data
     * @return Result code
     */
    INT8U checkReceive(void);

    /**
     * @brief Check for errors
     * @return Result code
     */
    INT8U checkError(void);

    /**
     * @brief Get error status
     * @return Result code
     */
    INT8U getError(void);

    /**
     * @brief Get error count for received messages
     * @return Error count
     */
    INT8U errorCountRX(void);

    /**
     * @brief Get error count for transmitted messages
     * @return Error count
     */
    INT8U errorCountTX(void);

    /**
     * @brief Enable one-shot transmission
     * @return Result code
     */
    INT8U enOneShotTX(void);

    /**
     * @brief Disable one-shot transmission
     * @return Result code
     */
    INT8U disOneShotTX(void);

    /**
     * @brief Abort queued transmission(s)
     * @return Result code
     */
    INT8U abortTX(void);

    /**
     * @brief Sets GPO
     * @param data Data to set
     * @return Result code
     */
    INT8U setGPO(INT8U data);

    /**
     * @brief Reads GPI
     * @return GPI value
     */
    INT8U getGPI(void);
};

#endif
