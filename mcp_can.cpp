/**
 * @file mcp_can.cpp
 * @author Halit Cetin (halitcetin@live.com)
 * @brief MCP 2515 driver implementation file.
 * @version 0.1
 * @date 19-01-2024
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "mcp_can.h"

#define spi_readwrite mcpSPI->write
#define spi_read() spi_readwrite(0x00)

void MCP_CAN::mcp2515_reset(void)
{
    mcpSPI->lock();
    MCP2515_SELECT();
    spi_readwrite(MCP_RESET);
    MCP2515_UNSELECT();
    mcpSPI->unlock();
    ThisThread::sleep_for(5s); // If the MCP2515 was in sleep mode when the reset command was issued then we need to wait a while for it to reset properly
}

INT8U MCP_CAN::mcp2515_readRegister(const INT8U address)
{
    INT8U ret;

    mcpSPI->lock();
    MCP2515_SELECT();
    spi_readwrite(MCP_READ);
    spi_readwrite(address);
    ret = spi_read();
    MCP2515_UNSELECT();
    mcpSPI->unlock();

    return ret;
}

void MCP_CAN::mcp2515_readRegisterS(const INT8U address, INT8U values[], const INT8U n)
{
    INT8U i;
    mcpSPI->lock();
    MCP2515_SELECT();
    spi_readwrite(MCP_READ);
    spi_readwrite(address);
    // mcp2515 has auto-increment of address-pointer
    for (i = 0; i < n; i++)
        values[i] = spi_read();

    MCP2515_UNSELECT();
    mcpSPI->unlock();
}

void MCP_CAN::mcp2515_setRegister(const INT8U address, const INT8U value)
{
    mcpSPI->lock();
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
    spi_readwrite(value);
    MCP2515_UNSELECT();
    mcpSPI->unlock();
}

void MCP_CAN::mcp2515_setRegisterS(const INT8U address, const INT8U values[], const INT8U n)
{
    INT8U i;
    mcpSPI->lock();
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);

    for (i = 0; i < n; i++)
        spi_readwrite(values[i]);

    MCP2515_UNSELECT();
    mcpSPI->unlock();
}

void MCP_CAN::mcp2515_modifyRegister(const INT8U address, const INT8U mask, const INT8U data)
{
    mcpSPI->lock();
    MCP2515_SELECT();
    spi_readwrite(MCP_BITMOD);
    spi_readwrite(address);
    spi_readwrite(mask);
    spi_readwrite(data);
    MCP2515_UNSELECT();
    mcpSPI->unlock();
}

INT8U MCP_CAN::mcp2515_readStatus(void)
{
    INT8U i;
    mcpSPI->lock();
    MCP2515_SELECT();
    spi_readwrite(MCP_READ_STATUS);
    i = spi_read();
    MCP2515_UNSELECT();
    mcpSPI->unlock();
    return i;
}

void MCP_CAN::setSleepWakeup(const INT8U enable)
{
    mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, enable ? MCP_WAKIF : 0);
}

INT8U MCP_CAN::setMode(const INT8U opMode)
{
    mcpMode = opMode;
    return mcp2515_setCANCTRL_Mode(mcpMode);
}

INT8U MCP_CAN::mcp2515_setCANCTRL_Mode(const INT8U newmode)
{
    // If the chip is asleep and we want to change mode then a manual wake needs to be done
    // This is done by setting the wake up interrupt flag
    // This undocumented trick was found at https://github.com/mkleemann/can/blob/master/can_sleep_mcp2515.c
    if ((mcp2515_readRegister(MCP_CANSTAT) & MODE_MASK) == MCP_SLEEP && newmode != MCP_SLEEP)
    {
        // Make sure wake interrupt is enabled
        uint8_t wakeIntEnabled = (mcp2515_readRegister(MCP_CANINTE) & MCP_WAKIF);
        if (!wakeIntEnabled)
            mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, MCP_WAKIF);

        // Set wake flag (this does the actual waking up)
        mcp2515_modifyRegister(MCP_CANINTF, MCP_WAKIF, MCP_WAKIF);

        // Wait for the chip to exit SLEEP and enter LISTENONLY mode.

        // If the chip is not connected to a CAN bus (or the bus has no other powered nodes) it will sometimes trigger the wake interrupt as soon
        // as it's put to sleep, but it will stay in SLEEP mode instead of automatically switching to LISTENONLY mode.
        // In this situation the mode needs to be manually set to LISTENONLY.

        if (mcp2515_requestNewMode(MCP_LISTENONLY) != MCP2515_OK)
            return MCP2515_FAIL;

        // Turn wake interrupt back off if it was originally off
        if (!wakeIntEnabled)
            mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, 0);
    }

    // Clear wake flag
    mcp2515_modifyRegister(MCP_CANINTF, MCP_WAKIF, 0);

    return mcp2515_requestNewMode(newmode);
}

INT8U MCP_CAN::mcp2515_requestNewMode(const INT8U newmode)
{
    Timer timer;
    timer.start(); // Start the timer

    // Spam new mode request and wait for the operation to complete
    while (1)
    {
        // Request new mode
        // This is inside the loop as sometimes requesting the new mode once doesn't work (usually when attempting to sleep)
        mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);

        uint8_t statReg = mcp2515_readRegister(MCP_CANSTAT);
        if ((statReg & MODE_MASK) == newmode) // We're now in the new mode
            return MCP2515_OK;
        else if (timer.elapsed_time().count() > 200000) // Wait no more than 200ms for the operation to complete
            return MCP2515_FAIL;
    }
}

INT8U MCP_CAN::mcp2515_configRate(const INT8U canSpeed, const INT8U canClock)
{
    INT8U set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock & MCP_CLOCK_SELECT)
    {
    case (MCP_8MHZ):
        switch (canSpeed)
        {
        case (CAN_5KBPS): //   5KBPS
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

        case (CAN_10KBPS): //  10KBPS
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

        case (CAN_20KBPS): //  20KBPS
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

        case (CAN_31K25BPS): //  31.25KBPS
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

        case (CAN_33K3BPS): //  33.33KBPS
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

        case (CAN_40KBPS): //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

        case (CAN_50KBPS): //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

        case (CAN_80KBPS): //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

        case (CAN_100KBPS): // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

        case (CAN_125KBPS): // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

        case (CAN_200KBPS): // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

        case (CAN_250KBPS): // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

        case (CAN_500KBPS): // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;

        case (CAN_1000KBPS): //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;

        default:
            set = 0;
            return MCP2515_FAIL;
            break;
        }
        break;

    case (MCP_16MHZ):
        switch (canSpeed)
        {
        case (CAN_5KBPS): //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

        case (CAN_10KBPS): //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

        case (CAN_20KBPS): //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

        case (CAN_33K3BPS): //  20Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

        case (CAN_40KBPS): //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

        case (CAN_50KBPS): //  50Kbps
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

        case (CAN_80KBPS): //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

        case (CAN_100KBPS): // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

        case (CAN_125KBPS): // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

        case (CAN_200KBPS): // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

        case (CAN_250KBPS): // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

        case (CAN_500KBPS): // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;

        case (CAN_1000KBPS): //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;

        default:
            set = 0;
            return MCP2515_FAIL;
            break;
        }
        break;

    case (MCP_20MHZ):
        switch (canSpeed)
        {
        case (CAN_40KBPS): //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

        case (CAN_50KBPS): //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

        case (CAN_80KBPS): //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

        case (CAN_100KBPS): // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

        case (CAN_125KBPS): // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

        case (CAN_200KBPS): // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

        case (CAN_250KBPS): // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

        case (CAN_500KBPS): // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;

        case (CAN_1000KBPS): //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;

        default:
            set = 0;
            return MCP2515_FAIL;
            break;
        }
        break;

    default:
        set = 0;
        return MCP2515_FAIL;
        break;
    }

    if (canClock & MCP_CLKOUT_ENABLE)
    {
        cfg3 &= (~SOF_ENABLE);
    }

    if (set)
    {
        mcp2515_setRegister(MCP_CNF1, cfg1);
        mcp2515_setRegister(MCP_CNF2, cfg2);
        mcp2515_setRegister(MCP_CNF3, cfg3);
        return MCP2515_OK;
    }

    return MCP2515_FAIL;
}

void MCP_CAN::mcp2515_initCANBuffers(void)
{
    INT8U i, a1, a2, a3;

    INT8U std = 0;
    INT8U ext = 1;
    INT32U ulMask = 0x00, ulFilt = 0x00;

    mcp2515_write_mf(MCP_RXM0SIDH, ext, ulMask); /*Set both masks to 0           */
    mcp2515_write_mf(MCP_RXM1SIDH, ext, ulMask); /*Mask register ignores ext bit */

    /* Set all filters to 0         */
    mcp2515_write_mf(MCP_RXF0SIDH, ext, ulFilt); /* RXB0: extended               */
    mcp2515_write_mf(MCP_RXF1SIDH, std, ulFilt); /* RXB1: standard               */
    mcp2515_write_mf(MCP_RXF2SIDH, ext, ulFilt); /* RXB2: extended               */
    mcp2515_write_mf(MCP_RXF3SIDH, std, ulFilt); /* RXB3: standard               */
    mcp2515_write_mf(MCP_RXF4SIDH, ext, ulFilt);
    mcp2515_write_mf(MCP_RXF5SIDH, std, ulFilt);

    /* Clear, deactivate the three  */
    /* transmit buffers             */
    /* TXBnCTRL -> TXBnD7           */
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++)
    { /* in-buffer loop               */
        mcp2515_setRegister(a1, 0);
        mcp2515_setRegister(a2, 0);
        mcp2515_setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcp2515_setRegister(MCP_RXB0CTRL, 0);
    mcp2515_setRegister(MCP_RXB1CTRL, 0);
}

INT8U MCP_CAN::mcp2515_init(const INT8U canIDMode, const INT8U canSpeed, const INT8U canClock)
{

    INT8U res;

    mcp2515_reset();

    mcpMode = MCP_LOOPBACK;

    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if (res > 0)
    {
#if DEBUG_MODE
        printf("Entering Configuration Mode Failure... \n");
#endif
        return res;
    }
#if DEBUG_MODE
    printf("Entering Configuration Mode Successful! \n");
#endif

    // Set Baudrate
    if (mcp2515_configRate(canSpeed, canClock))
    {
#if DEBUG_MODE
        printf("Setting Baudrate Failure... \n");
#endif
        return res;
    }
#if DEBUG_MODE
    printf("Setting Baudrate Successful! \n");
#endif

    if (res == MCP2515_OK)
    {

        /* init canbuffers              */
        mcp2515_initCANBuffers();

        /* interrupt mode               */
        mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

        // Sets BF pins as GPO
        mcp2515_setRegister(MCP_BFPCTRL, MCP_BxBFS_MASK | MCP_BxBFE_MASK);
        // Sets RTS pins as GPI
        mcp2515_setRegister(MCP_TXRTSCTRL, 0x00);

        switch (canIDMode)
        {
        case (MCP_ANY):
            mcp2515_modifyRegister(MCP_RXB0CTRL,
                                   MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                                   MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
            mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                                   MCP_RXB_RX_ANY);
            break;
            /*          The followingn two functions of the MCP2515 do not work, there is a bug in the silicon.
                        case (MCP_STD):
                        mcp2515_modifyRegister(MCP_RXB0CTRL,
                        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                        MCP_RXB_RX_STD | MCP_RXB_BUKT_MASK );
                        mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                        MCP_RXB_RX_STD);
                        break;

                        case (MCP_EXT):
                        mcp2515_modifyRegister(MCP_RXB0CTRL,
                        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                        MCP_RXB_RX_EXT | MCP_RXB_BUKT_MASK );
                        mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                        MCP_RXB_RX_EXT);
                        break;
            */
        case (MCP_STDEXT):
            mcp2515_modifyRegister(MCP_RXB0CTRL,
                                   MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                                   MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK);
            mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                                   MCP_RXB_RX_STDEXT);
            break;

        default:
#if DEBUG_MODE
            printf("`Setting ID Mode Failure... \n");
#endif
            return MCP2515_FAIL;
            break;
        }

        res = mcp2515_setCANCTRL_Mode(mcpMode);
        if (res)
        {
#if DEBUG_MODE
            printf("Returning to Previous Mode Failure... \n");
#endif
            return res;
        }
    }
    return res;
}

void MCP_CAN::mcp2515_write_id(const INT8U mcp_addr, const INT8U ext, const INT32U id)
{
    uint16_t canid;
    INT8U tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if (ext == 1)
    {
        tbufdata[MCP_EID0] = (INT8U)(canid & 0xFF);
        tbufdata[MCP_EID8] = (INT8U)(canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (INT8U)(canid & 0x03);
        tbufdata[MCP_SIDL] += (INT8U)((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (INT8U)(canid >> 5);
    }
    else
    {
        tbufdata[MCP_SIDH] = (INT8U)(canid >> 3);
        tbufdata[MCP_SIDL] = (INT8U)((canid & 0x07) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }

    mcp2515_setRegisterS(mcp_addr, tbufdata, 4);
}

void MCP_CAN::mcp2515_write_mf(const INT8U mcp_addr, const INT8U ext, const INT32U id)
{
    uint16_t canid;
    INT8U tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if (ext == 1)
    {
        tbufdata[MCP_EID0] = (INT8U)(canid & 0xFF);
        tbufdata[MCP_EID8] = (INT8U)(canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (INT8U)(canid & 0x03);
        tbufdata[MCP_SIDL] += (INT8U)((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (INT8U)(canid >> 5);
    }
    else
    {
        tbufdata[MCP_EID0] = (INT8U)(canid & 0xFF);
        tbufdata[MCP_EID8] = (INT8U)(canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (INT8U)((canid & 0x07) << 5);
        tbufdata[MCP_SIDH] = (INT8U)(canid >> 3);
    }

    mcp2515_setRegisterS(mcp_addr, tbufdata, 4);
}

void MCP_CAN::mcp2515_read_id(const INT8U mcp_addr, INT8U *ext, INT32U *id)
{
    INT8U tbufdata[4];

    *ext = 0;
    *id = 0;

    mcp2515_readRegisterS(mcp_addr, tbufdata, 4);

    *id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

    if ((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) == MCP_TXB_EXIDE_M)
    {
        /* extended id                  */
        *id = (*id << 2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id << 8) + tbufdata[MCP_EID8];
        *id = (*id << 8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

void MCP_CAN::mcp2515_write_canMsg(const INT8U buffer_sidh_addr)
{
    INT8U mcp_addr;
    mcp_addr = buffer_sidh_addr;
    mcp2515_setRegisterS(mcp_addr + 5, m_nDta, m_nDlc); /* write data bytes             */

    if (m_nRtr == 1) /* if RTR set bit in uint8_t       */
        m_nDlc |= MCP_RTR_MASK;

    mcp2515_setRegister((mcp_addr + 4), m_nDlc);  /* write the RTR and DLC        */
    mcp2515_write_id(mcp_addr, m_nExtFlg, m_nID); /* write CAN id                 */
}

void MCP_CAN::mcp2515_read_canMsg(const INT8U buffer_sidh_addr) /* read can msg                 */
{
    INT8U mcp_addr, ctrl;

    mcp_addr = buffer_sidh_addr;

    mcp2515_read_id(mcp_addr, &m_nExtFlg, &m_nID);

    ctrl = mcp2515_readRegister(mcp_addr - 1);
    m_nDlc = mcp2515_readRegister(mcp_addr + 4);

    if (ctrl & 0x08)
        m_nRtr = 1;
    else
        m_nRtr = 0;

    m_nDlc &= MCP_DLC_MASK;
    mcp2515_readRegisterS(mcp_addr + 5, &(m_nDta[0]), m_nDlc);
}

INT8U MCP_CAN::mcp2515_getNextFreeTXBuf(INT8U *txbuf_n) /* get Next free txbuf          */
{
    INT8U res, i, ctrlval;
    INT8U ctrlregs[MCP_N_TXBUFFERS] = {MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL};

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

    /* check all 3 TX-Buffers       */
    for (i = 0; i < MCP_N_TXBUFFERS; i++)
    {
        ctrlval = mcp2515_readRegister(ctrlregs[i]);
        if ((ctrlval & MCP_TXB_TXREQ_M) == 0)
        {
            *txbuf_n = ctrlregs[i] + 1; /* return SIDH-address of Buffer*/

            res = MCP2515_OK;
            return res; /* ! function exit              */
        }
    }
    return res;
}

MCP_CAN::MCP_CAN(SPI *_SPI, DigitalOut *_CS)
{
    mcpSPI = _SPI;
    MCPCS = _CS;

    MCPCS->write(1); // Assuming you want to start with CS high
    MCP2515_UNSELECT();
    mcpSPI->format(8, 0);        // Set SPI format to 8 bits per frame, SPI mode 0
    mcpSPI->frequency(10000000); // Set SPI frequency to 10 MHz (adjust as needed)
}

INT8U MCP_CAN::begin(INT8U idmodeset, INT8U speedset, INT8U clockset)
{
    INT8U res;

    res = mcp2515_init(idmodeset, speedset, clockset);
    if (res == MCP2515_OK)
        return CAN_OK;

    return CAN_FAILINIT;
}

INT8U MCP_CAN::init_Mask(INT8U num, INT8U ext, INT32U ulData)
{
    INT8U res = MCP2515_OK;
#if DEBUG_MODE
    printf("Starting to Set Mask! \n");
#endif
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if (res > 0)
    {
#if DEBUG_MODE
        printf("Entering Configuration Mode Failure... \n");
#endif
        return res;
    }

    if (num == 0)
    {
        mcp2515_write_mf(MCP_RXM0SIDH, ext, ulData);
    }
    else if (num == 1)
    {
        mcp2515_write_mf(MCP_RXM1SIDH, ext, ulData);
    }
    else
        res = MCP2515_FAIL;

    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if (res > 0)
    {
#if DEBUG_MODE
        printf("Entering Previous Mode Failure... \n");
        printf("Setting Mask Failure... \n");
#endif
        return res;
    }
#if DEBUG_MODE
    printf("Setting Mask Successful! \n");
#endif
    return res;
}

INT8U MCP_CAN::init_Mask(INT8U num, INT32U ulData)
{
    INT8U res = MCP2515_OK;
    INT8U ext = 0;
#if DEBUG_MODE
    printf("Starting to Set Mask! \n");
#endif
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if (res > 0)
    {
#if DEBUG_MODE
        printf("Entering Configuration Mode Failure... \n");
#endif
        return res;
    }

    if ((ulData & 0x80000000) == 0x80000000)
        ext = 1;

    if (num == 0)
    {
        mcp2515_write_mf(MCP_RXM0SIDH, ext, ulData);
    }
    else if (num == 1)
    {
        mcp2515_write_mf(MCP_RXM1SIDH, ext, ulData);
    }
    else
        res = MCP2515_FAIL;

    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if (res > 0)
    {
#if DEBUG_MODE
        printf("Entering Previous Mode Failure... \n");
        printf("Setting Mask Failure... \n");
#endif
        return res;
    }
#if DEBUG_MODE
    printf("Setting Mask Successful! \n");
#endif
    return res;
}

INT8U MCP_CAN::init_Filt(INT8U num, INT8U ext, INT32U ulData)
{
    INT8U res = MCP2515_OK;
#if DEBUG_MODE
    printf("Starting to Set Filter! \n");
#endif
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if (res > 0)
    {
#if DEBUG_MODE
        printf("Enter Configuration Mode Failure... \n");
#endif
        return res;
    }

    switch (num)
    {
    case 0:
        mcp2515_write_mf(MCP_RXF0SIDH, ext, ulData);
        break;

    case 1:
        mcp2515_write_mf(MCP_RXF1SIDH, ext, ulData);
        break;

    case 2:
        mcp2515_write_mf(MCP_RXF2SIDH, ext, ulData);
        break;

    case 3:
        mcp2515_write_mf(MCP_RXF3SIDH, ext, ulData);
        break;

    case 4:
        mcp2515_write_mf(MCP_RXF4SIDH, ext, ulData);
        break;

    case 5:
        mcp2515_write_mf(MCP_RXF5SIDH, ext, ulData);
        break;

    default:
        res = MCP2515_FAIL;
    }

    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if (res > 0)
    {
#if DEBUG_MODE
        printf("Entering Previous Mode Failure... \n");
        printf("Setting Filter Failure... \n");
#endif
        return res;
    }
#if DEBUG_MODE
    printf("Setting Filter Successful! \n");
#endif

    return res;
}

INT8U MCP_CAN::init_Filt(INT8U num, INT32U ulData)
{
    INT8U res = MCP2515_OK;
    INT8U ext = 0;

#if DEBUG_MODE
    printf("Starting to Set Filter! \n");
#endif
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if (res > 0)
    {
#if DEBUG_MODE
        printf("Enter Configuration Mode Failure... \n");
#endif
        return res;
    }

    if ((ulData & 0x80000000) == 0x80000000)
        ext = 1;

    switch (num)
    {
    case 0:
        mcp2515_write_mf(MCP_RXF0SIDH, ext, ulData);
        break;

    case 1:
        mcp2515_write_mf(MCP_RXF1SIDH, ext, ulData);
        break;

    case 2:
        mcp2515_write_mf(MCP_RXF2SIDH, ext, ulData);
        break;

    case 3:
        mcp2515_write_mf(MCP_RXF3SIDH, ext, ulData);
        break;

    case 4:
        mcp2515_write_mf(MCP_RXF4SIDH, ext, ulData);
        break;

    case 5:
        mcp2515_write_mf(MCP_RXF5SIDH, ext, ulData);
        break;

    default:
        res = MCP2515_FAIL;
    }

    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if (res > 0)
    {
#if DEBUG_MODE
        printf("Entering Previous Mode Failure... \n");
        printf("Setting Filter Failure... \n");
#endif
        return res;
    }
#if DEBUG_MODE
    printf("Setting Filter Successful! \n");
#endif

    return res;
}

INT8U MCP_CAN::setMsg(INT32U id, INT8U rtr, INT8U ext, INT8U len, INT8U *pData)
{
    int i = 0;
    m_nID = id;
    m_nRtr = rtr;
    m_nExtFlg = ext;
    m_nDlc = len;
    for (i = 0; i < MAX_CHAR_IN_MESSAGE; i++)
        m_nDta[i] = *(pData + i);

    return MCP2515_OK;
}

INT8U MCP_CAN::clearMsg()
{
    m_nID = 0;
    m_nDlc = 0;
    m_nExtFlg = 0;
    m_nRtr = 0;
    m_nfilhit = 0;
    for (int i = 0; i < m_nDlc; i++)
        m_nDta[i] = 0x00;

    return MCP2515_OK;
}

INT8U MCP_CAN::sendMsg()
{
    INT8U res, res1, txbuf_n;
    Timer timer;

    timer.start(); /* Start the timer */

    // 24 * 4 microseconds typical
    do
    {
        res = mcp2515_getNextFreeTXBuf(&txbuf_n); /* info = addr. */
    } while (res == MCP_ALLTXBUSY && (timer.elapsed_time().count() < TIMEOUTVALUE));

    if (timer.elapsed_time().count() >= TIMEOUTVALUE)
    {
        return CAN_GETTXBFTIMEOUT; /* get tx buff time out */
    }

    timer.reset(); /* Reset the timer */
    mcp2515_write_canMsg(txbuf_n);
    mcp2515_modifyRegister(txbuf_n - 1, MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M);

    do
    {
        res1 = mcp2515_readRegister(txbuf_n - 1); /* read send buff ctrl reg */
        res1 = res1 & 0x08;
    } while (res1 && (timer.elapsed_time().count() < TIMEOUTVALUE));

    if (timer.elapsed_time().count() >= TIMEOUTVALUE) /* send msg timeout */
        return CAN_SENDMSGTIMEOUT;

    return CAN_OK;
}

INT8U MCP_CAN::sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf)
{
    INT8U res;

    setMsg(id, 0, ext, len, buf);
    res = sendMsg();

    return res;
}

INT8U MCP_CAN::sendMsgBuf(INT32U id, INT8U len, INT8U *buf)
{
    INT8U ext = 0, rtr = 0;
    INT8U res;

    if ((id & 0x80000000) == 0x80000000)
        ext = 1;

    if ((id & 0x40000000) == 0x40000000)
        rtr = 1;

    setMsg(id, rtr, ext, len, buf);
    res = sendMsg();

    return res;
}

INT8U MCP_CAN::readMsg()
{
    INT8U stat, res;

    stat = mcp2515_readStatus();

    if (stat & MCP_STAT_RX0IF) /* Msg in Buffer 0              */
    {
        mcp2515_read_canMsg(MCP_RXBUF_0);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if (stat & MCP_STAT_RX1IF) /* Msg in Buffer 1              */
    {
        mcp2515_read_canMsg(MCP_RXBUF_1);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else
        res = CAN_NOMSG;

    return res;
}

INT8U MCP_CAN::readMsgBuf(INT32U *id, INT8U *ext, INT8U *len, INT8U buf[])
{
    if (readMsg() == CAN_NOMSG)
        return CAN_NOMSG;

    *id = m_nID;
    *len = m_nDlc;
    *ext = m_nExtFlg;
    for (int i = 0; i < m_nDlc; i++)
        buf[i] = m_nDta[i];

    return CAN_OK;
}

INT8U MCP_CAN::readMsgBuf(INT32U *id, INT8U *len, INT8U buf[])
{
    if (readMsg() == CAN_NOMSG)
        return CAN_NOMSG;

    if (m_nExtFlg)
        m_nID |= 0x80000000;

    if (m_nRtr)
        m_nID |= 0x40000000;

    *id = m_nID;
    *len = m_nDlc;

    for (int i = 0; i < m_nDlc; i++)
        buf[i] = m_nDta[i];

    return CAN_OK;
}

INT8U MCP_CAN::checkReceive(void)
{
    INT8U res;
    res = mcp2515_readStatus(); /* RXnIF in Bit 1 and 0         */
    if (res & MCP_STAT_RXIF_MASK)
        return CAN_MSGAVAIL;
    else
        return CAN_NOMSG;
}

INT8U MCP_CAN::checkError(void)
{
    INT8U eflg = mcp2515_readRegister(MCP_EFLG);

    if (eflg & MCP_EFLG_ERRORMASK)
        return CAN_CTRLERROR;
    else
        return CAN_OK;
}

INT8U MCP_CAN::getError(void)
{
    return mcp2515_readRegister(MCP_EFLG);
}

INT8U MCP_CAN::errorCountRX(void)
{
    return mcp2515_readRegister(MCP_REC);
}

INT8U MCP_CAN::errorCountTX(void)
{
    return mcp2515_readRegister(MCP_TEC);
}

INT8U MCP_CAN::enOneShotTX(void)
{
    mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
    if ((mcp2515_readRegister(MCP_CANCTRL) & MODE_ONESHOT) != MODE_ONESHOT)
        return CAN_FAIL;
    else
        return CAN_OK;
}

INT8U MCP_CAN::disOneShotTX(void)
{
    mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, 0);
    if ((mcp2515_readRegister(MCP_CANCTRL) & MODE_ONESHOT) != 0)
        return CAN_FAIL;
    else
        return CAN_OK;
}

INT8U MCP_CAN::abortTX(void)
{
    mcp2515_modifyRegister(MCP_CANCTRL, ABORT_TX, ABORT_TX);

    // Maybe check to see if the TX buffer transmission request bits are cleared instead?
    if ((mcp2515_readRegister(MCP_CANCTRL) & ABORT_TX) != ABORT_TX)
        return CAN_FAIL;
    else
        return CAN_OK;
}

INT8U MCP_CAN::setGPO(INT8U data)
{
    mcp2515_modifyRegister(MCP_BFPCTRL, MCP_BxBFS_MASK, (data << 4));

    return 0;
}

INT8U MCP_CAN::getGPI(void)
{
    INT8U res;
    res = mcp2515_readRegister(MCP_TXRTSCTRL) & MCP_BxRTS_MASK;
    return (res >> 3);
}
