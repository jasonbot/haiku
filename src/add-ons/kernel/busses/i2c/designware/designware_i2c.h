/*
 * Copyright 2024, Haiku, Inc.
 * Distributed under the terms of the MIT License.
 */

#ifndef _DESIGNWARE_I2C_H_
#define _DESIGNWARE_I2C_H_

#include <i2c.h>
#include <ByteOrder.h>
#include <assert.h>

#include <AutoDeleterOS.h>
#include <lock.h>


#define CHECK_RET(err) {status_t _err = (err); if (_err < B_OK) return _err;}

#define DESIGNWARE_I2C_DRIVER_MODULE_NAME "busses/i2c/designware_i2c/driver_v1"


#define DW_IC_CON				0x00
#define DW_IC_CON_MASTER			0x1
#define DW_IC_CON_SPEED_STD			0x2
#define DW_IC_CON_SPEED_FAST			0x4
#define DW_IC_CON_SPEED_HIGH			0x6
#define DW_IC_CON_10BIT_ADDR_MASTER		0x10
#define DW_IC_CON_RESTART_EN			0x20
#define DW_IC_CON_SLAVE_DISABLE			0x40
#define DW_IC_CON_TX_EMPTY_CTRL			0x100
#define DW_IC_TAR				0x04
#define DW_IC_DATA_CMD				0x10
#define DW_IC_DATA_CMD_READ			(1 << 8)
#define DW_IC_DATA_CMD_STOP			(1 << 9)
#define DW_IC_DATA_CMD_RESTART			(1 << 10)
#define DW_IC_SS_SCL_HCNT			0x14
#define DW_IC_SS_SCL_LCNT			0x18
#define DW_IC_FS_SCL_HCNT			0x1c
#define DW_IC_FS_SCL_LCNT			0x20
#define DW_IC_INTR_STAT				0x2c
#define DW_IC_INTR_STAT_RX_UNDER		(1 << 0)
#define DW_IC_INTR_STAT_RX_OVER			(1 << 1)
#define DW_IC_INTR_STAT_RX_FULL			(1 << 2)
#define DW_IC_INTR_STAT_TX_OVER			(1 << 3)
#define DW_IC_INTR_STAT_TX_EMPTY		(1 << 4)
#define DW_IC_INTR_STAT_RD_REQ			(1 << 5)
#define DW_IC_INTR_STAT_TX_ABRT			(1 << 6)
#define DW_IC_INTR_STAT_RX_DONE			(1 << 7)
#define DW_IC_INTR_STAT_ACTIVITY		(1 << 8)
#define DW_IC_INTR_STAT_STOP_DET		(1 << 9)
#define DW_IC_INTR_STAT_START_DET		(1 << 10)
#define DW_IC_INTR_STAT_GEN_CALL		(1 << 11)
#define DW_IC_INTR_MASK				0x30
#define DW_IC_RAW_INTR_STAT			0x34
#define DW_IC_RX_TL				0x38
#define DW_IC_TX_TL				0x3c
#define DW_IC_CLR_INTR				0x40
#define DW_IC_CLR_RX_UNDER			0x44
#define DW_IC_CLR_RX_OVER			0x48
#define DW_IC_CLR_TX_OVER			0x4c
#define DW_IC_CLR_RD_REQ			0x50
#define DW_IC_CLR_TX_ABRT			0x54
#define DW_IC_CLR_RX_DONE			0x58
#define DW_IC_CLR_ACTIVITY			0x5c
#define DW_IC_CLR_STOP_DET			0x60
#define DW_IC_CLR_START_DET			0x64
#define DW_IC_CLR_GEN_CALL			0x68
#define DW_IC_ENABLE				0x6c
#define DW_IC_STATUS				0x70
#define DW_IC_STATUS_ACTIVITY			0x1
#define DW_IC_TXFLR				0x74
#define DW_IC_RXFLR				0x78
#define DW_IC_SDA_HOLD				0x7c
#define DW_IC_TX_ABRT_SOURCE			0x80
#define DW_IC_ENABLE_STATUS			0x9c
#define DW_IC_COMP_PARAM1			0xf4
#define DW_IC_COMP_PARAM1_RX(x)			(1 + (((x) >> 8) & 0xff))
#define DW_IC_COMP_PARAM1_TX(x)			(1 + (((x) >> 16) & 0xff))
#define DW_IC_COMP_VERSION			0xf8


class DesignwareI2c {
public:
	static float SupportsDevice(device_node* parent);
	static status_t RegisterDevice(device_node* parent);
	static status_t InitDriver(device_node* node, DesignwareI2c*& outDriver);
	void UninitDriver();

	void SetI2cBus(i2c_bus bus);
	status_t ExecCommand(i2c_op op,
		i2c_addr slaveAddress, const uint8 *cmdBuffer, size_t cmdLength,
		uint8* dataBuffer, size_t dataLength);
	status_t AcquireBus();
	void ReleaseBus();

private:
	inline status_t InitDriverInt(device_node* node);

	void EnableDevice(bool enable);
	status_t WaitForBusIdle();
	status_t WaitForTxEmpty();
	status_t WaitForRxFull();

private:
	struct mutex fLock = MUTEX_INITIALIZER("Designware i2c");

	AreaDeleter fRegsArea;
	volatile addr_t fRegs{};

	device_node* fNode{};
	i2c_bus fBus{};

	uint8 fTxFifoDepth = 32;
	uint8 fRxFifoDepth = 32;
};


extern device_manager_info* gDeviceManager;
extern i2c_for_controller_interface* gI2c;
extern i2c_sim_interface gDesignwareI2cDriver;

#endif	// _DESIGNWARE_I2C_H_
