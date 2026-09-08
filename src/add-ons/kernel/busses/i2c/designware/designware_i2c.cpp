/*
 * Copyright 2024, Haiku, Inc.
 * Distributed under the terms of the MIT License.
 */


#include "designware_i2c.h"
#include <bus/FDT.h>

#include <AutoDeleterDrivers.h>

#include <string.h>
#include <new>


#define write32(address, data) \
	(*((volatile uint32*)((addr_t)fRegs + (address))) = (data))
#define read32(address) \
	(*((volatile uint32*)((addr_t)fRegs + (address))))


//#define TRACE_DESIGNWARE_I2C
#ifdef TRACE_DESIGNWARE_I2C
#	define TRACE(x...) dprintf("\33[33mdesignware_i2c:\33[0m " x)
#else
#	define TRACE(x...) ;
#endif
#define TRACE_ALWAYS(x...)	dprintf("\33[33mdesignware_i2c:\33[0m " x)
#define ERROR(x...)			dprintf("\33[33mdesignware_i2c:\33[0m " x)


static bool
is_designware_compatible(const char* compatible)
{
	if (compatible == NULL)
		return false;

	if (strstr(compatible, "snps,designware-i2c") != NULL)
		return true;
	if (strstr(compatible, "rockchip,rk3399-i2c") != NULL)
		return true;
	if (strstr(compatible, "rockchip,rk3328-i2c") != NULL)
		return true;
	if (strstr(compatible, "rockchip,rk3368-i2c") != NULL)
		return true;
	if (strstr(compatible, "rockchip,rk3288-i2c") != NULL)
		return true;

	return false;
}


void
DesignwareI2c::EnableDevice(bool enable)
{
	uint32 status = enable ? 1 : 0;
	for (int tries = 100; tries >= 0; tries--) {
		write32(DW_IC_ENABLE, status);
		if ((read32(DW_IC_ENABLE_STATUS) & 1) == status)
			return;
		bigtime_t start = system_time();
		while (system_time() - start < 25)
			;
	}
	ERROR("EnableDevice failed\n");
}


status_t
DesignwareI2c::WaitForBusIdle()
{
	for (int tries = 100; tries >= 0; tries--) {
		uint32 status = read32(DW_IC_STATUS);
		if ((status & DW_IC_STATUS_ACTIVITY) == 0)
			return B_OK;
		bigtime_t start = system_time();
		while (system_time() - start < 1000)
			;
	}
	return B_BUSY;
}


status_t
DesignwareI2c::WaitForTxEmpty()
{
	for (int tries = 1000; tries >= 0; tries--) {
		uint32 status = read32(DW_IC_RAW_INTR_STAT);
		if ((status & DW_IC_INTR_STAT_TX_EMPTY) != 0)
			return B_OK;
		bigtime_t start = system_time();
		while (system_time() - start < 100)
			;
	}
	return B_BUSY;
}


status_t
DesignwareI2c::WaitForRxFull()
{
	for (int tries = 1000; tries >= 0; tries--) {
		uint32 status = read32(DW_IC_RAW_INTR_STAT);
		if ((status & DW_IC_INTR_STAT_RX_FULL) != 0)
			return B_OK;
		bigtime_t start = system_time();
		while (system_time() - start < 100)
			;
	}
	return B_BUSY;
}


float
DesignwareI2c::SupportsDevice(device_node* parent)
{
	const char* bus;
	status_t status = gDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false);
	if (status < B_OK)
		return -1.0f;

	if (strcmp(bus, "fdt") != 0)
		return 0.0f;

	const char* compatible;
	status = gDeviceManager->get_attr_string(parent, "fdt/compatible", &compatible, false);
	if (status < B_OK)
		return -1.0f;

	if (!is_designware_compatible(compatible))
		return 0.0f;

	return 1.0f;
}


status_t
DesignwareI2c::RegisterDevice(device_node* parent)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE, {string: "DesignWare I2C Controller"} },
		{ B_DEVICE_FIXED_CHILD, B_STRING_TYPE, {string: I2C_FOR_CONTROLLER_MODULE_NAME} },
		{}
	};

	return gDeviceManager->register_node(parent, DESIGNWARE_I2C_DRIVER_MODULE_NAME, attrs, NULL, NULL);
}


status_t
DesignwareI2c::InitDriver(device_node* node, DesignwareI2c*& outDriver)
{
	ObjectDeleter<DesignwareI2c> driver(new(std::nothrow) DesignwareI2c());
	if (!driver.IsSet())
		return B_NO_MEMORY;

	CHECK_RET(driver->InitDriverInt(node));
	outDriver = driver.Detach();
	return B_OK;
}


status_t
DesignwareI2c::InitDriverInt(device_node* node)
{
	fNode = node;
	TRACE_ALWAYS("+DesignwareI2c::InitDriver()\n");

	DeviceNodePutter<&gDeviceManager> parent(gDeviceManager->get_parent_node(node));

	const char* bus;
	CHECK_RET(gDeviceManager->get_attr_string(parent.Get(), B_DEVICE_BUS, &bus, false));
	if (strcmp(bus, "fdt") != 0)
		return B_ERROR;

	fdt_device_module_info *parentModule;
	fdt_device* parentDev;
	CHECK_RET(gDeviceManager->get_driver(parent.Get(),
		(driver_module_info**)&parentModule, (void**)&parentDev));

	uint64 regs = 0;
	uint64 regsLen = 0;
	if (!parentModule->get_reg(parentDev, 0, &regs, &regsLen))
		return B_ERROR;

	fRegsArea.SetTo(map_physical_memory("Designware i2c MMIO", regs, regsLen, B_ANY_KERNEL_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, (void**)&fRegs));
	if (!fRegsArea.IsSet())
		return fRegsArea.Get();

	uint32 version = read32(DW_IC_COMP_VERSION);
	TRACE_ALWAYS("DesignWare I2C version: 0x%08" B_PRIx32 "\n", version);

	uint32 compParam = read32(DW_IC_COMP_PARAM1);
	fTxFifoDepth = DW_IC_COMP_PARAM1_TX(compParam);
	fRxFifoDepth = DW_IC_COMP_PARAM1_RX(compParam);
	TRACE_ALWAYS("TX FIFO depth: %d, RX FIFO depth: %d\n", fTxFifoDepth, fRxFifoDepth);

	EnableDevice(false);

	write32(DW_IC_CON, DW_IC_CON_MASTER | DW_IC_CON_SLAVE_DISABLE |
		DW_IC_CON_RESTART_EN | DW_IC_CON_SPEED_FAST);

	write32(DW_IC_RX_TL, 0);
	write32(DW_IC_TX_TL, fTxFifoDepth / 2);

	write32(DW_IC_INTR_MASK, 0);
	read32(DW_IC_CLR_INTR);

	EnableDevice(true);

	TRACE_ALWAYS("-DesignwareI2c::InitDriver()\n");
	return B_OK;
}


void
DesignwareI2c::UninitDriver()
{
	EnableDevice(false);
	delete this;
}


void
DesignwareI2c::SetI2cBus(i2c_bus bus)
{
	TRACE("DesignwareI2c::SetI2cBus()\n");
	fBus = bus;
}


status_t
DesignwareI2c::ExecCommand(i2c_op op,
	i2c_addr slaveAddress, const uint8 *cmdBuffer, size_t cmdLength,
	uint8* dataBuffer, size_t dataLength)
{
	TRACE("DesignwareI2c::ExecCommand(op=%d, addr=0x%02x, cmdLen=%lu, dataLen=%lu)\n",
		op, slaveAddress, cmdLength, dataLength);

	CHECK_RET(WaitForBusIdle());

	EnableDevice(false);
	write32(DW_IC_CON, read32(DW_IC_CON) & ~DW_IC_CON_10BIT_ADDR_MASTER);
	write32(DW_IC_TAR, slaveAddress);
	write32(DW_IC_INTR_MASK, 0);
	read32(DW_IC_CLR_INTR);
	EnableDevice(true);

	if (cmdLength > 0) {
		TRACE("Writing command buffer (%lu bytes)\n", cmdLength);
		for (size_t i = 0; i < cmdLength; i++) {
			uint32 cmd = cmdBuffer[i];
			if (i == cmdLength - 1 && dataLength == 0 && IS_STOP_OP(op))
				cmd |= DW_IC_DATA_CMD_STOP;
			write32(DW_IC_DATA_CMD, cmd);
		}
		CHECK_RET(WaitForTxEmpty());
	}

	if (dataLength > 0) {
		if (IS_WRITE_OP(op)) {
			TRACE("Writing data buffer (%lu bytes)\n", dataLength);
			for (size_t i = 0; i < dataLength; i++) {
				uint32 cmd = dataBuffer[i];
				if (i == 0 && cmdLength > 0 && IS_READ_OP(op))
					cmd |= DW_IC_DATA_CMD_RESTART;
				if (i == dataLength - 1 && IS_STOP_OP(op))
					cmd |= DW_IC_DATA_CMD_STOP;
				write32(DW_IC_DATA_CMD, cmd);
			}
			CHECK_RET(WaitForTxEmpty());
		} else {
			TRACE("Reading data buffer (%lu bytes)\n", dataLength);
			for (size_t i = 0; i < dataLength; i++) {
				uint32 cmd = DW_IC_DATA_CMD_READ;
				if (i == 0 && cmdLength > 0)
					cmd |= DW_IC_DATA_CMD_RESTART;
				if (i == dataLength - 1 && IS_STOP_OP(op))
					cmd |= DW_IC_DATA_CMD_STOP;
				write32(DW_IC_DATA_CMD, cmd);

				CHECK_RET(WaitForRxFull());
				dataBuffer[i] = read32(DW_IC_DATA_CMD) & 0xff;
			}
		}
	}

	if (IS_STOP_OP(op)) {
		for (int tries = 100; tries >= 0; tries--) {
			uint32 status = read32(DW_IC_RAW_INTR_STAT);
			if ((status & DW_IC_INTR_STAT_STOP_DET) != 0) {
				read32(DW_IC_CLR_STOP_DET);
				break;
			}
			bigtime_t start = system_time();
			while (system_time() - start < 1000)
				;
		}
	}

	return B_OK;
}


status_t
DesignwareI2c::AcquireBus()
{
	return mutex_lock(&fLock);
}


void
DesignwareI2c::ReleaseBus()
{
	mutex_unlock(&fLock);
}
