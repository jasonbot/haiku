/*
 * Copyright 2026 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */

#include <algorithm>
#include <new>
#include <stdio.h>
#include <string.h>

#include <ByteOrder.h>
#include <KernelExport.h>
#include <OS.h>

#include <arch_cpu.h>

#include <drivers/bus/FDT.h>

#include "IOSchedulerSimple.h"
#include "mmc.h"
#include "dwmmc.h"


//#define TRACE_DWMMC
#ifdef TRACE_DWMMC
#	define TRACE(x...) dprintf("\33[33mdwmmc:\33[0m " x)
#else
#	define TRACE(x...) ;
#endif
#define TRACE_ALWAYS(x...)	dprintf("\33[33mdwmmc:\33[0m " x)
#define ERROR(x...)			dprintf("\33[33mdwmmc:\33[0m " x)
#define CALLED(x...)		TRACE("CALLED %s\n", __PRETTY_FUNCTION__)


// Busy-wait helper. snooze() is unreliable on this bring-up platform, so all
// short IO waits spin on system_time() instead.
static inline void
spin_us(bigtime_t us)
{
	bigtime_t end = system_time() + us;
	while (system_time() < end)
		__asm__ __volatile__("dmb ishst" : : : "memory");
}


#define DWMMC_DEVICE_MODULE_NAME "busses/mmc/dwmmc/driver_v1"
#define DWMMC_MMC_BUS_MODULE_NAME "busses/mmc/dwmmc/device/v1"


// The clock source (CIU) clock is fixed for the SoC we target (Rockchip
// RK3399 uses a 150MHz clock for the dw_mshc blocks).
#define DWMMC_CLOCK_IN_KHZ		150000

#define DWMMC_CLOCK_MAX_KHZ		200000
#define DWMMC_CLOCK_MIN_KHZ		100

#define kInvalidCommandFlags	0xffffffff


device_manager_info* gDeviceManager;
device_module_info* gMMCBusController;


struct DwMmcDevice {
	device_node* fNode;

	DwMmcDevice()
		:
		fNode(NULL)
	{
	}
};


/* Compute the CMD register flags for a command. The response type is derived
 * from the command as documented in the SD Physical Layer Simplified
 * Specification (and eMMC JEDEC standard for the MMC only commands), the
 * same way the sdhci driver does it. */
static uint32
_CommandFlags(uint8_t command, card_type cardType, bool* _response128,
	bool* _waitBusy)
{
	*_response128 = false;
	*_waitBusy = false;

	switch (command) {
		case GO_IDLE_STATE:
			// No reply
			return 0;

		case SD_APP_CMD:
		case SD_ERASE_WR_BLK_START:
		case SD_ERASE_WR_BLK_END:
			// R1 reply
			return DWMCI_CMD_RESP_EXP | DWMCI_CMD_CHECK_CRC;

		case SELECT_DESELECT_CARD:
		case SD_ERASE:
		case SD_STOP_TRANSMISSION:
			// R1b reply, the card keeps the data line busy afterwards
			*_waitBusy = true;
			return DWMCI_CMD_RESP_EXP | DWMCI_CMD_CHECK_CRC;

		case ALL_SEND_CID:
		case SEND_CSD:
			// R2 reply (136 bits)
			*_response128 = true;
			return DWMCI_CMD_RESP_EXP | DWMCI_CMD_RESP_LENGTH
				| DWMCI_CMD_CHECK_CRC;

		case MMC_SEND_OP_COND:
		case SD_SEND_OP_COND:
			// R3 reply, no CRC field
			return DWMCI_CMD_RESP_EXP;

		case SD_SET_BUS_WIDTH: // SD application command; also MMC_SWITCH
			if (cardType == CARD_TYPE_MMC)
				*_waitBusy = true; // R1b
			return DWMCI_CMD_RESP_EXP | DWMCI_CMD_CHECK_CRC;

		case SD_SEND_RELATIVE_ADDR:
			// R6 reply (or R1 for MMC)
			return DWMCI_CMD_RESP_EXP | DWMCI_CMD_CHECK_CRC;

		case SD_SEND_IF_COND:
			// R7 reply (or R1 for MMC)
			return DWMCI_CMD_RESP_EXP | DWMCI_CMD_CHECK_CRC;

		case SD_READ_SINGLE_BLOCK:
		case SD_READ_MULTIPLE_BLOCKS:
		case SD_WRITE_SINGLE_BLOCK:
		case SD_WRITE_MULTIPLE_BLOCKS:
			// R1 reply, data will follow
			return DWMCI_CMD_RESP_EXP | DWMCI_CMD_CHECK_CRC;

		default:
			ERROR("Unknown command %x\n", command);
			return kInvalidCommandFlags;
	}
}


DwMmcBus::DwMmcBus(volatile uint32_t* registers, uint8_t irq,
	area_id regsArea, uint32_t fifoDepth)
	:
	fRegisters(registers),
	fIrq(irq),
	fRegsArea(regsArea),
	fCardType(CARD_TYPE_UNKNOWN),
	fStatus(B_OK),
	fFifoDepth(fifoDepth),
	fScanSemaphore(-1)
{
	// Make sure we start from a clean state
	if (_WaitReset(DWMCI_RESET_ALL) != B_OK) {
		ERROR("controller reset failed\n");
		fStatus = B_ERROR;
		return;
	}

	TRACE("DesignWare MMC controller version: %#" B_PRIx32 "\n",
		(Read(DWMCI_VERID) >> 16) & 0xfff);

	// Power on the card slot
	Write(DWMCI_PWREN, 1);

	// Determine the FIFO depth (in 32-bit words). Prefer the fifo-depth
	// device tree property; fall back to the current FIFOTH watermark value
	// that the firmware programmed.
	if (fFifoDepth == 0) {
		uint32_t fifoTh = Read(DWMCI_FIFOTH);
		fFifoDepth = 1 + ((fifoTh & DWMCI_RX_WMARK_MASK)
			>> DWMCI_RX_WMARK_SHIFT);
	}
	TRACE("FIFO depth: %" B_PRIu32 " words\n", fFifoDepth);

	// Configure the FIFO thresholds: 32-word bursts and a watermark half the
	// fifo size (receive watermark just under half, transmit watermark half).
	Write(DWMCI_FIFOTH, DWMCI_MSIZE(2)
		| DWMCI_RX_WMARK(fFifoDepth / 2 - 1)
		| DWMCI_TX_WMARK(fFifoDepth / 2));

	// We operate in polling mode: make sure no interrupt ever reaches the GIC
	Write(DWMCI_INTMASK, 0);

	// Long enough timeouts for the data and response lines
	Write(DWMCI_TMOUT, 0xffffffff);

	// 1-bit bus by default, switched to 4-bit once the card is enumerated
	Write(DWMCI_CTYPE, 0);

	// Slow clock so the card can be initialized safely
	if (SetClock(400) != B_OK)
		ERROR("Could not set initial clock\n");
}


DwMmcBus::~DwMmcBus()
{
	TerminateBus();

	if (fRegsArea >= 0)
		delete_area(fRegsArea);

	fStatus = B_SHUTTING_DOWN;
}


uint32_t
DwMmcBus::Read(uint32_t registerOffset)
{
	return *(volatile uint32_t*)(fRegisters + (registerOffset / 4));
}


void
DwMmcBus::Write(uint32_t registerOffset, uint32_t value)
{
	*(volatile uint32_t*)(fRegisters + (registerOffset / 4)) = value;
}


void
DwMmcBus::ClearInterrupts()
{
	Write(DWMCI_RINTSTS, 0xffff);
}


status_t
DwMmcBus::InitCheck()
{
	return fStatus;
}


status_t
DwMmcBus::_WaitReset(uint32_t value)
{
	// Wait for any previous reset to complete
	bigtime_t start = system_time();
	while ((Read(DWMCI_CTRL) & DWMCI_CTRL_RESET) != 0) {
		if (system_time() - start > 1000000)
			return B_TIMED_OUT;
		spin_us(100);
	}

	Write(DWMCI_CTRL, value);

	start = system_time();
	while ((Read(DWMCI_CTRL) & (DWMCI_CTRL_RESET | DWMCI_CTRL_FIFO_RESET
			| DWMCI_CTRL_DMA_RESET)) != 0) {
		if (system_time() - start > 1000000)
			return B_TIMED_OUT;
		spin_us(100);
	}

	return B_OK;
}


status_t
DwMmcBus::_WaitCommandComplete(bigtime_t timeout)
{
	bigtime_t start = system_time();

	while (true) {
		uint32_t status = Read(DWMCI_RINTSTS);

		if ((status & DWMCI_INTMSK_RTO) != 0) {
			Write(DWMCI_RINTSTS, DWMCI_INTMSK_RTO);
			TRACE_ALWAYS("WaitCmd RTO\n");
			return B_TIMED_OUT;
		}
		if ((status & DWMCI_INTMSK_RCRC) != 0) {
			Write(DWMCI_RINTSTS, DWMCI_INTMSK_RCRC);
			TRACE_ALWAYS("WaitCmd RCRC\n");
			return B_BAD_VALUE;
		}
		if ((status & DWMCI_INTMSK_HLE) != 0) {
			Write(DWMCI_RINTSTS, DWMCI_INTMSK_HLE);
			TRACE_ALWAYS("WaitCmd HLE\n");
			return B_IO_ERROR;
		}
		if ((status & DWMCI_INTMSK_CDONE) != 0) {
			Write(DWMCI_RINTSTS, DWMCI_INTMSK_CDONE);
			return B_OK;
		}

		if (system_time() - start > timeout) {
			TRACE_ALWAYS("WaitCmd timeout\n");
			return B_TIMED_OUT;
		}

		for (uint32_t i = 0; i < 100000; i++)
			__asm__ __volatile__("dmb ishst" : : : "memory");
	}
}


status_t
DwMmcBus::_WaitForIdle(bigtime_t timeout)
{
	bigtime_t start = system_time();

	while ((Read(DWMCI_STATUS) & DWMCI_BUSY) != 0) {
		if (system_time() - start > timeout)
			return B_TIMED_OUT;
		__asm__ __volatile__("dmb ishst" : : : "memory");
	}

	return B_OK;
}


status_t
DwMmcBus::_UpdateClock()
{
	Write(DWMCI_CMDARG, 0);
#if defined(__aarch64__)
	__asm__ __volatile__("dmb ishst" : : : "memory");
#endif
	Write(DWMCI_CMD, DWMCI_CMD_UPD_CLK | DWMCI_CMD_PRV_DAT_WAIT
		| DWMCI_CMD_START);
#if defined(__aarch64__)
	__asm__ __volatile__("dmb ish" : : : "memory");
#endif

	// A clock update is not an MMC command, so no CDONE interrupt is
	// generated. The controller clears the START bit in CMD once the clock
	// change has been latched, so poll for that instead.
	bigtime_t start = system_time();
	while ((Read(DWMCI_CMD) & DWMCI_CMD_START) != 0) {
		if (system_time() - start > 1000000) {
			ERROR("clock update timeout: CMD=%#" B_PRIx32
				" STATUS=%#" B_PRIx32 " CTRL=%#" B_PRIx32 "\n",
				Read(DWMCI_CMD), Read(DWMCI_STATUS),
				Read(DWMCI_CTRL));
			return B_TIMED_OUT;
		}
		spin_us(10);
	}

	// Sweep any stale raw interrupt bits so they don't confuse the next
	// command.
	ClearInterrupts();

	return B_OK;
}


status_t
DwMmcBus::SetClock(int kilohertz)
{
	if (kilohertz < DWMMC_CLOCK_MIN_KHZ
		|| kilohertz > DWMMC_CLOCK_MAX_KHZ) {
		ERROR("Invalid clock speed %d kHz\n", kilohertz);
		return B_BAD_VALUE;
	}

	// Mirror the reference driver's divider math so the card is never
	// over-clocked. For the input clock the divider is
	// DIV_ROUND_UP(bus_hz / clock, 2); at 400 kHz that gives 188 (398 kHz).
	uint32_t divider = DWMMC_CLOCK_IN_KHZ / (uint32_t)kilohertz;
	if (DWMMC_CLOCK_IN_KHZ % (uint32_t)kilohertz
		&& DWMMC_CLOCK_IN_KHZ > (uint32_t)kilohertz)
		divider++;
	if (DWMMC_CLOCK_IN_KHZ != (uint32_t)kilohertz)
		divider = (divider + 1) / 2;
	if (divider == 0)
		divider = 1;
	divider--;
	if (divider > 0xffff)
		divider = 0xffff;

	TRACE("SetClock(%d): divider %" B_PRIu32 ", actual %" B_PRIu32 " kHz\n",
		kilohertz, divider, DWMMC_CLOCK_IN_KHZ / ((divider + 1) * 2));

	status_t status;

	// Disable the card clock and latch the change.
	Write(DWMCI_CLKENA, 0);
	Write(DWMCI_CLKSRC, 0);
	status = _UpdateClock();
	if (status != B_OK)
		return status;

	// Program the new divider and latch it.
	Write(DWMCI_CLKDIV, divider);
	status = _UpdateClock();
	if (status != B_OK)
		return status;

	// Re-enable the clock and latch the divider into effect.
	Write(DWMCI_CLKENA, DWMCI_CLKEN_ENABLE);
	return _UpdateClock();
}


status_t
DwMmcBus::ExecuteCommand(uint8_t command, uint32_t argument, uint32_t* response)
{
	TRACE("ExecuteCommand(%d, %#" B_PRIx32 ")\n", command, argument);

	bool response128 = false;
	bool waitBusy = false;
	uint32_t cmdFlags = _CommandFlags(command, fCardType, &response128,
		&waitBusy);
	if (cmdFlags == kInvalidCommandFlags)
		return B_BAD_DATA;

	ClearInterrupts();

	Write(DWMCI_CMDARG, argument);
	Write(DWMCI_CMD, DWMCI_CMD_CMD_INDEX(command) | cmdFlags
		| DWMCI_CMD_START);

	status_t status = _WaitCommandComplete(1000000);
	if (status != B_OK) {
		ERROR("Command %d failed: %s\n", command, strerror(status));
		goto done;
	}

	// Read the response. Responses come in natural register order: RESP0 is
	// the least significant word and RESP3 the most significant one (the
	// first word received on the bus).
	if (response != NULL) {
		if (response128) {
			response[0] = Read(DWMCI_RESP0);
			response[1] = Read(DWMCI_RESP1);
			response[2] = Read(DWMCI_RESP2);
			response[3] = Read(DWMCI_RESP3);
		} else
			response[0] = Read(DWMCI_RESP0);
	}

	// Commands that return R1b keep the data line busy until the card is
	// ready for the next command.
	if (waitBusy) {
		status = _WaitForIdle(200000);
		if (status == B_TIMED_OUT) {
			TRACE_ALWAYS("Command %d: data line still busy\n", command);
			status = B_OK;
		}
	}

done:
	ClearInterrupts();
	return status;
}


status_t
DwMmcBus::_TransferData(uint32_t command, uint32_t argument, uint8_t* buffer,
	size_t length, bool isWrite)
{
	TRACE("_TransferData(%d, arg %#" B_PRIx32 ", %" B_PRIuSIZE " bytes, %s)\n",
		command, argument, length, isWrite ? "write" : "read");

	bool response128 = false;
	bool waitBusy = false;
	uint32_t cmdFlags = _CommandFlags(command, fCardType, &response128,
		&waitBusy);
	if (cmdFlags == kInvalidCommandFlags)
		return B_BAD_DATA;

	// Configure the transfer
	Write(DWMCI_BLKSIZ, 512);
	Write(DWMCI_BYTCNT, length);

	ClearInterrupts();

	// Issue the command with a data phase
	Write(DWMCI_CMDARG, argument);
	Write(DWMCI_CMD, DWMCI_CMD_CMD_INDEX(command) | cmdFlags
		| DWMCI_CMD_DATA_EXP | (isWrite ? DWMCI_CMD_RW : 0)
		| DWMCI_CMD_START);

	status_t status = _WaitCommandComplete(1000000);
	if (status != B_OK) {
		ERROR("Data command %d failed: %s\n", command, strerror(status));
		ClearInterrupts();
		return status;
	}

	// Transfer the data through the FIFO, in polling mode
	if (isWrite)
		status = _WriteFIFO(buffer, length);
	else
		status = _ReadFIFO(buffer, length);

	if (status != B_OK)
		ClearInterrupts();

	// Multi-block commands keep the card streaming data until an explicit
	// stop (CMD12). The controller only transfers the requested byte count,
	// so the card must be told to stop after the last block.
	if (status == B_OK
		&& (command == SD_READ_MULTIPLE_BLOCKS
			|| command == SD_WRITE_MULTIPLE_BLOCKS)) {
		uint32_t response;
		status = ExecuteCommand(SD_STOP_TRANSMISSION, 0, &response);
		if (status != B_OK)
			ERROR("Stop transmission failed: %s\n", strerror(status));
	}

	return status;
}


status_t
DwMmcBus::_ReadFIFO(uint8_t* buffer, size_t length)
{
	uint32_t* words = (uint32_t*)buffer;
	size_t wordCount = length / 4;
	size_t received = 0;

	bigtime_t start = system_time();

	while (received < wordCount) {
		uint32_t rint = Read(DWMCI_RINTSTS);

		if ((rint & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT)) != 0) {
			Write(DWMCI_RINTSTS, rint & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT));
			ERROR("Read error: %#x\n", rint);
			if ((rint & DWMCI_DATA_TOUT) != 0)
				return B_TIMED_OUT;
			return B_IO_ERROR;
		}

		uint32_t status = Read(DWMCI_STATUS);
		uint32_t fifoCount = (status >> DWMCI_FIFO_SHIFT) & DWMCI_FIFO_MASK;

		// Drain whatever is available (also the last bytes that arrive at the
		// end of the transfer and never reach the receive watermark).
		if (fifoCount > 0) {
			size_t toRead = std::min((size_t)fifoCount,
				wordCount - received);
			for (size_t i = 0; i < toRead; i++)
				words[received + i] = Read(DWMCI_DATA);
			received += toRead;
			continue;
		}

		if ((rint & DWMCI_INTMSK_RXDR) != 0)
			Write(DWMCI_RINTSTS, DWMCI_INTMSK_RXDR);

		if ((rint & DWMCI_INTMSK_DTO) != 0) {
			// Transfer complete but we did not get all the expected data
			Write(DWMCI_RINTSTS, DWMCI_INTMSK_DTO);
			return B_IO_ERROR;
		}

		if (system_time() - start > 1000000) {
			ClearInterrupts();
			return B_TIMED_OUT;
		}

		spin_us(10);
	}

	// Wait for the data transfer to complete
	start = system_time();
	while (true) {
		uint32_t rint = Read(DWMCI_RINTSTS);

		if ((rint & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT)) != 0) {
			Write(DWMCI_RINTSTS, rint & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT));
			ERROR("Read completion error: %#x\n", rint);
			if ((rint & DWMCI_DATA_TOUT) != 0)
				return B_TIMED_OUT;
			return B_IO_ERROR;
		}

		if ((rint & DWMCI_INTMSK_DTO) != 0) {
			Write(DWMCI_RINTSTS, DWMCI_INTMSK_DTO);
			return B_OK;
		}

		if (system_time() - start > 1000000) {
			ClearInterrupts();
			return B_TIMED_OUT;
		}

		spin_us(10);
	}
}


status_t
DwMmcBus::_WriteFIFO(uint8_t* buffer, size_t length)
{
	uint32_t* words = (uint32_t*)buffer;
	size_t wordCount = length / 4;
	size_t sent = 0;

	bigtime_t start = system_time();

	while (sent < wordCount) {
		uint32_t rint = Read(DWMCI_RINTSTS);

		if ((rint & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT)) != 0) {
			Write(DWMCI_RINTSTS, rint & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT));
			ERROR("Write error: %#x\n", rint);
			if ((rint & DWMCI_DATA_TOUT) != 0)
				return B_TIMED_OUT;
			return B_IO_ERROR;
		}

		if ((rint & DWMCI_INTMSK_DTO) != 0) {
			// Transfer done before all data was written
			Write(DWMCI_RINTSTS, DWMCI_INTMSK_DTO);
			return B_IO_ERROR;
		}

		if ((rint & DWMCI_INTMSK_TXDR) != 0)
			Write(DWMCI_RINTSTS, DWMCI_INTMSK_TXDR);

		uint32_t status = Read(DWMCI_STATUS);
		uint32_t fifoCount = (status >> DWMCI_FIFO_SHIFT) & DWMCI_FIFO_MASK;
		uint32_t available = fFifoDepth - fifoCount;

		if (available > 0) {
			size_t toSend = std::min((size_t)available, wordCount - sent);
			for (size_t i = 0; i < toSend; i++)
				Write(DWMCI_DATA, words[sent + i]);
			sent += toSend;
		} else {
if (system_time() - start > 1000000) {
			ClearInterrupts();
			return B_TIMED_OUT;

		}
		spin_us(10);
	}
}
	// Wait for the data transfer to complete
	start = system_time();
	while (true) {
		uint32_t rint = Read(DWMCI_RINTSTS);

		if ((rint & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT)) != 0) {
			Write(DWMCI_RINTSTS, rint & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT));
			ERROR("Write completion error: %#x\n", rint);
			if ((rint & DWMCI_DATA_TOUT) != 0)
				return B_TIMED_OUT;
			return B_IO_ERROR;
		}

		if ((rint & DWMCI_INTMSK_DTO) != 0) {
			Write(DWMCI_RINTSTS, DWMCI_INTMSK_DTO);
			return B_OK;
		}

		if (system_time() - start > 1000000) {
			ClearInterrupts();
			return B_TIMED_OUT;
		}

		spin_us(10);
	}
}


status_t
DwMmcBus::DoIO(uint8_t command, IOOperation* operation, bool offsetAsSectors)
{
	bool isWrite = operation->IsWrite();

	static const uint32 kBlockSize = 512;
	// PIO reads through the FIFO are only reliable for single-block transfers
	// on this controller; larger transfers intermittently lose data words.
	static const uint32 kMaxTransfer = 512;
	off_t offset = operation->Offset();
	generic_size_t length = operation->Length();

	TRACE("%s %" B_PRIuGENADDR " bytes at %" B_PRIdOFF "\n",
		isWrite ? "Write" : "Read", length, offset);

	// The IO scheduler enforces these constraints through the DMA properties
	ASSERT(offset % kBlockSize == 0);
	ASSERT(length % kBlockSize == 0);

	const generic_io_vec* vecs = operation->Vecs();
	generic_size_t vecOffset = 0;

	status_t result = B_OK;

	while (length > 0) {
		size_t toCopy = std::min((generic_size_t)length,
			vecs->length - vecOffset);
		toCopy = std::min<size_t>(toCopy, kMaxTransfer);

		// If the current vec is empty, we can move to the next
		if (toCopy == 0) {
			vecs++;
			vecOffset = 0;
			continue;
		}

		ASSERT(toCopy % kBlockSize == 0);

		// We transfer the data with PIO, so we need a kernel virtual address
		// for the (physical) memory segment the operation points to.
		phys_addr_t physBase = vecs->base + vecOffset;
		size_t physOffset = physBase & (B_PAGE_SIZE - 1);
		phys_addr_t mapBase = physBase - physOffset;
		size_t mapSize = (toCopy + physOffset + B_PAGE_SIZE - 1)
			& ~(B_PAGE_SIZE - 1);

		void* virtualBuffer;
		area_id mapArea = map_physical_memory("dwmmc data buffer", mapBase,
			mapSize, B_ANY_KERNEL_ADDRESS, B_KERNEL_READ_AREA
			| B_KERNEL_WRITE_AREA, &virtualBuffer);
		if (mapArea < B_OK) {
			result = mapArea;
			break;
		}

		result = _TransferData(command,
			offset / (offsetAsSectors ? kBlockSize : 1),
			(uint8_t*)virtualBuffer + physOffset, toCopy, isWrite);

		if (!isWrite && result == B_OK) {
			// The PIO read path intermittently drops or flips a word when
			// latching from the controller FIFO (no data CRC error is ever
			// reported by the controller). Re-read the same block and compare
			// to the first read; retry until the contents agree.
			static uint8_t sVerified[512];
			bool verified = false;
			for (int32_t attempt = 0; attempt < 8; attempt++) {
				uint8_t* target = (uint8_t*)virtualBuffer + physOffset;
				result = _TransferData(command,
					offset / (offsetAsSectors ? kBlockSize : 1),
					target, toCopy, false);
				if (result != B_OK)
					continue;
				if (attempt == 0) {
					memcpy(sVerified, target, toCopy);
				} else if (memcmp(sVerified, target, toCopy) == 0) {
					verified = true;
					break;
				}
			}
			if (result != B_OK) {
				ERROR("Read verification failed at offset %" B_PRIdOFF
					": %s\n", offset, strerror(result));
			} else if (verified) {
				memcpy((uint8_t*)virtualBuffer + physOffset, sVerified,
					toCopy);
			} else {
				ERROR("Read verification failed after retries at offset %"
					B_PRIdOFF "\n", offset);
				result = B_IO_ERROR;
			}
		}

		delete_area(mapArea);

		if (result != B_OK)
			break;

		length -= toCopy;
		vecOffset += toCopy;
		offset += toCopy;
	}

	return result;
}


void
DwMmcBus::SetScanSemaphore(sem_id sem)
{
	fScanSemaphore = sem;
	TRACE_ALWAYS("SetScanSemaphore(sem %" B_PRId32 ")\n", sem);

	// There is no reliable way to detect a missing card from the controller
	// (the slide switch on the Pinebook Pro uses a GPIO), so just start a
	// scan immediately.
	if (fScanSemaphore >= 0)
		release_sem(fScanSemaphore);
}


void
DwMmcBus::SetBusWidth(int width)
{
	uint32_t ctype;
	switch (width) {
		case 0:
		case 1:
			ctype = 0;
			break;
		case 4:
			ctype = DWMCI_CTYPE_4BIT;
			break;
		case 8:
			ctype = DWMCI_CTYPE_8BIT;
			break;
		default:
			panic("Incorrect bitwidth value %d\n", width);
			return;
	}

	Write(DWMCI_CTYPE, ctype);
}


void
DwMmcBus::SetCardType(card_type type)
{
	fCardType = type;
}


void
DwMmcBus::TerminateBus()
{
	CALLED();

	// Turn off the card clock and its update request.
	Write(DWMCI_CLKENA, 0);
	Write(DWMCI_CMDARG, 0);
	Write(DWMCI_CMD, DWMCI_CMD_UPD_CLK | DWMCI_CMD_START);
	Write(DWMCI_PWREN, 0);
}


// #pragma mark -


void
uninit_bus(void* bus_cookie)
{
	DwMmcBus* bus = (DwMmcBus*)bus_cookie;
	delete bus;
}


void
bus_removed(void* bus_cookie)
{
	return;
}


static status_t
register_child_devices(void* cookie)
{
	CALLED();
	DwMmcDevice* context = (DwMmcDevice*)cookie;

	device_attr attrs[] = {
		// properties of this controller for the mmc bus manager
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "DesignWare MMC host" }},
		{ B_DEVICE_FIXED_CHILD, B_STRING_TYPE,
			{ .string = MMC_BUS_MODULE_NAME } },
		{ B_DEVICE_BUS, B_STRING_TYPE, { .string = "mmc" } },

		// These restrict the IO requests to transfers we can handle. Since
		// we transfer data with PIO the exact values do not restrict DMA,
		// but they keep the buffers simple (single, sector aligned chunks).
		{ B_DMA_ALIGNMENT, B_UINT32_TYPE, { .ui32 = 511 } },
		{ B_DMA_HIGH_ADDRESS, B_UINT64_TYPE, { .ui64 = 0x100000000LL } },
		{ B_DMA_BOUNDARY, B_UINT32_TYPE, { .ui32 = (1 << 19) - 1 } },
		{ B_DMA_MAX_SEGMENT_COUNT, B_UINT32_TYPE, { .ui32 = 1 } },
		{ B_DMA_MAX_SEGMENT_BLOCKS, B_UINT32_TYPE, { .ui32 = (1 << 10) - 1 } },

		{ NULL }
	};

	device_node* node;
	if (gDeviceManager->register_node(context->fNode,
			DWMMC_MMC_BUS_MODULE_NAME, attrs, NULL, &node) != B_OK)
		return B_BAD_VALUE;

	return B_OK;
}


static status_t
init_device(device_node* node, void** device_cookie)
{
	CALLED();

	DwMmcDevice* context = new(std::nothrow) DwMmcDevice;
	if (context == NULL)
		return B_NO_MEMORY;
	context->fNode = node;
	*device_cookie = context;

	return B_OK;
}


static void
uninit_device(void* device_cookie)
{
	DwMmcDevice* context = (DwMmcDevice*)device_cookie;
	delete context;
}


static status_t
register_device(device_node* parent)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "DesignWare MMC controller" }},
		{}
	};

	return gDeviceManager->register_node(parent, DWMMC_DEVICE_MODULE_NAME,
		attrs, NULL, NULL);
}


static float
supports_device(device_node* parent)
{
	const char* bus;

	// make sure the parent is an FDT device node
	if (gDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false)
		!= B_OK) {
		TRACE("Could not find required attribute device/bus\n");
		return -1;
	}

	if (strcmp(bus, "fdt") != 0)
		return 0.0f;

	// The FDT bus registers one "fdt/compatible" attribute per compatible
	// string of the node, so iterate through all of them.
	const char* matched = NULL;
	device_attr* attr = NULL;
	while (gDeviceManager->get_next_attr(parent, &attr) == B_OK) {
		if (attr->type != B_STRING_TYPE)
			continue;
		if (strcmp(attr->name, "fdt/compatible") != 0)
			continue;

		if (strcmp(attr->value.string, "rockchip,rk3399-dw-mshc") == 0
			|| strcmp(attr->value.string, "snps,dw-mmc") == 0) {
			matched = attr->value.string;
			break;
		}
	}

	if (matched == NULL)
		return 0.0f;

	TRACE("DesignWare MMC controller found (%s)\n", matched);
	return 0.8f;
}


status_t
init_bus(device_node* node, void** bus_cookie)
{
	CALLED();

	// Get the FDT driver and device of the controller
	device_node* parent = gDeviceManager->get_parent_node(node);
	device_node* fdtParent = gDeviceManager->get_parent_node(parent);

	fdt_device_module_info* fdt;
	fdt_device* fdtDevice;
	gDeviceManager->get_driver(fdtParent, (driver_module_info**)&fdt,
		(void**)&fdtDevice);
	gDeviceManager->put_node(fdtParent);
	gDeviceManager->put_node(parent);

	uint64 regs, regsLen;
	if (!fdt->get_reg(fdtDevice, 0, &regs, &regsLen)) {
		ERROR("No registers in device tree node\n");
		return B_ERROR;
	}
	TRACE("registers: (0x%" B_PRIx64 ", 0x%" B_PRIx64 ")\n", regs, regsLen);

	// Optional FIFO depth from the device tree (in 32-bit words)
	uint32_t fifoDepth = 0;
	int propLen;
	const void* prop = fdt->get_prop(fdtDevice, "fifo-depth", &propLen);
	if (prop != NULL && propLen >= 4)
		fifoDepth = B_BENDIAN_TO_HOST_INT32(*(const uint32_t*)prop);

	// We operate in polling mode, the interrupt is only used for reference
	uint64 interrupt = 0;
	device_node* interruptController;
	if (!fdt->get_interrupt(fdtDevice, 0, &interruptController, &interrupt))
		interrupt = 0;
	uint8_t irq = (uint8_t)(interrupt & 0xff);
	TRACE("interrupt: %u\n", irq);

	area_id regsArea;
	volatile uint32_t* _regs;
	regsArea = map_physical_memory("dwmmc_regs_map", regs, regsLen,
		B_ANY_KERNEL_BLOCK_ADDRESS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA,
		(void**)&_regs);

	if (regsArea < B_OK) {
		ERROR("Could not map registers\n");
		return B_BAD_VALUE;
	}

	DwMmcBus* bus = new(std::nothrow) DwMmcBus(_regs, irq, regsArea,
		fifoDepth);

	status_t status = B_NO_MEMORY;
	if (bus != NULL)
		status = bus->InitCheck();

	if (status != B_OK) {
		if (bus != NULL)
			delete bus;
		else
			delete_area(regsArea);
		return status;
	}

	// Store the created object as a cookie, allowing users of the bus to
	// locate it.
	*bus_cookie = bus;

	return status;
}


module_dependency module_dependencies[] = {
	{ MMC_BUS_MODULE_NAME, (module_info**)&gMMCBusController },
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&gDeviceManager },
	{}
};


status_t
set_clock(void* controller, uint32_t kilohertz)
{
	DwMmcBus* bus = (DwMmcBus*)controller;
	return bus->SetClock(kilohertz);
}


status_t
execute_command(void* controller, uint8_t command, uint32_t argument,
	uint32_t* response)
{
	DwMmcBus* bus = (DwMmcBus*)controller;
	return bus->ExecuteCommand(command, argument, response);
}


status_t
do_io(void* controller, uint8_t command, IOOperation* operation,
	bool offsetAsSectors)
{
	DwMmcBus* bus = (DwMmcBus*)controller;
	return bus->DoIO(command, operation, offsetAsSectors);
}


void
set_scan_semaphore(void* controller, sem_id sem)
{
	DwMmcBus* bus = (DwMmcBus*)controller;
	return bus->SetScanSemaphore(sem);
}


void
set_bus_width(void* controller, int width)
{
	DwMmcBus* bus = (DwMmcBus*)controller;
	return bus->SetBusWidth(width);
}


void
set_card_type(void* controller, card_type type)
{
	DwMmcBus* bus = (DwMmcBus*)controller;
	bus->SetCardType(type);
}


void
terminate_bus(void* controller)
{
	DwMmcBus* bus = (DwMmcBus*)controller;
	bus->TerminateBus();
}


// Root device that binds to the FDT bus. It will register an mmc_bus_interface
// node for the SD/MMC slot in the device.
static driver_module_info sDWMMCDevice = {
	{
		DWMMC_DEVICE_MODULE_NAME,
		0,
		NULL
	},
	supports_device,
	register_device,
	init_device,
	uninit_device,
	register_child_devices,
	NULL,	// rescan
	NULL,	// device removed
};


// Device node registered for the SD slot. It implements the MMC operations so
// the bus manager can use it to communicate with SD cards.
mmc_bus_interface gDWMMCBusModule = {
	.info = {
		.info = {
			.name = DWMMC_MMC_BUS_MODULE_NAME,
		},

		.init_driver = init_bus,
		.uninit_driver = uninit_bus,
		.device_removed = bus_removed,
	},

	.set_clock = set_clock,
	.execute_command = execute_command,
	.do_io = do_io,
	.set_scan_semaphore = set_scan_semaphore,
	.set_bus_width = set_bus_width,
	.terminate_bus = terminate_bus,
	.set_card_type = set_card_type,
};


module_info* modules[] = {
	(module_info* )&sDWMMCDevice,
	(module_info* )&gDWMMCBusModule,
	NULL
};