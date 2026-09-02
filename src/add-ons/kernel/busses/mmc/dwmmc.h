/*
 * Copyright 2026 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _DWMMC_H
#define _DWMMC_H


#include <device_manager.h>

#include <KernelExport.h>

#include "mmc.h"


// Register map of the DesignWare MMC (dw_mmc) host controller. All offsets
// are byte offsets into the controller MMIO range.
#define DWMCI_CTRL			0x000
#define DWMCI_PWREN			0x004
#define DWMCI_CLKDIV			0x008
#define DWMCI_CLKSRC			0x00c
#define DWMCI_CLKENA			0x010
#define DWMCI_TMOUT			0x014
#define DWMCI_CTYPE			0x018
#define DWMCI_BLKSIZ			0x01c
#define DWMCI_BYTCNT			0x020
#define DWMCI_INTMASK			0x024
#define DWMCI_CMDARG			0x028
#define DWMCI_CMD			0x02c
#define DWMCI_RESP0			0x030
#define DWMCI_RESP1			0x034
#define DWMCI_RESP2			0x038
#define DWMCI_RESP3			0x03c
#define DWMCI_MINTSTS			0x040
#define DWMCI_RINTSTS			0x044
#define DWMCI_STATUS			0x048
#define DWMCI_FIFOTH			0x04c
#define DWMCI_CDETECT			0x050
#define DWMCI_WRTPRT			0x054
#define DWMCI_GPIO			0x058
#define DWMCI_TCMCNT			0x05c
#define DWMCI_TBBCNT			0x060
#define DWMCI_DEBNCE			0x064
#define DWMCI_USRID			0x068
#define DWMCI_VERID			0x06c
#define DWMCI_HCON			0x070
#define DWMCI_UHS_REG			0x074
#define DWMCI_BMOD			0x080
#define DWMCI_PLDMND			0x084
#define DWMCI_DATA			0x200

// Raw interrupt status bits
#define DWMCI_INTMSK_RE			(1 << 1)
#define DWMCI_INTMSK_CDONE		(1 << 2)
#define DWMCI_INTMSK_DTO		(1 << 3)
#define DWMCI_INTMSK_TXDR		(1 << 4)
#define DWMCI_INTMSK_RXDR		(1 << 5)
#define DWMCI_INTMSK_RCRC		(1 << 6)
#define DWMCI_INTMSK_DCRC		(1 << 7)
#define DWMCI_INTMSK_RTO		(1 << 8)
#define DWMCI_INTMSK_DRTO		(1 << 9)
#define DWMCI_INTMSK_HTO		(1 << 10)
#define DWMCI_INTMSK_FRUN		(1 << 11)
#define DWMCI_INTMSK_HLE		(1 << 12)
#define DWMCI_INTMSK_SBE		(1 << 13)
#define DWMCI_INTMSK_ACD		(1 << 14)
#define DWMCI_INTMSK_EBE		(1 << 15)

#define DWMCI_DATA_ERR	(DWMCI_INTMSK_EBE | DWMCI_INTMSK_SBE \
	| DWMCI_INTMSK_HLE | DWMCI_INTMSK_FRUN | DWMCI_INTMSK_DCRC)
#define DWMCI_DATA_TOUT	(DWMCI_INTMSK_HTO | DWMCI_INTMSK_DRTO)

// CTRL register
#define DWMCI_CTRL_RESET		(1 << 0)
#define DWMCI_CTRL_FIFO_RESET		(1 << 1)
#define DWMCI_CTRL_DMA_RESET		(1 << 2)
#define DWMCI_DMA_EN			(1 << 5)
#define DWMCI_CTRL_SEND_AS_CCSD		(1 << 10)
#define DWMCI_IDMAC_EN			(1 << 25)
#define DWMCI_RESET_ALL	(DWMCI_CTRL_RESET | DWMCI_CTRL_FIFO_RESET \
	| DWMCI_CTRL_DMA_RESET)

// CMD register
#define DWMCI_CMD_CMD_INDEX(command)	((command) & 0x3f)
#define DWMCI_CMD_RESP_EXP		(1 << 6)
#define DWMCI_CMD_RESP_LENGTH		(1 << 7)
#define DWMCI_CMD_CHECK_CRC		(1 << 8)
#define DWMCI_CMD_DATA_EXP		(1 << 9)
#define DWMCI_CMD_RW			(1 << 10)
#define DWMCI_CMD_SEND_STOP		(1 << 12)
#define DWMCI_CMD_PRV_DAT_WAIT		(1 << 13)
#define DWMCI_CMD_ABORT_STOP		(1 << 16)
#define DWMCI_CMD_UPD_CLK		(1 << 21)
#define DWMCI_CMD_USE_HOLD_REG		(1 << 29)
#define DWMCI_CMD_START			(1 << 31)

// CLKENA register
#define DWMCI_CLKEN_ENABLE		(1 << 0)
#define DWMCI_CLKEN_LOW_PWR		(1 << 16)

// CTYPE register
#define DWMCI_CTYPE_4BIT		(1 << 0)
#define DWMCI_CTYPE_8BIT		(1 << 16)

// STATUS register
#define DWMCI_FIFO_EMPTY		(1 << 2)
#define DWMCI_FIFO_FULL			(1 << 3)
#define DWMCI_BUSY			(1 << 9)
#define DWMCI_FIFO_MASK			0x1fff
#define DWMCI_FIFO_SHIFT		17

// FIFOTH register
#define DWMCI_MSIZE(x)			((x) << 28)
#define DWMCI_RX_WMARK(x)		((x) << 16)
#define DWMCI_RX_WMARK_SHIFT		16
#define DWMCI_RX_WMARK_MASK		(0xfff << DWMCI_RX_WMARK_SHIFT)
#define DWMCI_TX_WMARK(x)		((x) & 0xfff)

// BMOD register (internal DMA; unused, we operate in polling mode)
#define DWMCI_BMOD_IDMAC_RESET		(1 << 0)


class DwMmcBus {
	public:
								DwMmcBus(volatile uint32_t* registers,
									uint8_t irq, area_id regsArea,
									uint32_t fifoDepth);
								~DwMmcBus();

			status_t			InitCheck();
			status_t			SetClock(int kilohertz);
			status_t			ExecuteCommand(uint8_t command,
									uint32_t argument, uint32_t* response);
			status_t			DoIO(uint8_t command, IOOperation* operation,
									bool offsetAsSectors);
			void				SetScanSemaphore(sem_id sem);
			void				SetBusWidth(int width);
			void				SetCardType(card_type type);
			void				TerminateBus();

	private:
			uint32_t			Read(uint32_t registerOffset);
			void				Write(uint32_t registerOffset, uint32_t value);
			void				ClearInterrupts();

			status_t			_WaitReset(uint32_t value);
			status_t			_WaitForIdle(bigtime_t timeout);
			status_t			_WaitCommandComplete(bigtime_t timeout);
			status_t			_UpdateClock();
			status_t			_ReadFIFO(uint8_t* buffer, size_t length);
			status_t			_WriteFIFO(uint8_t* buffer, size_t length);
			status_t			_TransferData(uint32_t command,
									uint32_t argument, uint8_t* buffer,
									size_t length, bool isWrite);

			volatile uint32_t*	fRegisters;
			uint8_t				fIrq;
			area_id				fRegsArea;
			card_type			fCardType;
			status_t			fStatus;
			uint32_t			fFifoDepth;
			sem_id				fScanSemaphore;
};


#endif // _DWMMC_H