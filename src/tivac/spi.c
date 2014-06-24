/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2013  Ingo Korb <ingo@akana.de>
   Copyright (C) 2014  Fredrik Ahlberg <fredrik@z80.se>

   Inspired by MMC2IEC by Lars Pontoppidan et al.

   FAT filesystem access based on code from ChaN and Jim Brain, see ff.c|h.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


   spi.c: Low-level SPI routines

*/

#include "config.h"
#include "spi.h"

#include <stdbool.h>
#include <stdint.h>
#include "tw/hw_types.h"
#include "tw/hw_gpio.h"
#include "tw/pin_map.h"
#include "tw/ssi.h"
#include "tw/gpio.h"
#include "tw/sysctl.h"
#include "tw/rom.h"

void spi_init(spi_speed_t speed) {
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE+GPIO_O_CR) = 0xff;

	ROM_GPIOPinConfigure(GPIO_PF0_SSI1RX);
	ROM_GPIOPinConfigure(GPIO_PF1_SSI1TX);
	ROM_GPIOPinConfigure(GPIO_PF2_SSI1CLK);

	ROM_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
	ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

	HWREG(GPIO_PORTF_BASE+GPIO_O_CR) = 0;
	HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0;

	/* deassert CSn */
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

	ROM_SSIConfigSetExpClk(SSI1_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_MASTER, speed == SPI_SPEED_FAST ? 16e6 : 400e3, 8);

	ROM_SSIEnable(SSI1_BASE);
}

void spi_tx_byte(uint8_t data) {
	uint32_t dummy;
	ROM_SSIDataPut(SSI1_BASE, data);
	ROM_SSIDataGet(SSI1_BASE, &dummy);
}

uint8_t spi_rx_byte(void) {
	uint32_t data;
	ROM_SSIDataPut(SSI1_BASE, 0xff);
	ROM_SSIDataGet(SSI1_BASE, &data);
	return data;
}

void spi_tx_block(const void *ptr, unsigned int length) {
  const uint8_t *data = (const uint8_t *)ptr;

  while (length--) {
	  ROM_SSIDataPut(SSI1_BASE, *data++);
  }
}

void spi_rx_block(void *ptr, unsigned int length) {
  uint8_t *data = (uint8_t *)ptr;
  uint32_t d;

	while (length > 0) {
		ROM_SSIDataPut(SSI1_BASE, 0xff);
		ROM_SSIDataGet(SSI1_BASE, &d);
		*data++ = d;
		length--;
	}

#if 0
  uint8_t *data = (uint8_t *)ptr;
  unsigned int txlen = length;

  /* Wait until SSP is not busy */
  while (BITBAND(SSP_REGS->SR, SSP_BSY)) ;

  /* Clear RX fifo */
  while (BITBAND(SSP_REGS->SR, SSP_RNE))
    (void) SSP_REGS->DR;

  if ((length & 3) != 0 || ((uint32_t)ptr & 3) != 0) {
    /* Odd length or unaligned buffer */
    while (length > 0) {
      /* Wait until TX or RX FIFO are ready */
      while (txlen > 0 && !BITBAND(SSP_REGS->SR, SSP_TNF) &&
             !BITBAND(SSP_REGS->SR, SSP_RNE)) ;

      /* Try to receive data */
      while (length > 0 && BITBAND(SSP_REGS->SR, SSP_RNE)) {
        *data++ = SSP_REGS->DR;
        length--;
      }

      /* Send dummy data until TX full or RX ready */
      while (txlen > 0 && BITBAND(SSP_REGS->SR, SSP_TNF) && !BITBAND(SSP_REGS->SR, SSP_RNE)) {
        txlen--;
        SSP_REGS->DR = 0xff;
      }
    }
  } else {
    /* Clear interrupt flags of DMA channels 0 */
    LPC_GPDMA->DMACIntTCClear = BV(0);
    LPC_GPDMA->DMACIntErrClr  = BV(0);

    /* Set up RX DMA channel */
    LPC_GPDMACH0->DMACCSrcAddr  = (uint32_t)&SSP_REGS->DR;
    LPC_GPDMACH0->DMACCDestAddr = (uint32_t)ptr;
    LPC_GPDMACH0->DMACCLLI      = 0; // no linked list
    LPC_GPDMACH0->DMACCControl  = length
      | (0 << 12) // source burst size 1
      | (0 << 15) // destination burst size 1
      | (0 << 18) // source transfer width 1 byte
      | (2 << 21) // destination transfer width 4 bytes
      | (0 << 26) // source address not incremented
      | (1 << 27) // destination address incremented
      ;
    LPC_GPDMACH0->DMACCConfig = 1 // enable channel
      | (SSP_DMAID_RX << 1) // data source SSP RX
      | (2 << 11) // transfer from peripheral to memory
      ;

    /* Enable RX FIFO DMA */
    SSP_REGS->DMACR = 1;

    /* Write <length> bytes into TX FIFO */
    while (txlen > 0) {
      while (txlen > 0 && BITBAND(SSP_REGS->SR, SSP_TNF)) {
        txlen--;
        SSP_REGS->DR = 0xff;
      }
    }

    /* Wait until DMA channel disables itself */
    while (LPC_GPDMACH0->DMACCConfig & 1) ;

    /* Disable RX FIFO DMA */
    SSP_REGS->DMACR = 0;
  }
#endif
}

void spi_set_speed(spi_speed_t speed) {
  while (ROM_SSIBusy(SSI1_BASE)) ;

	ROM_SSIDisable(SSI1_BASE);

	ROM_SSIConfigSetExpClk(SSI1_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_MASTER, speed == SPI_SPEED_FAST ? 16e6 : 400e3, 8);

	ROM_SSIEnable(SSI1_BASE);
}

void spi_select_device(spi_device_t dev) {
  /* Wait until TX fifo is empty */
  while (ROM_SSIBusy(SSI1_BASE)) ;

  if (dev == SPIDEV_CARD0 || dev == SPIDEV_ALLCARDS)
    sdcard_set_ss(0);
  else
    sdcard_set_ss(1);

#ifdef CONFIG_TWINSD
  if (dev == SPIDEV_CARD1 || dev == SPIDEV_ALLCARDS)
    sdcard2_set_ss(0);
  else
    sdcard2_set_ss(1);
#endif
}
