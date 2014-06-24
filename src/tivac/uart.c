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


   uart.c: UART access routines for LPC17xx

*/

#include "config.h"
#include "uart.h"

#include <stdbool.h>
#include <stdint.h>
#include "tw/hw_types.h"
#include "tw/hw_gpio.h"
#include "tw/hw_ints.h"
#include "tw/gpio.h"
#include "tw/uart.h"
#include "tw/sysctl.h"
#include "tw/rom.h"

static char txbuf[1 << CONFIG_UART_TX_BUF_SHIFT];
static volatile unsigned int read_idx,write_idx;

void UART0_Handler(void)
{
	uint32_t status = ROM_UARTIntStatus(UART0_BASE, true);

	if (status & UART_INT_TX) {
		while (read_idx != write_idx && ROM_UARTSpaceAvail(UART0_BASE)) {
			ROM_UARTCharPut(UART0_BASE, txbuf[read_idx]);
			read_idx = (read_idx+1) & (sizeof(txbuf)-1);
		}
		if (read_idx == write_idx) {
			ROM_UARTIntDisable(UART0_BASE, UART_INT_TX);
		}
	}
}

void uart_putc(char c) {
  unsigned int tmp = (write_idx+1) & (sizeof(txbuf)-1) ;

  if (read_idx == write_idx && ROM_UARTSpaceAvail(UART0_BASE)) {
    /* buffer empty, THR empty -> send immediately */
	  ROM_UARTCharPut(UART0_BASE, c);
  } else {
#ifdef CONFIG_DEADLOCK_ME_HARDER
    while (tmp == read_idx) ;
#endif

    ROM_UARTIntDisable(UART0_BASE, UART_INT_TX);
    txbuf[write_idx] = c;
    write_idx = tmp;
    ROM_UARTIntEnable(UART0_BASE, UART_INT_TX);
  }
}

/* Just for printf */
void uart_putchar(char c) {
  if (c == '\n')
    uart_putc('\r');
  uart_putc(c);
}

/* Polling version only */
unsigned char uart_getc(void) {
  return ROM_UARTCharGet(UART0_BASE);
}

/* Returns true if a char is ready */
unsigned char uart_gotc(void) {
  return ROM_UARTCharsAvail(UART0_BASE);
}

void uart_init(void) {
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(),
			CONFIG_UART_BAUDRATE, UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
	ROM_UARTFIFOEnable(UART0_BASE);
	ROM_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
	ROM_UARTEnable(UART0_BASE);

	ROM_IntEnable(INT_UART0);
}

/* --- generic code below --- */
void uart_puthex(uint8_t num) {
  uint8_t tmp;
  tmp = (num & 0xf0) >> 4;
  if (tmp < 10)
    uart_putc('0'+tmp);
  else
    uart_putc('a'+tmp-10);

  tmp = num & 0x0f;
  if (tmp < 10)
    uart_putc('0'+tmp);
  else
    uart_putc('a'+tmp-10);
}

void uart_trace(void *ptr, uint16_t start, uint16_t len) {
  uint16_t i;
  uint8_t j;
  uint8_t ch;
  uint8_t *data = ptr;

  data+=start;
  for(i=0;i<len;i+=16) {

    uart_puthex(start>>8);
    uart_puthex(start&0xff);
    uart_putc('|');
    uart_putc(' ');
    for(j=0;j<16;j++) {
      if(i+j<len) {
        ch=*(data + j);
        uart_puthex(ch);
      } else {
        uart_putc(' ');
        uart_putc(' ');
      }
      uart_putc(' ');
    }
    uart_putc('|');
    for(j=0;j<16;j++) {
      if(i+j<len) {
        ch=*(data++);
        if(ch<32 || ch>0x7e)
          ch='.';
        uart_putc(ch);
      } else {
        uart_putc(' ');
      }
    }
    uart_putc('|');
    uart_putcrlf();
    start+=16;
  }
}

void uart_flush(void) {
  while (read_idx != write_idx) ;
}

void uart_puts(const char *text) {
  while (*text) {
    uart_putc(*text++);
  }
}

void uart_puts_P(const char *text) {
  uart_puts(text);
}

void uart_putcrlf(void) {
  uart_putc(13);
  uart_putc(10);
}
