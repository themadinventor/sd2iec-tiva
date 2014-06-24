/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2013  Ingo Korb <ingo@akana.de>

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


   system.c: System-specific initialisation (LPC17xx version)

*/

#include "config.h"
#include "system.h"

/* Early system initialisation */
void system_init_early(void) {
	asm volatile ("cpsid i");

	ROM_SysCtlDelay(2000000);

	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3);
}

void i2c_init(void) { }

/* Late initialisation, increase CPU clock */
void system_init_late(void) {
}

/* Put MCU in low-power mode */
void system_sleep(void) {
	asm volatile ("wfi");
}

/* Reset MCU */
void system_reset(void) {
	asm volatile ("cpsid i");

#if 0
  /* force watchdog reset */
  LPC_WDT->WDTC = 256;            // minimal timeout
  LPC_WDT->WDCLKSEL = BV(31);     // internal RC, lock register
  LPC_WDT->WDMOD = BV(0) | BV(1); // enable watchdog and reset-by-watchdog
  LPC_WDT->WDFEED = 0xaa;
  LPC_WDT->WDFEED = 0x55;         // initial feed to really enable WDT
#endif

  while (1) ;
}

/* Disable interrupts */
void disable_interrupts(void) {
	asm volatile ("cpsid i");
	ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
}

/* Enable interrupts */
void enable_interrupts(void) {
	asm volatile ("cpsie i");
	ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, -1);
}

/*** Timer/GPIO interrupt demux ***/

/* Declare handler functions */
SD_CHANGE_HANDLER;
PARALLEL_HANDLER;
IEC_ATN_HANDLER;
IEC_CLOCK_HANDLER;

void GPIOA_Handler(void)
{
	uint32_t state = ROM_GPIOIntStatus(GPIO_PORTA_BASE);

	ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);

	if (state & IEC_PIN_ATN) {
		iec_atn_handler();
	}

#ifdef CONFIG_LOADER_DREAMLOADER
	if (state & IEC_PIN_CLOCK) {
		iec_clock_handler();
	}
#endif

	ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, -1);

	ROM_GPIOIntClear(GPIO_PORTA_BASE, state);
}

/* timer interrupts, used to detect IEC pin changes */
#if 0
void IEC_TIMER_A_HANDLER(void) {
  if (IEC_TIMER_ATN == IEC_TIMER_A) {
    if (BITBAND(IEC_TIMER_ATN->IR, 4 + IEC_CAPTURE_ATN)) {
      IEC_TIMER_ATN->IR = 1 << (4 + IEC_CAPTURE_ATN);
      iec_atn_handler();
    }
  }

#ifdef CONFIG_LOADER_DREAMLOAD
  if (IEC_TIMER_CLOCK == IEC_TIMER_A) {
    if (BITBAND(IEC_TIMER_CLOCK->IR, 4 + IEC_CAPTURE_CLOCK)) {
      IEC_TIMER_CLOCK->IR = 1 << (4 + IEC_CAPTURE_CLOCK);
      iec_clock_handler();
    }
  }
#endif
}

void IEC_TIMER_B_HANDLER(void) {
  if (IEC_TIMER_ATN == IEC_TIMER_B) {
    if (BITBAND(IEC_TIMER_ATN->IR, 4 + IEC_CAPTURE_ATN)) {
      IEC_TIMER_ATN->IR = 1 << (4 + IEC_CAPTURE_ATN);
      iec_atn_handler();
    }
  }

#ifdef CONFIG_LOADER_DREAMLOAD
  if (IEC_TIMER_CLOCK == IEC_TIMER_B) {
    if (BITBAND(IEC_TIMER_CLOCK->IR, 4 + IEC_CAPTURE_CLOCK)) {
      IEC_TIMER_CLOCK->IR = 1 << (4 + IEC_CAPTURE_CLOCK);
      iec_clock_handler();
    }
  }
#endif
}

/* GPIO interrupt handler, shared with EINT3 */
void EINT3_IRQHandler(void) {
  if (BITBAND(lpc_gpioint_ptr[SD_CHANGE_GPIOINT].IOIntStatF, SD_DETECT_PIN) ||
      BITBAND(lpc_gpioint_ptr[SD_CHANGE_GPIOINT].IOIntStatR, SD_DETECT_PIN)) {
    BITBAND(lpc_gpioint_ptr[SD_CHANGE_GPIOINT].IOIntClr, SD_DETECT_PIN) = 1;
    sdcard_change_handler();
  }

#ifdef PARALLEL_ENABLED
  if (BITBAND(lpc_gpioint_ptr[PARALLEL_HSK_GPIOINT].IOIntStatF, PARALLEL_HSK_IN_BIT)) {
    BITBAND(lpc_gpioint_ptr[PARALLEL_HSK_GPIOINT].IOIntClr, PARALLEL_HSK_IN_BIT) = 1;
    parallel_handler();
  }
#endif
}
#endif

/*void HardFault_Handler(void) {
  set_test_led(1);
  while (1) ;
}

void MemManage_Handler(void) {
  set_test_led(1);
  while (1) ;
}

void BusFault_Handler(void) {
  set_test_led(1);
  while (1);
}

void UsageFault_Handler(void) {
  set_test_led(1);
  while (1) ;
}*/
