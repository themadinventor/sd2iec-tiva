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


   arch-timer.c: Architecture-specific timer functions

*/

#include "config.h"
#include "timer.h"

#include "tw/timer.h"
#include "tw/hw_timer.h"
#include "tw/hw_ints.h"

void timer_init(void) {
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
	ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
	
	ROM_SysTickPeriodSet(ROM_SysCtlClockGet()/100);
	ROM_SysTickIntEnable();
	ROM_SysTickEnable();
	ROM_IntEnable(FAULT_SYSTICK);
}

void delay_us(unsigned int time) {
	//ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, time * 80);
	HWREG(TIMER0_BASE + TIMER_O_TAILR) = time * 80;
	HWREGBITW(TIMER0_BASE + TIMER_O_ICR, 0) = 1; // clear TATO
	//ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	HWREG(TIMER0_BASE + TIMER_O_CTL) |= TIMER_A;

	while (!HWREGBITW(TIMER0_BASE + TIMER_O_RIS, 0)) ; // wait for TATORIS
}

void delay_ms(unsigned int time) {
	//ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, time * 80000);
	HWREG(TIMER0_BASE + TIMER_O_TAILR) = time * 80000;
	HWREGBITW(TIMER0_BASE + TIMER_O_ICR, 0) = 1; // clear TATO
	//ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	HWREG(TIMER0_BASE + TIMER_O_CTL) |= TIMER_A;

	while (!HWREGBITW(TIMER0_BASE + TIMER_O_RIS, 0)) ; // wait for TATORIS
}

/**
 * start_timeout - start a timeout
 * @usecs: number of microseconds before timeout
 *
 * This function sets up a timer so it times out after the specified
 * number of microseconds.
 */
void start_timeout(unsigned int usecs) {
	//ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, usecs * 80);
	HWREG(TIMER1_BASE + TIMER_O_TAILR) = usecs * 80;
	HWREGBITW(TIMER1_BASE + TIMER_O_ICR, 0) = 1; // clear TATO
	//ROM_TimerEnable(TIMER1_BASE, TIMER_A);
	HWREG(TIMER1_BASE + TIMER_O_CTL) |= TIMER_A;
}

/**
 * has_timed_out - returns true if timeout was reached
 *
 * This function returns true if the timer started by start_timeout
 * has reached its timeout value.
 */
unsigned int has_timed_out(void) {
	return HWREGBITW(TIMER1_BASE + TIMER_O_RIS, 0) ; // wait for TATORIS
}
