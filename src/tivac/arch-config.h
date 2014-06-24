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


   arch-config.h: The main architecture-specific config header

*/

#ifndef ARCH_CONFIG_H
#define ARCH_CONFIG_H

#define TARGET_IS_BLIZZARD_RB1
#define PART_TM4C123GH6PM

#include <stdbool.h>
#include <stdint.h>
#include "tw/hw_types.h"
#include "tw/hw_memmap.h"
#include "tw/hw_gpio.h"
#include "tw/hw_ints.h"
#include "tw/sysctl.h"
#include "tw/gpio.h"
#include "tw/rom.h"

/*
 * PA0,PA1:	Debug UART
 *
 * PA2:		IEC ATN input
 * PA3:		IEC CLOCK input
 * PA4:		IEC DATA input
 * PA5:		IEC SRQ input
 *
 * PB0:		IEC ATN output
 * PB1:		IEC CLOCK output
 * PB2:		IEC DATA output
 * PB3:		IEC SRQ output
 *
 * PF4:		Busy LED
 *
 */

/* Return value of buttons_read() */
typedef unsigned int rawbutton_t;

/* Interrupt handler for system tick */
#define SYSTEM_TICK_HANDLER void SysTick_Handler(void)

/* The GPIO interrupt is demuxed in system.c, function names are fixed */
#define SD_CHANGE_HANDLER  void sdcard_change_handler(void)
#define IEC_ATN_HANDLER    void iec_atn_handler(void)
#define IEC_CLOCK_HANDLER  void iec_clock_handler(void)
#define PARALLEL_HANDLER   void parallel_handler(void)

static inline void device_hw_address_init(void) {
  // Nothing, pins are input+pullups by default
}


// FIXME: Add a fully-commented example configuration that
//        demonstrates all configuration possilibilites

#  define HAVE_SD
#  define SD_SUPPLY_VOLTAGE (1L<<21)
#  define SPI_ON_SSP 0

/* SSI1, PF0..PF2, SD0-CS PF3 */
/* Detect PG0 */
static inline void sdcard_interface_init(void) {
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

	ROM_GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_0);

	HWREG(GPIO_PORTG_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTG_BASE+GPIO_O_CR) = 0xff;
	ROM_GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
	HWREG(GPIO_PORTG_BASE+GPIO_O_CR) = 0;
	HWREG(GPIO_PORTG_BASE+GPIO_O_LOCK) = 0;
}

static inline void sdcard_set_ss(int state) {
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, state ? GPIO_PIN_3 : 0);
}

static inline uint8_t sdcard_detect(void) {
	return !ROM_GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_0);
}

static inline uint8_t sdcard_wp(void) {
	return 0;
}

static inline uint8_t device_hw_address(void) {
	return 8;
#if 0
  return 8 + BITBAND(LPC_GPIO2->FIOPIN, 5) + 2*BITBAND(LPC_GPIO2->FIOPIN, 4);
#endif
}

//#define HAVE_SD_LED

#define SINGLE_LED

static inline void leds_init(void) {
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
}

//static inline __attribute__((always_inline)) void set_busy_led(uint8_t state) {
static inline __attribute__((always_inline)) void set_led(uint8_t state) {
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, state ? GPIO_PIN_4 : 0);
}

#if 0
static inline __attribute__((always_inline)) void set_dirty_led(uint8_t state) {
}

static inline __attribute__((always_inline)) void set_test_led(uint8_t state) {
}

static inline __attribute__((always_inline)) void set_sd_led(uint8_t state) {
}

static inline void toggle_dirty_led(void) {
}
#endif

static inline void toggle_led(void) {
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4,
			~ROM_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4));
}

/* IEC input bits */
#  define IEC_IN_BASE		GPIO_PORTA_BASE
#  define IEC_INPUT             HWREG(IEC_IN_BASE+GPIO_O_DATA+((IEC_PIN_ATN|IEC_PIN_CLOCK|IEC_PIN_DATA|IEC_PIN_SRQ)<<2))
//#  define IEC_INPUTS_INVERTED
#  define IEC_PIN_ATN           GPIO_PIN_2
#  define IEC_PIN_CLOCK         GPIO_PIN_3
#  define IEC_PIN_DATA          GPIO_PIN_4
#  define IEC_PIN_SRQ           GPIO_PIN_5
//#  define IEC_TIMER_ATN         LPC_TIM3
//#  define IEC_TIMER_CLOCK       LPC_TIM2
//#  define IEC_TIMER_DATA        LPC_TIM2
//#  define IEC_TIMER_SRQ         LPC_TIM3
//#  define IEC_IBIT_ATN       2
//#  define IEC_IBIT_CLOCK     3
//#  define IEC_IBIT_DATA      4
//#  define IEC_IBIT_SRQ       5

/* IEC output bits - must be EMR of the same timer used for input */
/* optionally all lines can be on timer 2 (4 match registers) */
//#  define IEC_OUTPUTS_INVERTED
//#  define IEC_ALL_MATCHES_ON_TIMER2
#  define IEC_OUT_BASE		GPIO_PORTB_BASE
//#  define IEC_OUTPUT		HWREG(IEC_OUT_BASE+GPIO_O_DATA)
#  define IEC_OPIN_ATN          GPIO_PIN_0
#  define IEC_OPIN_CLOCK        GPIO_PIN_1
#  define IEC_OPIN_DATA         GPIO_PIN_2
#  define IEC_OPIN_SRQ          GPIO_PIN_3
//#  define IEC_OBIT_ATN         0
//#  define IEC_OBIT_CLOCK       1
//#  define IEC_OBIT_DATA        2
//#  define IEC_OBIT_SRQ         3
/* name the two IEC timers as A and B (simplifies initialisation), A is the main reference */
/* Note: timer A is also used for timeouts */
//#  define IEC_TIMER_A           LPC_TIM2
//#  define IEC_TIMER_B           LPC_TIM3
/* interrupts */
//#  define IEC_TIMER_A_IRQn      TIMER2_IRQn
//#  define IEC_TIMER_B_IRQn      TIMER3_IRQn
//#  define IEC_TIMER_A_HANDLER   TIMER2_IRQHandler
//#  define IEC_TIMER_B_HANDLER   TIMER3_IRQHandler
/* LPC_SC->PCONP bits */
//#  define IEC_TIMER_A_PCONBIT   22
//#  define IEC_TIMER_B_PCONBIT   23
/* PCLKSELx registers */
//#  define IEC_TIMER_A_PCLKREG   PCLKSEL1
//#  define IEC_TIMER_B_PCLKREG   PCLKSEL1
/* PCLKSELx bits for 1:1 prescaler */
//#  define IEC_TIMER_A_PCLKBIT   12
//#  define IEC_TIMER_B_PCLKBIT   14

/* timeout timer - one of the two remaining ones */
/*#  define TIMEOUT_TIMER         LPC_TIM0
#  define TIMEOUT_TIMER_PCONBIT 1
#  define TIMEOUT_TIMER_PCLKREG PCLKSEL0
#  define TIMEOUT_TIMER_PCLKBIT 2*/

static inline void iec_pins_connect(void) {
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, IEC_PIN_ATN|IEC_PIN_DATA|IEC_PIN_CLOCK);

	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, IEC_OPIN_ATN|IEC_OPIN_DATA|IEC_OPIN_CLOCK);
	ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, IEC_OPIN_ATN|IEC_OPIN_DATA|IEC_OPIN_CLOCK,
			GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_OD);
}

static inline void iec_interrupts_init(void) {
	ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, IEC_PIN_ATN|IEC_PIN_CLOCK, GPIO_BOTH_EDGES);
	ROM_IntEnable(INT_GPIOA);
}

#  define BUTTON_NEXT           1 //BV(3)
#  define BUTTON_PREV           2 //BV(2)

static inline rawbutton_t buttons_read(void) {
  return 0xff;
}

static inline void buttons_init(void) {
  // None
}

#  define I2C_NUMBER          1
#  define I2C_PCLKDIV         1
#  define I2C_CLOCK           100000
#  define I2C_EEPROM_ADDRESS  0xa0
#  define I2C_EEPROM_SIZE     256
#  define I2C_EEPROM_PAGESIZE 8

static inline __attribute__((always_inline)) void i2c_pins_connect(void) {
}

#  define UART_NUMBER 0

static inline __attribute__((always_inline)) void uart_pins_connect(void) {
}

/* ---------------- End of user-configurable options ---------------- */

/* GPIO interrupt port-to-array-offset */
#ifdef SD_CHANGE_ON_GPIO2
#  define SD_CHANGE_GPIOINT 1
#else
#  define SD_CHANGE_GPIOINT 0
#endif

#ifdef PARALLEL_HSK_ON_GPIO2
#  define PARALLEL_HSK_GPIOINT 1
#else
#  define PARALLEL_HSK_GPIOINT 0
#endif

/* Bit number to bit value, used in iec_bus_read() */
#define IEC_BIT_ATN      GPIO_PIN_2
#define IEC_BIT_DATA     GPIO_PIN_4
#define IEC_BIT_CLOCK    GPIO_PIN_3
#define IEC_BIT_SRQ	 GPIO_PIN_5

/* Return type of iec_bus_read() */
typedef uint32_t iec_bus_t;

/* Shortcut input macros using the CM3 bitband feature */
#ifdef IEC_INPUTS_INVERTED
#  define IEC_IN_COND_INV(x) (!(x))
#else
#  define IEC_IN_COND_INV(x) (x)
#endif

//#define IEC_ATN   IEC_IN_COND_INV(HWREGBITW(IEC_IN_BASE, IEC_IBIT_ATN))
//#define IEC_CLOCK IEC_IN_COND_INV(HWREGBITW(IEC_IN_BASE, IEC_IBIT_CLOCK))
//#define IEC_DATA  IEC_IN_COND_INV(HWREGBITW(IEC_IN_BASE, IEC_IBIT_DATA))
//#define IEC_SRQ   IEC_IN_COND_INV(HWREGBITW(IEC_IN_BASE, IEC_IBIT_SRQ))

#define IEC_ATN   IEC_IN_COND_INV(HWREG(IEC_IN_BASE + GPIO_O_DATA + (IEC_PIN_ATN << 2)))
#define IEC_CLOCK IEC_IN_COND_INV(HWREG(IEC_IN_BASE + GPIO_O_DATA + (IEC_PIN_CLOCK << 2)))
#define IEC_DATA  IEC_IN_COND_INV(HWREG(IEC_IN_BASE + GPIO_O_DATA + (IEC_PIN_DATA << 2)))
#define IEC_SRQ   IEC_IN_COND_INV(HWREG(IEC_IN_BASE + GPIO_O_DATA + (IEC_PIN_SRQ << 2)))


/* Generate match timer macros */
/*#ifdef IEC_ALL_MATCHES_ON_TIMER2
#  define IEC_MTIMER_ATN   LPC_TIM2
#  define IEC_MTIMER_CLOCK LPC_TIM2
#  define IEC_MTIMER_DATA  LPC_TIM2
#  define IEC_MTIMER_SRQ   LPC_TIM2
#else
#  define IEC_MTIMER_ATN   IEC_TIMER_ATN
#  define IEC_MTIMER_CLOCK IEC_TIMER_CLOCK
#  define IEC_MTIMER_DATA  IEC_TIMER_DATA
#  define IEC_MTIMER_SRQ   IEC_TIMER_SRQ
#endif*/

/* IEC output with bitband access */
#ifdef IEC_OUTPUTS_INVERTED
#  define COND_INV(x) (!(x))
#else
#  define COND_INV(x) (x)
#endif

static inline __attribute__((always_inline)) void set_atn(unsigned int state) {
	HWREG(IEC_OUT_BASE + GPIO_O_DATA + (IEC_OPIN_ATN << 2)) = COND_INV(state) ? IEC_OPIN_ATN : 0;
}

static inline __attribute__((always_inline)) void set_clock(unsigned int state) {
	HWREG(IEC_OUT_BASE + GPIO_O_DATA + (IEC_OPIN_CLOCK << 2)) = COND_INV(state) ? IEC_OPIN_CLOCK : 0;
}

static inline __attribute__((always_inline)) void set_data(unsigned int state) {
	HWREG(IEC_OUT_BASE + GPIO_O_DATA + (IEC_OPIN_DATA << 2)) = COND_INV(state) ? IEC_OPIN_DATA : 0;
}

static inline __attribute__((always_inline)) void set_srq(unsigned int state) {
	HWREG(IEC_OUT_BASE + GPIO_O_DATA + (IEC_OPIN_SRQ << 2)) = COND_INV(state) ? IEC_OPIN_SRQ : 0;
}

/* Enable/disable ATN interrupt */
static inline __attribute__((always_inline)) void set_atn_irq(uint8_t state) {
	/*if (state) {
		ROM_GPIOIntEnable(GPIO_PORTA_BASE, IEC_PIN_ATN);
	} else {
		ROM_GPIOIntDisable(GPIO_PORTA_BASE, IEC_PIN_ATN);
	}*/
	HWREGBITW(GPIO_PORTA_BASE + GPIO_O_IM, 2 /*IEC_BIT_ATN*/) = state;
}

/* Enable/disable CLOCK interrupt */
static inline __attribute__((always_inline)) void set_clock_irq(uint8_t state) {
	HWREGBITW(GPIO_PORTA_BASE + GPIO_O_IM, 3 /*IEC_BIT_CLOCK*/) = state;
	/*if (state) {
		ROM_GPIOIntEnable(GPIO_PORTA_BASE, IEC_PIN_CLOCK);
	} else {
		ROM_GPIOIntDisable(GPIO_PORTA_BASE, IEC_PIN_CLOCK);
	}*/
}
#define HAVE_CLOCK_IRQ

#undef COND_INV

#ifdef HAVE_PARALLEL
static inline void parallel_init(void) {
#if 0
  /* set HSK_OUT to output, open drain, weak-high (pullup is default-on) */
  PARALLEL_HGPIO->FIOPIN |= BV(PARALLEL_HSK_OUT_BIT);
  PARALLEL_HGPIO->FIODIR |= BV(PARALLEL_HSK_OUT_BIT);
  LPC_PINCON->PARALLEL_HOD |= BV(PARALLEL_HSK_OUT_BIT);
  LPC_PINCON->PARALLEL_POD |= 0xff << PARALLEL_PSTARTBIT;

# ifdef PARALLEL_HSK_ON_GPIO2
  LPC_GPIOINT->IO2IntEnF |= BV(PARALLEL_HSK_IN_BIT);
# else
  LPC_GPIOINT->IO0IntEnF |= BV(PARALLEL_HSK_IN_BIT);
# endif
#endif
}
#else
static inline void parallel_init(void) {}
#endif

/* Display interrupt request line */
// FIXME2: Init function!
static inline void display_intrq_init(void) {
}

// FIXME: Define and add!
static inline unsigned int display_intrq_active(void) {
  return 0;
}

#endif
