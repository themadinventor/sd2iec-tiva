# architecture-dependent variables

#---------------- Source code ----------------
ASMSRC = tivac/startup.S tivac/crc.S

SRC += tivac/fastloader-ll.c tivac/iec-bus.c

ifeq ($(CONFIG_UART_DEBUG),y)
  SRC += tivac/printf.c
endif

# Various RTC implementations
ifeq ($(CONFIG_RTC_LPC17XX),y)
  SRC += rtc.c lpc17xx/rtc_lpc17xx.c
endif

# I2C is always needed for the config EEPROM
NEED_I2C := n
SRC += tivac/arch-eeprom.c

#---------------- Toolchain ----------------
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size
NM = arm-none-eabi-nm


#---------------- Bootloader ----------------
BINARY_LENGTH = 0x80000
CRCGEN        = scripts/lpc17xx/crcgen-lpc.pl


#---------------- Architecture variables ----------------
ARCH_CFLAGS  = -mthumb -mcpu=cortex-m4 -nostartfiles
ARCH_ASFLAGS = -mthumb -mcpu=cortex-m4
ARCH_LDFLAGS = -Tscripts/tivac/$(CONFIG_MCU).ld

#---------------- Config ----------------
# currently no stack tracking supported
