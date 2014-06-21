/* startup code for LPC17xx
 *
 * Written 2010 by Ingo Korb, no copyright claimed
 */
#include "asmconfig.h"

        .syntax unified

        .section .vectors

        .macro  except label
        .weak   \label
        .set    \label, __unhandled_exception
        .word   \label
        .endm

        /* Cortex M3 standard except vectors */
        .word   __stack
        .word   _start
        except  NMI_Handler
        except  HardFault_Handler  /* 2 */
        except  MemManage_Handler
        except  BusFault_Handler   /* 4 */
        except  UsageFault_Handler
        .word   0
        .word   0
        .word   0
        .word   0
        except  SVC_Handler     /* 10 */
        except  DebugMon_Handler
        .word   0
        except  PendSV_Handler
        except  SysTick_Handler

        /* External interrupt vectors */
	except  GPIOA_Handler               /* GPIO Port A */
	except  GPIOB_Handler               /* GPIO Port B */
	except  GPIOC_Handler               /* GPIO Port C */
	except  GPIOD_Handler               /* GPIO Port D */
	except  GPIOE_Handler               /* GPIO Port E */
	except  UART0_Handler               /* UART0 Rx and Tx */
	except  UART1_Handler               /* UART1 Rx and Tx */
	except  SSI0_Handler                /* SSI0 Rx and Tx */
	except  I2C0_Handler                /* I2C0 Master and Slave */
	except  PMW0_FAULT_Handler          /* PWM Fault */
	except  PWM0_0_Handler              /* PWM Generator 0 */
	except  PWM0_1_Handler              /* PWM Generator 1 */
	except  PWM0_2_Handler              /* PWM Generator 2 */
	except  QEI0_Handler                /* Quadrature Encoder 0 */
	except  ADC0SS0_Handler             /* ADC Sequence 0 */
	except  ADC0SS1_Handler             /* ADC Sequence 1 */
	except  ADC0SS2_Handler             /* ADC Sequence 2 */
	except  ADC0SS3_Handler             /* ADC Sequence 3 */
	except  WDT0_Handler                /* Watchdog timer */
	except  TIMER0A_Handler             /* Timer 0 subtimer A */
	except  TIMER0B_Handler             /* Timer 0 subtimer B */
	except  TIMER1A_Handler             /* Timer 1 subtimer A */
	except  TIMER1B_Handler             /* Timer 1 subtimer B */
	except  TIMER2A_Handler             /* Timer 2 subtimer A */
	except  TIMER2B_Handler             /* Timer 2 subtimer B */
	except  COMP0_Handler               /* Analog Comparator 0 */
	except  COMP1_Handler               /* Analog Comparator 1 */
	except  COMP2_Handler               /* Analog Comparator 2 */
	except  SYSCTL_Handler              /* System Control (PLL, OSC, BO) */
	except  FLASH_Handler               /* FLASH Control */
	except  GPIOF_Handler               /* GPIO Port F */
	except  GPIOG_Handler               /* GPIO Port G */
	except  GPIOH_Handler               /* GPIO Port H */
	except  UART2_Handler               /* UART2 Rx and Tx */
	except  SSI1_Handler                /* SSI1 Rx and Tx */
	except  TIMER3A_Handler             /* Timer 3 subtimer A */
	except  TIMER3B_Handler             /* Timer 3 subtimer B */
	except  I2C1_Handler                /* I2C1 Master and Slave */
	except  QEI1_Handler                /* Quadrature Encoder 1 */
	except  CAN0_Handler                /* CAN0 */
	except  CAN1_Handler                /* CAN1 */
	except  CAN2_Handler                /* CAN2 */
	.long   0                           /* Reserved */
	except  HIB_Handler                 /* Hibernate */
	except  USB0_Handler                /* USB0 */
	except  PWM0_3_Handler              /* PWM Generator 3 */
	except  UDMA_Handler                /* uDMA Software Transfer */
	except  UDMAERR_Handler             /* uDMA Error */
	except  ADC1SS0_Handler             /* ADC1 Sequence 0 */
	except  ADC1SS1_Handler             /* ADC1 Sequence 1 */
	except  ADC1SS2_Handler             /* ADC1 Sequence 2 */
	except  ADC1SS3_Handler             /* ADC1 Sequence 3 */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	except  GPIOJ_Handler               /* GPIO Port J */
	except  GPIOK_Handler               /* GPIO Port K */
	except  GPIOL_Handler               /* GPIO Port L */
	except  SSI2_Handler                /* SSI2 Rx and Tx */
	except  SSI3_Handler                /* SSI3 Rx and Tx */
	except  UART3_Handler               /* UART3 Rx and Tx */
	except  UART4_Handler               /* UART4 Rx and Tx */
	except  UART5_Handler               /* UART5 Rx and Tx */
	except  UART6_Handler               /* UART6 Rx and Tx */
	except  UART7_Handler               /* UART7 Rx and Tx */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	except  I2C2_Handler                /* I2C2 Master and Slave */
	except  I2C3_Handler                /* I2C3 Master and Slave */
	except  TIMER4A_Handler             /* Timer 4 subtimer A */
	except  TIMER4B_Handler             /* Timer 4 subtimer B */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	except  TIMER5A_Handler             /* Timer 5 subtimer A */
	except  TIMER5B_Handler             /* Timer 5 subtimer B */
	except  WTIMER0A_Handler            /* Wide Timer 0 subtimer A */
	except  WTIMER0B_Handler            /* Wide Timer 0 subtimer B */
	except  WTIMER1A_Handler            /* Wide Timer 1 subtimer A */
	except  WTIMER1B_Handler            /* Wide Timer 1 subtimer B */
	except  WTIMER2A_Handler            /* Wide Timer 2 subtimer A */
	except  WTIMER2B_Handler            /* Wide Timer 2 subtimer B */
	except  WTIMER3A_Handler            /* Wide Timer 3 subtimer A */
	except  WTIMER3B_Handler            /* Wide Timer 3 subtimer B */
	except  WTIMER4A_Handler            /* Wide Timer 4 subtimer A */
	except  WTIMER4B_Handler            /* Wide Timer 4 subtimer B */
	except  WTIMER5A_Handler            /* Wide Timer 5 subtimer A */
	except  WTIMER5B_Handler            /* Wide Timer 5 subtimer B */
	except  FPU_Handler                 /* FPU */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	except  I2C4_Handler                /* I2C4 Master and Slave */
	except  I2C5_Handler                /* I2C5 Master and Slave */
	except  GPIOM_Handler               /* GPIO Port M */
	except  GPION_Handler               /* GPIO Port N */
	except  QEI2_Handler                /* Quadrature Encoder 2 */
	.long   0                           /* Reserved */
	.long   0                           /* Reserved */
	except  GPIOP0_Handler              /* GPIO Port P (Summary or P0) */
	except  GPIOP1_Handler              /* GPIO Port P1 */
	except  GPIOP2_Handler              /* GPIO Port P2 */
	except  GPIOP3_Handler              /* GPIO Port P3 */
	except  GPIOP4_Handler              /* GPIO Port P4 */
	except  GPIOP5_Handler              /* GPIO Port P5 */
	except  GPIOP6_Handler              /* GPIO Port P6 */
	except  GPIOP7_Handler              /* GPIO Port P7 */
	except  GPIOQ0_Handler              /* GPIO Port Q (Summary or Q0) */
	except  GPIOQ1_Handler              /* GPIO Port Q1 */
	except  GPIOQ2_Handler              /* GPIO Port Q2 */
	except  GPIOQ3_Handler              /* GPIO Port Q3 */
	except  GPIOQ4_Handler              /* GPIO Port Q4 */
	except  GPIOQ5_Handler              /* GPIO Port Q5 */
	except  GPIOQ6_Handler              /* GPIO Port Q6 */
	except  GPIOQ7_Handler              /* GPIO Port Q7 */
	except  GPIOR_Handler               /* GPIO Port R */
	except  GPIOS_Handler               /* GPIO Port S */
	except  PMW1_0_Handler              /* PWM 1 Generator 0 */
	except  PWM1_1_Handler              /* PWM 1 Generator 1 */
	except  PWM1_2_Handler              /* PWM 1 Generator 2 */
	except  PWM1_3_Handler              /* PWM 1 Generator 3 */
	except  PWM1_FAULT_Handler          /* PWM 1 Fault */


        /* include the bootloader information block */
        .section .mainbootinfo

        .section .text

        .global _start
        .thumb_func
_start:
        /* copy data section to ram */
        ldr     r0, =__data_load_start
        ldr     r1, =__data_load_end
        ldr     r2, =__data_start
dataloop:
        ldr.w   r3, [r0], #4
        str.w   r3, [r2], #4
        cmp     r0, r1
        blo     dataloop

        /* clear bss section */
        ldr     r0, =__bss_start__
        ldr     r1, =__bss_end__
        ldr     r2, =0
bssloop:
        str.w   r2, [r0], #4
        cmp     r0, r1
        blo     bssloop

        /* start main() */
        b       main


        /* dummy exception handler: endless loop */
        .weak __unhandled_exception
        .thumb_func
__unhandled_exception:
        /* turn on LED 4 on an mbed module */
        //ldr     r0, =0x2009c038
        //ldr     r1, =0x800000
        //str     r1, [r0]
        b       __unhandled_exception

        .end
