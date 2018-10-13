/**
 * Reset and Clock Control
 *
 * Defined Constants and Types for the TM4C Reset and Clock Control</b>
 **/

#ifndef TM4C_RCC_H
#define TM4C_RCC_H

#include <stdint.h>
#include <stdio.h>
#include "inc/tm4c123gh6pm.h"
#include "cm3.h"

/* =============================================================================
 * SYSCTL_RCC values
 * ---------------------------------------------------------------------------*/

/** System Clock Divisor */
#define SYSCTL_RCC_SYSDIV_MASK      (0xF << 23)
/** PWM Unit Clock Divisor */
#define SYSCTL_RCC_PWMDIV_MASK      (0xF << 17)
/** Crystal Value */
#define SYSCTL_RCC_XTAL_MASK        (0x1F << 6)
#define SYSCTL_RCC_XTAL_4M          (0x06 << 6)
#define SYSCTL_RCC_XTAL_4M_096      (0x07 << 6)
#define SYSCTL_RCC_XTAL_4M_9152     (0x08 << 6)
#define SYSCTL_RCC_XTAL_5M          (0x09 << 6)
#define SYSCTL_RCC_XTAL_5M_12       (0x0A << 6)
#define SYSCTL_RCC_XTAL_6M          (0x0B << 6)
#define SYSCTL_RCC_XTAL_6M_144      (0x0C << 6)
#define SYSCTL_RCC_XTAL_7M_3728     (0x0D << 6)
#define SYSCTL_RCC_XTAL_8M          (0x0E << 6)
#define SYSCTL_RCC_XTAL_8M_192      (0x0F << 6)
#define SYSCTL_RCC_XTAL_10M         (0x10 << 6)
#define SYSCTL_RCC_XTAL_12M         (0x11 << 6)
#define SYSCTL_RCC_XTAL_12M_288     (0x12 << 6)
#define SYSCTL_RCC_XTAL_13M_56      (0x13 << 6)
#define SYSCTL_RCC_XTAL_14M_31818   (0x14 << 6)
#define SYSCTL_RCC_XTAL_16M         (0x15 << 6)
#define SYSCTL_RCC_XTAL_16M_384     (0x16 << 6)
#define SYSCTL_RCC_XTAL_18M         (0x17 << 6)
#define SYSCTL_RCC_XTAL_20M         (0x18 << 6)
#define SYSCTL_RCC_XTAL_24M         (0x19 << 6)
#define SYSCTL_RCC_XTAL_25M         (0x1A << 6)
/** Oscillator Source */
#define SYSCTL_RCC_OSCSRC_MASK      (0x3 << 4)
#define SYSCTL_RCC_OSCSRC_MOSC      (0x0 << 4)
#define SYSCTL_RCC_OSCSRC_PIOSC     (0x1 << 4)
#define SYSCTL_RCC_OSCSRC_PIOSC_D4  (0x2 << 4)
#define SYSCTL_RCC_OSCSRC_30K       (0x3 << 4)
/** Precision Internal Oscillator Disable */
#define SYSCTL_RCC_IOSCDIS      (1 << 1)

/* Peripheral run mode clock gating control */
#define SYSCTL_RCGCWD           MMIO32(SYSCTL_BASE + 0x600)
#define SYSCTL_RCGCTIMER        MMIO32(SYSCTL_BASE + 0x604)
#define SYSCTL_RCGCGPIO         MMIO32(SYSCTL_BASE + 0x608)
#define SYSCTL_RCGCDMA          MMIO32(SYSCTL_BASE + 0x60C)
#define SYSCTL_RCGCHIB          MMIO32(SYSCTL_BASE + 0x614)
#define SYSCTL_RCGCUART         MMIO32(SYSCTL_BASE + 0x618)
#define SYSCTL_RCGCSSI          MMIO32(SYSCTL_BASE + 0x61C)
#define SYSCTL_RCGCI2C          MMIO32(SYSCTL_BASE + 0x620)
#define SYSCTL_RCGCUSB          MMIO32(SYSCTL_BASE + 0x628)
#define SYSCTL_RCGCCAN          MMIO32(SYSCTL_BASE + 0x634)
#define SYSCTL_RCGCADC          MMIO32(SYSCTL_BASE + 0x638)
#define SYSCTL_RCGCACMP         MMIO32(SYSCTL_BASE + 0x63C)
#define SYSCTL_RCGCPWM          MMIO32(SYSCTL_BASE + 0x640)
#define SYSCTL_RCGCQEI          MMIO32(SYSCTL_BASE + 0x644)
#define SYSCTL_RCGCEEPROM       MMIO32(SYSCTL_BASE + 0x658)
#define SYSCTL_RCGCWTIMER       MMIO32(SYSCTL_BASE + 0x65C)
/* Peripheral sleep mode clock gating control */
#define SYSCTL_SCGCWD           MMIO32(SYSCTL_BASE + 0x700)
#define SYSCTL_SCGCTIMER        MMIO32(SYSCTL_BASE + 0x704)
#define SYSCTL_SCGCGPIO         MMIO32(SYSCTL_BASE + 0x708)
#define SYSCTL_SCGCDMA          MMIO32(SYSCTL_BASE + 0x70C)
#define SYSCTL_SCGCHIB          MMIO32(SYSCTL_BASE + 0x714)
#define SYSCTL_SCGCUART         MMIO32(SYSCTL_BASE + 0x718)
#define SYSCTL_SCGCSSI          MMIO32(SYSCTL_BASE + 0x71C)
#define SYSCTL_SCGCI2C          MMIO32(SYSCTL_BASE + 0x720)
#define SYSCTL_SCGCUSB          MMIO32(SYSCTL_BASE + 0x728)
#define SYSCTL_SCGCCAN          MMIO32(SYSCTL_BASE + 0x734)
#define SYSCTL_SCGCADC          MMIO32(SYSCTL_BASE + 0x738)
#define SYSCTL_SCGCACMP         MMIO32(SYSCTL_BASE + 0x73C)
#define SYSCTL_SCGCPWM          MMIO32(SYSCTL_BASE + 0x740)
#define SYSCTL_SCGCQEI          MMIO32(SYSCTL_BASE + 0x744)
#define SYSCTL_SCGCEEPROM       MMIO32(SYSCTL_BASE + 0x758)
#define SYSCTL_SCGCWTIMER       MMIO32(SYSCTL_BASE + 0x75C)
/* Peripheral deep-sleep mode clock gating control */
#define SYSCTL_DCGCWD           MMIO32(SYSCTL_BASE + 0x800)
#define SYSCTL_DCGCTIMER        MMIO32(SYSCTL_BASE + 0x804)
#define SYSCTL_DCGCGPIO         MMIO32(SYSCTL_BASE + 0x808)
#define SYSCTL_DCGCDMA          MMIO32(SYSCTL_BASE + 0x80C)
#define SYSCTL_DCGCHIB          MMIO32(SYSCTL_BASE + 0x814)
#define SYSCTL_DCGCUART         MMIO32(SYSCTL_BASE + 0x818)
#define SYSCTL_DCGCSSI          MMIO32(SYSCTL_BASE + 0x81C)
#define SYSCTL_DCGCI2C          MMIO32(SYSCTL_BASE + 0x820)
#define SYSCTL_DCGCUSB          MMIO32(SYSCTL_BASE + 0x828)
#define SYSCTL_DCGCCAN          MMIO32(SYSCTL_BASE + 0x834)
#define SYSCTL_DCGCADC          MMIO32(SYSCTL_BASE + 0x838)
#define SYSCTL_DCGCACMP         MMIO32(SYSCTL_BASE + 0x83C)
#define SYSCTL_DCGCPWM          MMIO32(SYSCTL_BASE + 0x840)
#define SYSCTL_DCGCQEI          MMIO32(SYSCTL_BASE + 0x844)
#define SYSCTL_DCGCEEPROM       MMIO32(SYSCTL_BASE + 0x858)
#define SYSCTL_DCGCWTIMER       MMIO32(SYSCTL_BASE + 0x85C)
/* Peripheral ready */
#define SYSCTL_PRWD         MMIO32(SYSCTL_BASE + 0xA00)
#define SYSCTL_PRTIMER          MMIO32(SYSCTL_BASE + 0xA04)
#define SYSCTL_PRGPIO           MMIO32(SYSCTL_BASE + 0xA08)
#define SYSCTL_PRDMA            MMIO32(SYSCTL_BASE + 0xA0C)
#define SYSCTL_PRHIB            MMIO32(SYSCTL_BASE + 0xA14)
#define SYSCTL_PRUART           MMIO32(SYSCTL_BASE + 0xA18)
#define SYSCTL_PRSSI            MMIO32(SYSCTL_BASE + 0xA1C)
#define SYSCTL_PRI2C            MMIO32(SYSCTL_BASE + 0xA20)
#define SYSCTL_PRUSB            MMIO32(SYSCTL_BASE + 0xA28)
#define SYSCTL_PRCAN            MMIO32(SYSCTL_BASE + 0xA34)
#define SYSCTL_PRADC            MMIO32(SYSCTL_BASE + 0xA38)
#define SYSCTL_PRACMP           MMIO32(SYSCTL_BASE + 0xA3C)
#define SYSCTL_PRPWM            MMIO32(SYSCTL_BASE + 0xA40)
#define SYSCTL_PRQEI            MMIO32(SYSCTL_BASE + 0xA44)
#define SYSCTL_PREEPROM         MMIO32(SYSCTL_BASE + 0xA58)
#define SYSCTL_PRWTIMER         MMIO32(SYSCTL_BASE + 0xA5C)

/* =============================================================================
 * SYSCTL_RCC2 values
 * ---------------------------------------------------------------------------*/
/** Auto Clock Gating */
#define SYSCTL_RCC2_ACG             (1 << 27)
/** System Clock Divisor 2 */
#define SYSCTL_RCC2_SYSDIV2_MASK    (0x3F << 23)
/** System clock divisor mask when RCC2_DIV400 is set */
#define SYSCTL_RCC2_SYSDIV400_MASK  (0x7F << 22)

/** Oscillator Source 2 */
#define SYSCTL_RCC2_OSCSRC2_MASK    (0x7 << 4)
#define SYSCTL_RCC2_OSCSRC2_MOSC    (0x0 << 4)
#define SYSCTL_RCC2_OSCSRC2_PIOSC   (0x1 << 4)
#define SYSCTL_RCC2_OSCSRC2_PIOSC_D4    (0x2 << 4)
#define SYSCTL_RCC2_OSCSRC2_30K     (0x3 << 4)
#define SYSCTL_RCC2_OSCSRC2_32K768  (0x7 << 4)

/**
 * Clock enable definitions
 *
 * The definitions are specified in the form
 * 31:5 register offset from SYSCTL_BASE for the clock register
 * 4:0  bit offset for the given peripheral
 *
 * The names have the form [clock_type]_[periph_type]_[periph_number]
 * Where clock_type is
 *     RCC for run clock
 *     SCC for sleep clock
 *     DCC for deep-sleep clock
 **/
enum tm4c_clken {
    /*
     * Run clock control
     */
    RCC_WD0 = ((uint32_t)&SYSCTL_RCGCWD - SYSCTL_BASE) << 5,
    RCC_WD1,

    RCC_TIMER0 = ((uint32_t)&SYSCTL_RCGCTIMER - SYSCTL_BASE) << 5,
    RCC_TIMER1,
    RCC_TIMER2,
    RCC_TIMER3,
    RCC_TIMER4,
    RCC_TIMER5,

    RCC_GPIOA = ((uint32_t)&SYSCTL_RCGCGPIO - SYSCTL_BASE) << 5,
    RCC_GPIOB,
    RCC_GPIOC,
    RCC_GPIOD,
    RCC_GPIOE,
    RCC_GPIOF,
    RCC_GPIOG,
    RCC_GPIOH,
    RCC_GPIOJ,
    RCC_GPIOK,
    RCC_GPIOL,
    RCC_GPIOM,
    RCC_GPION,
    RCC_GPIOP,
    RCC_GPIOQ,

    RCC_DMA = ((uint32_t)&SYSCTL_RCGCDMA - SYSCTL_BASE) << 5,

    RCC_HIB = ((uint32_t)&SYSCTL_RCGCGPIO - SYSCTL_BASE) << 5,

    RCC_UART0 = ((uint32_t)&SYSCTL_RCGCUART - SYSCTL_BASE) << 5,
    RCC_UART1,
    RCC_UART2,
    RCC_UART3,
    RCC_UART4,
    RCC_UART5,
    RCC_UART6,
    RCC_UART7,

    RCC_SSI0 = ((uint32_t)&SYSCTL_RCGCSSI - SYSCTL_BASE) << 5,
    RCC_SSI1,
    RCC_SSI2,
    RCC_SSI3,

    RCC_I2C0 = ((uint32_t)&SYSCTL_RCGCI2C - SYSCTL_BASE) << 5,
    RCC_I2C1,
    RCC_I2C2,
    RCC_I2C3,
    RCC_I2C4,
    RCC_I2C5,

    RCC_USB0 = ((uint32_t)&SYSCTL_RCGCUSB - SYSCTL_BASE) << 5,

    RCC_CAN0 = ((uint32_t)&SYSCTL_RCGCCAN - SYSCTL_BASE) << 5,
    RCC_CAN1,

    RCC_ADC0 = ((uint32_t)&SYSCTL_RCGCADC - SYSCTL_BASE) << 5,
    RCC_ADC1,

    RCC_ACMP0 = ((uint32_t)&SYSCTL_RCGCACMP - SYSCTL_BASE) << 5,

    RCC_PWM0 = ((uint32_t)&SYSCTL_RCGCPWM - SYSCTL_BASE) << 5,
    RCC_PWM1,

    RCC_QEI0 = ((uint32_t)&SYSCTL_RCGCQEI - SYSCTL_BASE) << 5,
    RCC_QEI1,

    RCC_EEPROM0 = ((uint32_t)&SYSCTL_RCGCEEPROM - SYSCTL_BASE) << 5,

    RCC_WTIMER0 = ((uint32_t)&SYSCTL_RCGCWTIMER - SYSCTL_BASE) << 5,
    RCC_WTIMER1,
    RCC_WTIMER2,
    RCC_WTIMER3,
    RCC_WTIMER4,
    RCC_WTIMER5,


    /*
     * Sleep clock control
     */
    SCC_WD0 = ((uint32_t)&SYSCTL_SCGCWD - SYSCTL_BASE) << 5,
    SCC_WD1,

    SCC_TIMER0 = ((uint32_t)&SYSCTL_SCGCTIMER - SYSCTL_BASE) << 5,
    SCC_TIMER1,
    SCC_TIMER2,
    SCC_TIMER3,
    SCC_TIMER4,
    SCC_TIMER5,

    SCC_GPIOA = ((uint32_t)&SYSCTL_SCGCGPIO - SYSCTL_BASE) << 5,
    SCC_GPIOB,
    SCC_GPIOC,
    SCC_GPIOD,
    SCC_GPIOE,
    SCC_GPIOF,
    SCC_GPIOG,
    SCC_GPIOH,
    SCC_GPIOJ,
    SCC_GPIOK,
    SCC_GPIOL,
    SCC_GPIOM,
    SCC_GPION,
    SCC_GPIOP,
    SCC_GPIOQ,

    SCC_DMA = ((uint32_t)&SYSCTL_SCGCDMA - SYSCTL_BASE) << 5,

    SCC_HIB = ((uint32_t)&SYSCTL_SCGCGPIO - SYSCTL_BASE) << 5,

    SCC_UART0 = ((uint32_t)&SYSCTL_SCGCUART - SYSCTL_BASE) << 5,
    SCC_UART1,
    SCC_UART2,
    SCC_UART3,
    SCC_UART4,
    SCC_UART5,
    SCC_UART6,
    SCC_UART7,

    SCC_SSI0 = ((uint32_t)&SYSCTL_SCGCSSI - SYSCTL_BASE) << 5,
    SCC_SSI1,
    SCC_SSI2,
    SCC_SSI3,

    SCC_I2C0 = ((uint32_t)&SYSCTL_SCGCI2C - SYSCTL_BASE) << 5,
    SCC_I2C1,
    SCC_I2C2,
    SCC_I2C3,
    SCC_I2C4,
    SCC_I2C5,

    SCC_USB0 = ((uint32_t)&SYSCTL_SCGCUSB - SYSCTL_BASE) << 5,

    SCC_CAN0 = ((uint32_t)&SYSCTL_SCGCCAN - SYSCTL_BASE) << 5,
    SCC_CAN1,

    SCC_ADC0 = ((uint32_t)&SYSCTL_SCGCADC - SYSCTL_BASE) << 5,
    SCC_ADC1,

    SCC_ACMP0 = ((uint32_t)&SYSCTL_SCGCACMP - SYSCTL_BASE) << 5,

    SCC_PWM0 = ((uint32_t)&SYSCTL_SCGCPWM - SYSCTL_BASE) << 5,
    SCC_PWM1,

    SCC_QEI0 = ((uint32_t)&SYSCTL_SCGCQEI - SYSCTL_BASE) << 5,
    SCC_QEI1,

    SCC_EEPROM0 = ((uint32_t)&SYSCTL_SCGCEEPROM - SYSCTL_BASE) << 5,

    SCC_WTIMER0 = ((uint32_t)&SYSCTL_SCGCWTIMER - SYSCTL_BASE) << 5,
    SCC_WTIMER1,
    SCC_WTIMER2,
    SCC_WTIMER3,
    SCC_WTIMER4,
    SCC_WTIMER5,

    /*
     * Deep-sleep clock control
     */
    DCC_WD0 = ((uint32_t)&SYSCTL_DCGCWD - SYSCTL_BASE) << 5,
    DCC_WD1,

    DCC_TIMER0 = ((uint32_t)&SYSCTL_DCGCTIMER - SYSCTL_BASE) << 5,
    DCC_TIMER1,
    DCC_TIMER2,
    DCC_TIMER3,
    DCC_TIMER4,
    DCC_TIMER5,

    DCC_GPIOA = ((uint32_t)&SYSCTL_DCGCGPIO - SYSCTL_BASE) << 5,
    DCC_GPIOB,
    DCC_GPIOC,
    DCC_GPIOD,
    DCC_GPIOE,
    DCC_GPIOF,
    DCC_GPIOG,
    DCC_GPIOH,
    DCC_GPIOJ,
    DCC_GPIOK,
    DCC_GPIOL,
    DCC_GPIOM,
    DCC_GPION,
    DCC_GPIOP,
    DCC_GPIOQ,

    DCC_DMA = ((uint32_t)&SYSCTL_DCGCDMA - SYSCTL_BASE) << 5,

    DCC_HIB = ((uint32_t)&SYSCTL_DCGCGPIO - SYSCTL_BASE) << 5,

    DCC_UART0 = ((uint32_t)&SYSCTL_DCGCUART - SYSCTL_BASE) << 5,
    DCC_UART1,
    DCC_UART2,
    DCC_UART3,
    DCC_UART4,
    DCC_UART5,
    DCC_UART6,
    DCC_UART7,

    DCC_SSI0 = ((uint32_t)&SYSCTL_DCGCSSI - SYSCTL_BASE) << 5,
    DCC_SSI1,
    DCC_SSI2,
    DCC_SSI3,

    DCC_I2C0 = ((uint32_t)&SYSCTL_DCGCI2C - SYSCTL_BASE) << 5,
    DCC_I2C1,
    DCC_I2C2,
    DCC_I2C3,
    DCC_I2C4,
    DCC_I2C5,

    DCC_USB0 = ((uint32_t)&SYSCTL_DCGCUSB - SYSCTL_BASE) << 5,

    DCC_CAN0 = ((uint32_t)&SYSCTL_DCGCCAN - SYSCTL_BASE) << 5,
    DCC_CAN1,

    DCC_ADC0 = ((uint32_t)&SYSCTL_DCGCADC - SYSCTL_BASE) << 5,
    DCC_ADC1,

    DCC_ACMP0 = ((uint32_t)&SYSCTL_DCGCACMP - SYSCTL_BASE) << 5,

    DCC_PWM0 = ((uint32_t)&SYSCTL_DCGCPWM - SYSCTL_BASE) << 5,
    DCC_PWM1,

    DCC_QEI0 = ((uint32_t)&SYSCTL_DCGCQEI - SYSCTL_BASE) << 5,
    DCC_QEI1,

    DCC_EEPROM0 = ((uint32_t)&SYSCTL_DCGCEEPROM - SYSCTL_BASE) << 5,

    DCC_WTIMER0 = ((uint32_t)&SYSCTL_DCGCWTIMER - SYSCTL_BASE) << 5,
    DCC_WTIMER1,
    DCC_WTIMER2,
    DCC_WTIMER3,
    DCC_WTIMER4,
    DCC_WTIMER5,

};

/**
 * Oscillator source values
 *
 * Possible values of the oscillator source.
 */
enum osc_src {
	OSCSRC_MOSC		= SYSCTL_RCC2_OSCSRC2_MOSC,
	OSCSRC_PIOSC	= SYSCTL_RCC2_OSCSRC2_PIOSC,
	OSCSRC_PIOSC_D4	= SYSCTL_RCC2_OSCSRC2_PIOSC_D4,
	OSCSRC_30K_INT	= SYSCTL_RCC2_OSCSRC2_30K,
	OSCSRC_32K_EXT	= SYSCTL_RCC2_OSCSRC2_32K768,
};

/**
 * PWM clock divisor values
 *
 * Possible values of the binary divisor used to pre-divide the
 *  system clock down for use as the timing reference for the PWM module.
 */
enum pwm_clkdiv {
	PWMDIV_2		= SYSCTL_RCC_PWMDIV_2,
	PWMDIV_4		= SYSCTL_RCC_PWMDIV_4,
	PWMDIV_8		= SYSCTL_RCC_PWMDIV_8,
	PWMDIV_16		= SYSCTL_RCC_PWMDIV_16,
	PWMDIV_32		= SYSCTL_RCC_PWMDIV_32,
	PWMDIV_64		= SYSCTL_RCC_PWMDIV_64,
};

/**
 * Predefined crystal values
 *
 * Predefined crystal values for the XTAL field in SYSCTL_RCC.
 * Using these predefined values in the XTAL field, the SYSCTL_PLLFREQ0 and
 * SYSCTL_PLLFREQ1 are automatically adjusted in hardware to provide a PLL clock
 * of 400MHz.
 **/
enum xtal_t {
	XTAL_4M			= SYSCTL_RCC_XTAL_4M,
	XTAL_4M_096		= SYSCTL_RCC_XTAL_4M_096,
	XTAL_4M_9152	= SYSCTL_RCC_XTAL_4M_9152,
	XTAL_5M			= SYSCTL_RCC_XTAL_5M,
	XTAL_5M_12		= SYSCTL_RCC_XTAL_5M_12,
	XTAL_6M			= SYSCTL_RCC_XTAL_6M,
	XTAL_6M_144		= SYSCTL_RCC_XTAL_6M_144,
	XTAL_7M_3728	= SYSCTL_RCC_XTAL_7M_3728,
	XTAL_8M			= SYSCTL_RCC_XTAL_8M,
	XTAL_8M_192		= SYSCTL_RCC_XTAL_8M_192,
	XTAL_10M		= SYSCTL_RCC_XTAL_10M,
	XTAL_12M		= SYSCTL_RCC_XTAL_12M,
	XTAL_12M_288	= SYSCTL_RCC_XTAL_12M_288,
	XTAL_13M_56		= SYSCTL_RCC_XTAL_13M_56,
	XTAL_14M_31818	= SYSCTL_RCC_XTAL_14M_31818,
	XTAL_16M		= SYSCTL_RCC_XTAL_16M,
	XTAL_16M_384	= SYSCTL_RCC_XTAL_16M_384,
	XTAL_18M		= SYSCTL_RCC_XTAL_18M,
	XTAL_20M		= SYSCTL_RCC_XTAL_20M,
	XTAL_24M		= SYSCTL_RCC_XTAL_24M,
	XTAL_25M		= SYSCTL_RCC_XTAL_25M,
};

extern uint32_t tm4c_rcc_sysclk_freq;

/* =============================================================================
 * Function prototypes
 * ---------------------------------------------------------------------------*/

/* Low-level clock API */
void rcc_configure_xtal(enum xtal_t xtal);
void rcc_disable_main_osc(void);
void rcc_disable_interal_osc(void);
void rcc_enable_main_osc(void);
void rcc_enable_interal_osc(void);
void rcc_enable_rcc2(void);
void rcc_pll_off(void);
void rcc_pll_on(void);
void rcc_set_osc_source(enum osc_src src);
void rcc_pll_bypass_disable(void);
void rcc_pll_bypass_enable(void);
void rcc_set_pll_divisor(uint8_t div400);
void rcc_set_pwm_divisor(enum pwm_clkdiv div);
void rcc_usb_pll_off(void);
void rcc_usb_pll_on(void);
void rcc_wait_for_pll_ready(void);
/* High-level clock API */
void rcc_change_pll_divisor(uint8_t plldiv400);
uint32_t rcc_get_system_clock_frequency(void);
void rcc_sysclk_config(enum osc_src src, enum xtal_t xtal, uint8_t pll_div400);

void periph_clock_enable(enum tm4c_clken periph);
void periph_clock_disable(enum tm4c_clken periph);

#endif /* TM4C_RCC_H */
