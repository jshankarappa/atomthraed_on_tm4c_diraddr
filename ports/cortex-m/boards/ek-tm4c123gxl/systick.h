/** SysTick Defines
 *
 * Defined Constants and Types for the Cortex SysTick
 *
 **/

/**
 * @note this file has been not following the register naming scheme, the
 * correct names defined, and the old ones stay there for compatibility with
 * old software (will be deprecated in the future)
 */

#ifndef TM4C_SYSTICK_H
#define TM4C_SYSTICK_H

#include <stdint.h>
#include <stdio.h>
#include "inc/tm4c123gh6pm.h"
#include "cm3.h"

/* --- SYSTICK registers --------------------------------------------------- */

/* calibration value register (STK_CALIB) */
#define STK_CALIB			MMIO32(SYS_TICK_BASE + 0x0C)

/* --- STK_CSR values ------------------------------------------------------ */
/* Bits [31:17] Reserved, must be kept cleared. */
/* COUNTFLAG: */
#define STK_CSR_COUNTFLAG		(1 << 16)

/* Bits [15:3] Reserved, must be kept cleared. */
/* CLKSOURCE: Clock source selection */
#define STK_CSR_CLKSOURCE_LSB		2
#define STK_CSR_CLKSOURCE		(1 << STK_CSR_CLKSOURCE_LSB)

/**
 * Clock source selection
 *  CM3_systick_defines
**/
#if defined(__ARM_ARCH_6M__)
#define STK_CSR_CLKSOURCE_EXT		(0 << STK_CSR_CLKSOURCE_LSB)
#define STK_CSR_CLKSOURCE_AHB		(1 << STK_CSR_CLKSOURCE_LSB)
#else
#define STK_CSR_CLKSOURCE_AHB_DIV8	(0 << STK_CSR_CLKSOURCE_LSB)
#define STK_CSR_CLKSOURCE_AHB		(1 << STK_CSR_CLKSOURCE_LSB)
#endif

/* TICKINT: SysTick exception request enable */
#define STK_CSR_TICKINT			(1 << 1)
/* ENABLE: Counter enable */
#define STK_CSR_ENABLE			(1 << 0)

/* --- STK_RVR values ------------------------------------------------------ */
/* Bits [31:24] Reserved, must be kept cleared. */
/* RELOAD[23:0]: RELOAD value */
#define STK_RVR_RELOAD			0x00FFFFFF

/* --- STK_CVR values ------------------------------------------------------ */
/* Bits [31:24] Reserved, must be kept cleared. */
/* CURRENT[23:0]: Current counter value */
#define STK_CVR_CURRENT			0x00FFFFFF

/* --- STK_CALIB values ---------------------------------------------------- */
/* NOREF: NOREF flag */
#define STK_CALIB_NOREF			(1 << 31)
/* SKEW: SKEW flag */
#define STK_CALIB_SKEW			(1 << 30)
/* Bits [29:24] Reserved, must be kept cleared. */
/* TENMS[23:0]: Calibration value */
#define STK_CALIB_TENMS			0x00FFFFFF

/* --- Function Prototypes ------------------------------------------------- */

void systick_set_reload(uint32_t value);
bool systick_set_frequency(uint32_t freq, uint32_t ahb);
uint32_t systick_get_reload(void);
uint32_t systick_get_value(void);
void systick_set_clocksource(uint8_t clocksource);
void systick_interrupt_enable(void);
void systick_interrupt_disable(void);
void systick_counter_enable(void);
void systick_counter_disable(void);
uint8_t systick_get_countflag(void);
void systick_clear(void);

uint32_t systick_get_calib(void);

#endif      // TM4C_SYSTICK_H
