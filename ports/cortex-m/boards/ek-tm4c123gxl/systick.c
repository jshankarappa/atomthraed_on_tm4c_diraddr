/**
 * SysTick
 *
 * Cortex System Tick Timer
 *
 **/

#include "systick.h"

/*---------------------------------------------------------------------------*/
/** SysTick Set the Automatic Reload Value.
 *
 * The counter is set to the reload value when the counter starts and after it
 * reaches zero.
 *
 * note: The systick counter value might be undefined upon startup. To get
 * predictable behavior, it is a good idea to set or clear the counter after
 * set reload. see also - systick_clear
 *
 * param[in] value uint32_t. 24 bit reload value.
 **/

void systick_set_reload(uint32_t value)
{
    NVIC_ST_RELOAD_R = (value & STK_RVR_RELOAD);
}

/*---------------------------------------------------------------------------*/
/** SysTick Read the Automatic Reload Value.
 *
 * returns : 24 bit reload value as uint32_t.
 */

uint32_t systick_get_reload(void)
{
	return NVIC_ST_RELOAD_R & STK_RVR_RELOAD;
}

/** SysTick Set clock and frequency of overflow
 *
 * This function sets the systick to AHB clock source, and the prescaler to
 * generate interrupts with the desired frequency. The function fails, if
 * the frequency is too low.
 *
 * param[in] freq uint32_t The desired frequency in Hz
 * param[in] ahb uint32_t The current AHB frequency in Hz
 * returns true, if success, false if the desired frequency cannot be set.
 */
bool systick_set_frequency(uint32_t freq, uint32_t ahb)
{
	uint32_t ratio = ahb / freq;

#if defined(__ARM_ARCH_6M__)
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
#else
	if (ratio >= (STK_RVR_RELOAD * 8)) {
		/* This frequency is too slow */
		return false;
	} else if (ratio >= STK_RVR_RELOAD) {
		ratio /= 8;
		systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	} else {
		systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	}
#endif
	systick_set_reload(ratio - 1);
	return true;
}

/*---------------------------------------------------------------------------*/
/** Get the current SysTick counter value.
 *
 * returns 24 bit current value as uint32_t.
 */

uint32_t systick_get_value(void)
{
	return NVIC_ST_CURRENT_R & STK_CVR_CURRENT;
}

/*---------------------------------------------------------------------------*/
/** Set the SysTick Clock Source.
 *
 * The clock source can be either the AHB clock or the same clock divided by 8.
 *
 * param[in] clocksource uint8_t. Clock source from systick_clksource.
 */

void systick_set_clocksource(uint8_t clocksource)
{
    NVIC_ST_CTRL_R = (NVIC_ST_CTRL_R & ~STK_CSR_CLKSOURCE) |
		  (clocksource & STK_CSR_CLKSOURCE);
}

/*---------------------------------------------------------------------------*/
/** Enable SysTick Interrupt.
 *
 */

void systick_interrupt_enable(void)
{
    NVIC_ST_CTRL_R |= STK_CSR_TICKINT;
}

/*---------------------------------------------------------------------------*/
/** Disable SysTick Interrupt.
 *
 */

void systick_interrupt_disable(void)
{
    NVIC_ST_CTRL_R &= ~STK_CSR_TICKINT;
}

/*---------------------------------------------------------------------------*/
/** Enable SysTick Counter.
 *
 */

void systick_counter_enable(void)
{
    NVIC_ST_CTRL_R |= STK_CSR_ENABLE;
}

/*---------------------------------------------------------------------------*/
/** Disable SysTick Counter.
 *
 */

void systick_counter_disable(void)
{
    NVIC_ST_CTRL_R &= ~STK_CSR_ENABLE;
}

/*---------------------------------------------------------------------------*/
/** SysTick Read the Counter Flag.
 *
 * The count flag is set when the timer count becomes zero, and is cleared when
 * the flag is read.
 *
 * returns Boolean if flag set.
 */

uint8_t systick_get_countflag(void)
{
	return (NVIC_ST_CTRL_R & STK_CSR_COUNTFLAG) ? 1 : 0;
}

/*---------------------------------------------------------------------------*/
/** SysTick Clear counter Value.
 *
 * The counter value is cleared. Useful for well defined startup.
 */

void systick_clear(void)
{
    NVIC_ST_CURRENT_R = 0;
}

/*---------------------------------------------------------------------------*/
/** SysTick Get Calibration Value
 *
 * returns Current calibration value
 */
uint32_t systick_get_calib(void)
{
	return STK_CALIB & STK_CALIB_TENMS;
}
