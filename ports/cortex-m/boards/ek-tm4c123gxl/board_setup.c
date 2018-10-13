/*
 * board_setup.c
 *
 *  Created on: 25-Sep-2018
 *      Author: jshankar
 */

#include "uart.h"
#include "rcc.h"
#include "gpio.h"
#include "systick.h"
#include "atomport.h"

/**
 * Initialize and start SysTick counter. This will trigger the
 * sys_tick_handler() periodically once interrupts have been enabled
 * by archFirstThreadRestore() function
 */
static void systick_setup(void)
{
    systick_set_frequency(SYSTEM_TICKS_PER_SEC, 80000000);

    systick_interrupt_enable();

    systick_counter_enable();
}

/**
 * Set up the core clock to something other than the internal 16MHz PIOSC.
 * Make sure that you use the same clock frequency in  systick_setup().
 */
#define PLLDIV_80MHZ 5
static void clock_setup(void)
{
    /**
     * set up 400MHz PLL from 16MHz crystal and divide by 5 to get 80MHz
     * system clock
     */
    rcc_sysclk_config(OSCSRC_MOSC, XTAL_16M, PLLDIV_80MHZ);
}

/**
 * Set up user LED and provide function for toggling it. This is for
 * use by the test suite programs
 */
static void test_led_setup(void)
{
    /* LED's are connected to GPIO1, GPIO2 and GPIO3 on port F */
    periph_clock_enable(RCC_GPIOF);
    gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1 | GPIO2 | GPIO3);
    gpio_set_output_config(GPIOF, GPIO_OTYPE_PP, GPIO_DRIVE_2MA, GPIO1 |GPIO2 | GPIO3);

    gpio_write(GPIOF, GPIO1 | GPIO2 | GPIO3, 0); // All 3-led's are off

    // gpio_set(GPIOF, GPIO2);
}

void test_led_toggle(uint8_t gpios)
{
    gpio_toggle(GPIOF, gpios);
}

/**
 * Callback from your main program to set up the board's hardware before
 * the kernel is started.
 */
int board_setup(void)
{
    /* Disable interrupts. This makes sure that the sys_tick_handler will
     * not be called before the first thread has been started.
     * Interrupts will be enabled by archFirstThreadRestore().
     */
    cm_disable_interrupts();

    /* configure system clock, user LED and UART */
    gpio_enable_ahb_aperture();
    clock_setup();
    test_led_setup();
    uart_setup(115200);

    /* Initialize SysTick counter */
    systick_setup();

    cm_enable_interrupts();

    /* Set exception priority levels. Make PendSv the lowest priority and
     * SysTick the second to lowest
     */
    nvic_set_priority(NVIC_PENDSV_IRQ, 0xFF);
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0xFE);

    return 0;
}
