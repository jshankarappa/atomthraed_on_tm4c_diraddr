/*
 * uart.c
 *
 *  Created on: 09-Oct-2018
 *      Author: jshankar
 */

#include "uart.h"
#include "rcc.h"
#include "gpio.h"

/** UART configuration
 * Enabling and configuring the UART
 *
 * Enabling the UART is a two step process. The GPIO on which the UART resides
 * must be enabled, and the UART pins must be configured as alternate function,
 * digital pins. Pins must also be muxed to the appropriate alternate function.
 *
 * The second step involves enabling and the UART itself. The UART should be
 * disabled while it is being configured.
 *  -# The UART clock must be enabled with @ref periph_clock_enable().
 *  -# The UART must be disabled with @ref uart_disable().
 *  -# The UART clock source should be chosen before setting the baudrate.
 *  -# Baudrate, data bits, stop bit length, and parity can be configured.
 *  -# If needed, enable CTS or RTS lines via the @ref uart_set_flow_control().
 *  -# The UART can now be enabled with @ref uart_enable().
 *
 */

void uart_setup(uint32_t baud)
{
    periph_clock_enable(RCC_GPIOA);
    gpio_set_af(GPIOA, 1, GPIO0 | GPIO1);

    periph_clock_enable(RCC_UART0);

    /* We need a brief delay before we can access UART config registers */
    __asm__("nop");

    /* Disable the UART while we mess with its settings */
    uart_disable(UART0);

    /* Configure the UART clock source as precision internal oscillator */
    uart_clock_from_piosc(UART0);

    /* Set communication parameters */
    uart_set_baudrate(UART0, baud);
    uart_set_databits(UART0, 8);
    uart_set_parity(UART0, UART_PARITY_NONE);
    uart_set_stopbits(UART0, 1);

    /* Now that we're done messing with the settings, enable the UART */
    uart_enable(UART0);
}

/**
 * Enable the UART
 *
 * Enable the UART. The Rx and Tx lines are also enabled.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
void uart_enable(uint32_t uart)
{
    UART_CTL(uart) |= (UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE);
}

/**
 * Disable the UART
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
void uart_disable(uint32_t uart)
{
    UART_CTL(uart) &= ~UART_CTL_UARTEN;
}

/**
 * Set UART baudrate
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @param[in] baud Baud rate in bits per second (bps).*
 */
void uart_set_baudrate(uint32_t uart, uint32_t baud)
{
    uint32_t clock;

    /* Are we running off the internal clock or system clock? */
    if (UART_CC(uart) == UART_CC_CS_PIOSC) {
        clock = 16000000;
    } else {
        clock = rcc_get_system_clock_frequency();
    }

    /* Find the baudrate divisor */
    uint32_t div = (((clock * 8) / baud) + 1) / 2;

    /* Set the baudrate divisors */
    UART_IBRD(uart) = div / 64;
    UART_FBRD(uart) = div % 64;
}

/**
 * Set UART databits
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @param[in] databits number of data bits per transmission.
 */
void uart_set_databits(uint32_t uart, uint8_t databits)
{
    uint32_t reg32, bitint32_t;

    /* This has the same effect as using UART_LCRH_WLEN_5/6/7/8 directly */
    bitint32_t = (databits - 5) << 5;

    /* TODO: What about 9 data bits? */

    reg32 = UART_LCRH(uart);
    reg32 &= ~UART_LCRH_WLEN_MASK;
    reg32 |= bitint32_t;
    UART_LCRH(uart) = reg32;
}

/**
 * Set UART stopbits
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @param[in] bits the requested number of stopbits, either 1 or 2.
 */
void uart_set_stopbits(uint32_t uart, uint8_t stopbits)
{
    if (stopbits == 2) {
        UART_LCRH(uart) |= UART_LCRH_STP2;
    } else {
        UART_LCRH(uart) &= ~UART_LCRH_STP2;
    }
}

/**
 * Set UART parity
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @param[in] bits the requested parity scheme.
 */
void uart_set_parity(uint32_t uart, enum uart_parity parity)
{
    uint32_t reg32;

    reg32 = UART_LCRH(uart);
    reg32 |= UART_LCRH_PEN;
    reg32 &= ~(UART_LCRH_SPS | UART_LCRH_EPS);

    switch (parity) {
    case UART_PARITY_NONE:
        /* Once we disable parity the other bits are meaningless */
        UART_LCRH(uart) &= ~UART_LCRH_PEN;
        return;
    case UART_PARITY_ODD:
        break;
    case UART_PARITY_EVEN:
        reg32 |= UART_LCRH_EPS;
        break;
    case UART_PARITY_STICK_0:
        reg32 |= (UART_LCRH_SPS | UART_LCRH_EPS);
        break;
    case UART_PARITY_STICK_1:
        reg32 |= UART_LCRH_SPS;
        break;
    }

    UART_LCRH(uart) = reg32;
}

/**
 * Set the flow control scheme
 *
 * Set the flow control scheme by enabling or disabling RTS and CTS lines. This
 * will only have effect if the given UART supports the RTS and CTS lines.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @param[in] flow The flow control scheme to use (none, RTS, CTS or both) \n
 *                 UART_FLOWCTL_RTS -- enable the RTS line \n
 *                 UART_FLOWCTL_CTS -- enable the CTS line \n
 *                 UART_FLOWCTL_RTS_CTS -- enable both RTS and CTS lines
 */
void uart_set_flow_control(uint32_t uart, enum uart_flowctl flow)
{
    uint32_t reg32 = UART_CTL(uart);

    reg32 &= ~(UART_CTL_RTSEN | UART_CTL_CTSEN);

    if (flow == UART_FLOWCTL_RTS) {
        reg32 |= UART_CTL_RTSEN;
    } else if (flow == UART_FLOWCTL_CTS) {
        reg32 |= UART_CTL_CTSEN;
    } else if (flow == UART_FLOWCTL_RTS_CTS) {
        reg32 |= (UART_CTL_RTSEN | UART_CTL_CTSEN);
    }

    UART_CTL(uart) = reg32;
}

/**
 * Clock the UART module from the internal oscillator
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
void uart_clock_from_piosc(uint32_t uart)
{
    UART_CC(uart) = UART_CC_CS_PIOSC;
}

/**
 * Clock the UART module from the system clock
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
void uart_clock_from_sysclk(uint32_t uart)
{
    UART_CC(uart) = UART_CC_CS_SYSCLK;
}

/**

 * uart_send_recv UART transmission and reception
 *
 * Sending and receiving data through the UART
 *
 * Primitives for sending and receiving data are provided, @ref uart_send() and
 * @ref uart_recv(). These primitives do not check if data can be transmitted
 * or wait for data. If waiting until data is available or can be transmitted is
 * desired, blocking primitives are also available, @ref uart_send_blocking()
 * and @ref uart_recv_blocking().
 *
 * These primitives only handle one byte at at time, and thus may be unsuited
 * for some applications. You may also consider using @ref uart_dma.
 **/

/**
 * UART Send a Data Word.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @param[in] data data to send.
 */
void uart_send(uint32_t uart, uint16_t data)
{
    data &= 0xFF;
    UART_DR(uart) = data;
}

/**
 * UART Read a Received Data Word.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @return data from the Rx FIFO.
 **/
uint16_t uart_recv(uint32_t uart)
{
    return UART_DR(uart) & UART_DR_DATA_MASK;
}

/**
 * UART Send Data Word with Blocking
 *
 * Blocks until the transmit data FIFO can accept the next data word for
 * transmission.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 **/
void uart_send_blocking(uint32_t uart, uint16_t data)
{
    uart_wait_send_ready(uart);
    uart_send(uart, data);
}

/**
 * UART Read a Received Data Word with Blocking.
 *
 * Wait until a data word has been received then return the word.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @return data from the Rx FIFO.
 */
uint16_t uart_recv_blocking(uint32_t uart)
{
    uart_wait_recv_ready(uart);
    return uart_recv(uart);
}

/**
 * UART Wait for Transmit Data Buffer Not Full
 *
 * Blocks until the transmit data FIFO is not empty and can accept the next data word.
 *
 * Even if the FIFO is not empty, this function will return as long as there is
 * room for at least one more word.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
void uart_wait_send_ready(uint32_t uart)
{
    /* Wait until the Tx FIFO is no longer full */
    while (UART_FR(uart) & UART_FR_TXFF);
}

/**
 * UART Wait for Received Data Available
 *
 * Blocks until the receive data FIFO holds a at least valid received data word.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 **/
void uart_wait_recv_ready(uint32_t uart)
{
    /* Wait until the Tx FIFO is no longer empty */
    while( UART_FR(uart) & UART_FR_RXFE );
}

/** uart_irq UART Interrupt control
 * Configuring interrupts from the UART
 *
 * To have an event generate an interrupt, its interrupt source must be
 * unmasked. This can be achieved with @ref uart_enable_interrupts(). Interrupts
 * which are no longer needed can be disabled through
 * @ref uart_disable_interrupts().
 *
 * In order for the interrupt to generate an IRQ and a call to the interrupt
 * service routine, the interrupt for the target UART must be routed through the
 * NVIC with @ref nvic_enable_irq().
 *
 * Enabling an interrupt is as simple as unmasking the desired interrupt, and
 * routing the desired UART's interrupt through the NVIC.
 *
 * If a more than one interrupt is to be enabled at one time, the interrupts
 * can be enabled by a single call to @ref uart_enable_interrupts().
 * For example:
 *  // Unmask receive, CTS, and RI, interrupts
 *
 * After interrupts are properly enabled and routed through the NVIC, when an
 * event occurs, the appropriate IRQ flag is set by hardware, and execution
 * jumps to the UART ISR. The ISR should query the IRQ flags to determine which
 * event caused the interrupt. For this, use @ref uart_is_interrupt_source(),
 * with the desired UART_INT flag. After one or more interrupt sources are
 * serviced, the IRQ flags must be cleared by the ISR. This can be done with
 * @ref uart_clear_interrupt_flag().
 **/

/**
 * Enable Specific UART Interrupts
 *
 * Enable any combination of interrupts. Interrupts may be OR'ed together to
 * enable them with one call. For example, to enable both the RX and CTS
 * interrupts, pass (UART_INT_RX | UART_INT_CTS)
 *
 * Note that the NVIC must be enabled and properly configured for the interrupt
 * to be routed to the CPU.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @param[in] ints Interrupts which to enable. Any combination of interrupts may
 *                 be specified by OR'ing then together
 **/
void uart_enable_interrupts(uint32_t uart, enum uart_interrupt_flag ints)
{
    UART_IM(uart) |= ints;
}

/**
 * Enable Specific UART Interrupts
 *
 * Disabe any combination of interrupts. Interrupts may be OR'ed together to
 * disable them with one call. For example, to disable both the RX and CTS
 * interrupts, pass (UART_INT_RX | UART_INT_CTS)
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @param[in] ints Interrupts which to disable. Any combination of interrupts
 *         may be specified by OR'ing then together
 **/
void uart_disable_interrupts(uint32_t uart, enum uart_interrupt_flag ints)
{
    UART_IM(uart) &= ~ints;
}

/**
 * Enable the UART Receive Interrupt.
 *
 * Note that the NVIC must be enabled and properly configured for the interrupt
 * to be routed to the CPU.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
void uart_enable_rx_interrupt(uint32_t uart)
{
    uart_enable_interrupts(uart, UART_INT_RX);
}

/**
 * Disable the UART Receive Interrupt.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 **/
void uart_disable_rx_interrupt(uint32_t uart)
{
    uart_disable_interrupts(uart, UART_INT_RX);
}

/**
 * Enable the UART Transmit Interrupt.
 *
 * Note that the NVIC must be enabled and properly configured for the interrupt
 * to be routed to the CPU.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 **/
void uart_enable_tx_interrupt(uint32_t uart)
{
    uart_enable_interrupts(uart, UART_INT_TX);
}

/**
 * Disable the UART Transmit Interrupt.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
void uart_disable_tx_interrupt(uint32_t uart)
{
    uart_disable_interrupts(uart, UART_INT_TX);
}

/**
 * Mark interrupt as serviced
 *
 * After an interrupt is services, its flag must be cleared. If the flag is not
 * cleared, then execution will jump back to the start of the ISR after the ISR
 * returns.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @param[in] ints Interrupts which to clear. Any combination of interrupts may
 *                 be specified by OR'ing then together
 **/
void uart_clear_interrupt_flag(uint32_t uart, enum uart_interrupt_flag ints)
{
    UART_ICR(uart) |= ints;
}

/** uart_fifo UART FIFO control
 *  Enabling and controlling UART FIFO
 *
 * The UART on the LM4F can either be used with a single character TX and RX
 * buffer, or with a 8 character TX and RX FIFO. In order to use the FIFO it
 * must be enabled, this is done with uart_enable_fifo() and can be disabled
 * again with uart_disable_fifo().  On reset the FIFO is disabled, and it must
 * be explicitly be enabled.
 *
 * When enabling the UART FIFOs, RX and TX interrupts are triggered according
 * to the amount of data in the FIFOs. For the RX FIFO the trigger level is
 * defined by how full the FIFO is.  The TX FIFO trigger level is defined by
 * how empty the FIFO is instead.
 *
 * For example, to enable the FIFOs and trigger interrupts for a single
 * received and single transmitted character:
 *
 *  uart_enable_fifo(UART0);
 *  uart_set_fifo_trigger_levels(UART0, UART_FIFO_RX_TRIG_1_8,
 *                               UART_FIFO_TX_TRIG_7_8);
 *
 **/


/**
 * Enable FIFO for the UART.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 **/
void uart_enable_fifo(uint32_t uart)
{
    UART_LCRH(uart) |= UART_LCRH_FEN;
}

/**
 * Disable FIFO for the UART.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
void uart_disable_fifo(uint32_t uart)
{
    UART_LCRH(uart) &= ~UART_LCRH_FEN;
}

/**
 * Set the FIFO trigger levels.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @param[in] rx_level Trigger level for RX FIFO
 * @param[in] tx_level Trigger level for TX FIFO
 */
void uart_set_fifo_trigger_levels(uint32_t uart,
                  enum uart_fifo_rx_trigger_level rx_level,
                  enum uart_fifo_tx_trigger_level tx_level)
{
    UART_IFLS(uart) = rx_level | tx_level;
}

