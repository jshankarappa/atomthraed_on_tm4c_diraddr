/*
 * uart.h
 *
 *  Created on: 09-Oct-2018
 *      Author: jshankar
 */

#ifndef TM4C_UART_H
#define TM4C_UART_H

#include <stdint.h>
#include <stdio.h>
#include "inc/tm4c123gh6pm.h"
#include "cm3.h"

#define UART0_BASE          (0x4000C000U)
#define UART1_BASE          (0x4000D000U)
#define UART2_BASE          (0x4000E000U)
#define UART3_BASE          (0x4000F000U)
#define UART4_BASE          (0x40010000U)
#define UART5_BASE          (0x40011000U)
#define UART6_BASE          (0x40012000U)
#define UART7_BASE          (0x40013000U)

#define UART0               UART0_BASE
#define UART1               UART1_BASE
#define UART2               UART2_BASE
#define UART3               UART3_BASE
#define UART4               UART4_BASE
#define UART5               UART5_BASE
#define UART6               UART6_BASE
#define UART7               UART7_BASE

/* =============================================================================
 * UART registers
 * ---------------------------------------------------------------------------*/

/* UART data register */
#define UART_DR(uart_base)      MMIO32((uart_base) + 0x00)

/* UART Receive Status/Error Clear register */
#define UART_RSR(uart_base)     MMIO32((uart_base) + 0x04)
#define UART_ECR(uart_base)     MMIO32((uart_base) + 0x04)

/* UART Flag register */
#define UART_FR(uart_base)      MMIO32((uart_base) + 0x18)

/* UART IrDA Low-Power register */
#define UART_ILPR(uart_base)        MMIO32((uart_base) + 0x20)

/* UART Integer baudrate divisor */
#define UART_IBRD(uart_base)        MMIO32((uart_base) + 0x24)

/* UART Fractional baudrate divisor */
#define UART_FBRD(uart_base)        MMIO32((uart_base) + 0x28)

/* UART Line control */
#define UART_LCRH(uart_base)        MMIO32((uart_base) + 0x2C)

/* UART Control */
#define UART_CTL(uart_base)     MMIO32((uart_base) + 0x30)

/* UART Interrupt FIFO level select */
#define UART_IFLS(uart_base)        MMIO32((uart_base) + 0x34)

/* UART Interrupt mask */
#define UART_IM(uart_base)      MMIO32((uart_base) + 0x38)

/* UART Raw interrupt status */
#define UART_RIS(uart_base)     MMIO32((uart_base) + 0x3C)

/* UART Masked Interrupt status */
#define UART_MIS(uart_base)     MMIO32((uart_base) + 0x40)

/* UART Interrupt Clear */
#define UART_ICR(uart_base)     MMIO32((uart_base) + 0x44)

/* UART DMA control */
#define UART_DMACTL(uart_base)      MMIO32((uart_base) + 0x48)

/* UART LIN control */
#define UART_LCTL(uart_base)        MMIO32((uart_base) + 0x90)

/* UART LIN snap shot */
#define UART_LSS(uart_base)     MMIO32((uart_base) + 0x94)

/* UART LIN timer */
#define UART_LTIM(uart_base)        MMIO32((uart_base) + 0x98)

/* UART 9-Bit self address */
#define UART_9BITADDR(uart_base)    MMIO32((uart_base) + 0xA4)

/* UART 9-Bit self address mask */
#define UART_9BITAMASK(uart_base)   MMIO32((uart_base) + 0xA8)

/* UART Peripheral properties */
#define UART_PP(uart_base)          MMIO32((uart_base) + 0xFC0)

/* UART Clock configuration */
#define UART_CC(uart_base)          MMIO32((uart_base) + 0xFC8)

/* UART Peripheral Identification 4 */
#define UART_PERIPH_ID4(uart_base)  MMIO32((uart_base) + 0xFD0)

/* UART Peripheral Identification 5 */
#define UART_PERIPH_ID5(uart_base)  MMIO32((uart_base) + 0xFD4)

/* UART Peripheral Identification 6 */
#define UART_PERIPH_ID6(uart_base)  MMIO32((uart_base) + 0xFD8)

/* UART Peripheral Identification 7 */
#define UART_PERIPH_ID7(uart_base)  MMIO32((uart_base) + 0xFDC)

/* UART Peripheral Identification 0 */
#define UART_PERIPH_ID0(uart_base)  MMIO32((uart_base) + 0xFE0)

/* UART Peripheral Identification 1 */
#define UART_PERIPH_ID1(uart_base)  MMIO32((uart_base) + 0xFE4)

/* UART Peripheral Identification 2 */
#define UART_PERIPH_ID2(uart_base)  MMIO32((uart_base) + 0xFE8)

/* UART Peripheral Identification 3 */
#define UART_PERIPH_ID3(uart_base)  MMIO32((uart_base) + 0xFEC)

/* UART PrimeCell Identification 0 */
#define UART_PCELL_ID0(uart_base)   MMIO32((uart_base) + 0xFF0)

/* UART PrimeCell Identification 1 */
#define UART_PCELL_ID1(uart_base)   MMIO32((uart_base) + 0xFF4)

/* UART PrimeCell Identification 2 */
#define UART_PCELL_ID2(uart_base)   MMIO32((uart_base) + 0xFF8)

/* UART PrimeCell Identification 3 */
#define UART_PCELL_ID3(uart_base)   MMIO32((uart_base) + 0xFFC)

/** Word length */
#define UART_LCRH_WLEN_MASK     (3 << 5)

/** Data transmitted or received */
#define UART_DR_DATA_MASK       (0xFF << 0)

/* =============================================================================
 * UART_IFLS values
 * ---------------------------------------------------------------------------*/
/** UART Rx interrupt FIFO level select */
#define UART_IFLS_RXIFLSEL_MASK     (7 << 3)
#define UART_IFLS_RXIFLSEL_1_8      (0 << 3)
#define UART_IFLS_RXIFLSEL_1_4      (1 << 3)
#define UART_IFLS_RXIFLSEL_1_2      (2 << 3)
#define UART_IFLS_RXIFLSEL_3_4      (3 << 3)
#define UART_IFLS_RXIFLSEL_7_8      (4 << 3)
/** UART Tx interrupt FIFO level select */
#define UART_IFLS_TXIFLSEL_MASK     (7 << 0)
#define UART_IFLS_TXIFLSEL_7_8      (0 << 0)
#define UART_IFLS_TXIFLSEL_3_4      (1 << 0)
#define UART_IFLS_TXIFLSEL_1_2      (2 << 0)
#define UART_IFLS_TXIFLSEL_1_4      (3 << 0)
#define UART_IFLS_TXIFLSEL_1_8      (4 << 0)

/* =============================================================================
 * UART interrupt mask values
 *
 * These are interchangeable across UART_IM, UART_RIS, UART_MIS, and UART_ICR
 * registers.
 * ---------------------------------------------------------------------------*/
/** LIN mode edge 5 interrupt mask */
#define UART_IM_LME5IM          (1 << 15)
/** LIN mode edge 1 interrupt mask */
#define UART_IM_LME1IM          (1 << 14)
/** LIN mode sync break interrupt mask */
#define UART_IM_LMSBIM          (1 << 13)
/** Data Set Ready modem interrupt mask */
#define UART_IM_DSRIM           (1 << 3)
/** Data Carrier Detect modem interrupt mask */
#define UART_IM_DCDIM           (1 << 2)
/** Clear To Send modem interrupt mask */
#define UART_IM_CTSIM           (1 << 1)
/** Ring Indicator modem interrupt mask */
#define UART_IM_RIIM            (1 << 0)

/* =============================================================================
 * UART_LCTL values
 * ---------------------------------------------------------------------------*/
/** Sync break length */
#define UART_LCTL_BLEN_MASK     (3 << 4)
#define UART_LCTL_BLEN_16T      (3 << 4)
#define UART_LCTL_BLEN_15T      (2 << 4)
#define UART_LCTL_BLEN_14T      (1 << 4)
#define UART_LCTL_BLEN_13T      (0 << 4)
/** LIN master enable */
#define UART_LCTL_MASTER        (1 << 0)

/* =============================================================================
 * UART_9BITADDR values
 * ---------------------------------------------------------------------------*/
/** Enable 9-bit mode */
#define UART_UART_9BITADDR_9BITEN   (1 << 15)
/** Self-address for 9-bit mode */
#define UART_UART_9BITADDR_ADDR_MASK    (0xFF << 0)

/* =============================================================================
 * UART_PP values
 * ---------------------------------------------------------------------------*/
/** 9-bit support */
#define UART_UART_PP_NB         (1 << 1)
/** Smart Card support */
#define UART_UART_PP_SC         (1 << 0)

/* =============================================================================
 * UART_CC values
 * ---------------------------------------------------------------------------*/
/** UART baud clock source */
#define UART_CC_CS_MASK         (0xF << 0)

/* =============================================================================
 * Convenience enums
 * ---------------------------------------------------------------------------*/
enum uart_parity {
    UART_PARITY_NONE,
    UART_PARITY_ODD,
    UART_PARITY_EVEN,
    UART_PARITY_STICK_0,
    UART_PARITY_STICK_1,
};

enum uart_flowctl {
    UART_FLOWCTL_NONE,
    UART_FLOWCTL_RTS,
    UART_FLOWCTL_CTS,
    UART_FLOWCTL_RTS_CTS,
};

/**
 * UART interrupt masks
 *
 * These masks can be OR'ed together to specify more than one interrupt. For
 * example, (UART_INT_TXIM | UART_INT_TXIM) specifies both Rx and Tx Interrupt.
 */
enum uart_interrupt_flag {

    UART_INT_LME5   = UART_IM_LME5IM,
    UART_INT_LME1   = UART_IM_LME1IM,
    UART_INT_LMSB   = UART_IM_LMSBIM,
    UART_INT_9BIT   = UART_IM_9BITIM,
    UART_INT_OE = UART_IM_OEIM,
    UART_INT_BE = UART_IM_BEIM,
    UART_INT_PE = UART_IM_PEIM,
    UART_INT_FE = UART_IM_FEIM,
    UART_INT_RT = UART_IM_RTIM,
    UART_INT_TX = UART_IM_TXIM,
    UART_INT_RX = UART_IM_RXIM,
    UART_INT_DSR    = UART_IM_DSRIM,
    UART_INT_DCD    = UART_IM_DCDIM,
    UART_INT_CTS    = UART_IM_CTSIM,
    UART_INT_RI = UART_IM_RIIM,
};

/**
 * UART RX FIFO interrupt trigger levels
 *
 * The levels indicate how full the FIFO should be before an interrupt is
 * generated.  UART_FIFO_RX_TRIG_3_4 means that an interrupt is triggered when
 * the FIFO is 3/4 full. As the FIFO is 8 elements deep, 1/8 is equal to being
 * triggered by a single character.
 */
enum uart_fifo_rx_trigger_level {
    UART_FIFO_RX_TRIG_1_8   = UART_IFLS_RXIFLSEL_1_8,
    UART_FIFO_RX_TRIG_1_4   = UART_IFLS_RXIFLSEL_1_4,
    UART_FIFO_RX_TRIG_1_2   = UART_IFLS_RXIFLSEL_1_2,
    UART_FIFO_RX_TRIG_3_4   = UART_IFLS_RXIFLSEL_3_4,
    UART_FIFO_RX_TRIG_7_8   = UART_IFLS_RXIFLSEL_7_8
};

/**
 * UART TX FIFO interrupt trigger levels
 *
 * The levels indicate how empty the FIFO should be before an interrupt is
 * generated.  Note that this indicates the emptiness of the FIFO and not the
 * fullness. This is somewhat confusing, but it follows the wording of the
 * TM4C123GH6PM datasheet.
 *
 * UART_FIFO_TX_TRIG_3_4 means that an interrupt is triggered when the FIFO is
 * 3/4 empty. As the FIFO is 8 elements deep, 7/8 is equal to being triggered
 * by a single character.
 **/
enum uart_fifo_tx_trigger_level {
    UART_FIFO_TX_TRIG_7_8   = UART_IFLS_TXIFLSEL_7_8,
    UART_FIFO_TX_TRIG_3_4   = UART_IFLS_TXIFLSEL_3_4,
    UART_FIFO_TX_TRIG_1_2   = UART_IFLS_TXIFLSEL_1_2,
    UART_FIFO_TX_TRIG_1_4   = UART_IFLS_TXIFLSEL_1_4,
    UART_FIFO_TX_TRIG_1_8   = UART_IFLS_TXIFLSEL_1_8
};

/* =============================================================================
 * Function prototypes
 * ---------------------------------------------------------------------------*/
void uart_set_baudrate(uint32_t uart, uint32_t baud);
void uart_set_databits(uint32_t uart, uint8_t databits);
void uart_set_stopbits(uint32_t uart, uint8_t stopbits);
void uart_set_parity(uint32_t uart, enum uart_parity parity);
void uart_set_mode(uint32_t uart, uint32_t mode);
void uart_set_flow_control(uint32_t uart, enum uart_flowctl flow);
void uart_enable(uint32_t uart);
void uart_disable(uint32_t uart);
void uart_clock_from_piosc(uint32_t uart);
void uart_clock_from_sysclk(uint32_t uart);

void uart_send(uint32_t uart, uint16_t data);
uint16_t uart_recv(uint32_t uart);
void uart_wait_send_ready(uint32_t uart);
void uart_wait_recv_ready(uint32_t uart);
void uart_send_blocking(uint32_t uart, uint16_t data);
uint16_t uart_recv_blocking(uint32_t uart);

void uart_enable_rx_dma(uint32_t uart);
void uart_disable_rx_dma(uint32_t uart);
void uart_enable_tx_dma(uint32_t uart);
void uart_disable_tx_dma(uint32_t uart);

void uart_enable_fifo(uint32_t uart);
void uart_disable_fifo(uint32_t uart);
void uart_set_fifo_trigger_levels(uint32_t uart,
                  enum uart_fifo_rx_trigger_level rx_level,
                  enum uart_fifo_tx_trigger_level tx_level);

void uart_setup(uint32_t baud);

/* We inline FIFO full/empty checks as they are intended to be called from ISRs
 *
 * Determine if the TX fifo is full
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
static inline bool uart_is_tx_fifo_full(uint32_t uart)
{
    return UART_FR(uart) & UART_FR_TXFF;
}


/**
 * Determine if the TX fifo is empty
 *
 * param[in] uart UART block register address base @ref uart_reg_base
 */
static inline bool uart_is_tx_fifo_empty(uint32_t uart)
{
    return UART_FR(uart) & UART_FR_TXFE;
}

/**
 * Determine if the RX fifo is full
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
static inline bool uart_is_rx_fifo_full(uint32_t uart)
{
    return UART_FR(uart) & UART_FR_RXFF;
}

/**
 * Determine if the RX fifo is empty
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
static inline bool uart_is_rx_fifo_empty(uint32_t uart)
{
    return UART_FR(uart) & UART_FR_RXFE;
}

void uart_enable_interrupts(uint32_t uart, enum uart_interrupt_flag ints);
void uart_disable_interrupts(uint32_t uart, enum uart_interrupt_flag ints);
void uart_enable_rx_interrupt(uint32_t uart);
void uart_disable_rx_interrupt(uint32_t uart);
void uart_enable_tx_interrupt(uint32_t uart);
void uart_disable_tx_interrupt(uint32_t uart);
void uart_clear_interrupt_flag(uint32_t uart, enum uart_interrupt_flag ints);

/* Let's keep this one inlined. It's designed to be used in ISRs
 * Determine if interrupt is generated by the given source
 *
 * @param[in] uart UART block register address base
 * @param[in] source source to check.
 */
static inline bool uart_is_interrupt_source(uint32_t uart, enum uart_interrupt_flag source)
{
    return UART_MIS(uart) & source;
}

#endif /* TM4C_UART_H */
