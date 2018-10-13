/*
 * cm3.h
 *
 *  Created on: 29-Sep-2018
 *      Author: jshankar
 */

#ifndef CM3_H_
#define CM3_H_

#include <stdbool.h>
#include <stdint.h>

/* Generic memory-mapped I/O accessor functions */
#define MMIO8(addr)         (*(volatile uint8_t *)(addr))
#define MMIO16(addr)        (*(volatile uint16_t *)(addr))
#define MMIO32(addr)        (*(volatile uint32_t *)(addr))

/* Private peripheral bus - Internal */
#define PPBI_BASE           (0xE0000000U)
#define SCS_BASE            (PPBI_BASE + 0xE000)
/* SCB: System Control Block */
#define SCB_BASE            (SCS_BASE + 0x0D00)
#define SYSCTL_BASE         (0x400FE000U)
#define SYSCTL_GPIOHBCTL    MMIO32(SYSCTL_BASE + 0x06C)
/* SYS_TICK: System Timer */
#define SYS_TICK_BASE       (SCS_BASE + 0x0010)
/* NVIC: Nested Vector Interrupt Controller */
#define NVIC_BASE           (SCS_BASE + 0x0100)

#define SYSCTL_PLLSTAT      MMIO32(SYSCTL_BASE + 0x168)
#define SYSCTL_RCC          MMIO32(SYSCTL_BASE + 0x060)
#define SYSCTL_RCC2         MMIO32(SYSCTL_BASE + 0x070)

/** PENDSVSET: PendSV set-pending bit */
#define SCB_ICSR_PENDSVSET  (1 << 28)
/** ICSR: Interrupt Control State Register */
#define SCB_ICSR            MMIO32(SCB_BASE + 0x04)
/** System Handler Priority 8 bits Registers, SHPR1/2/3.
 * Note: 12 8bit Registers
 */
#define SCS_SHPR(ipr_id)    MMIO8(SCS_BASE + 0xD18 + (ipr_id))
/* NVIC_BASE + 0x220 (0xE000 E320 - 0xE000 E3FF): Reserved */

/* IPR: Interrupt Priority Registers */
/* Note: 240 8-bit Registers */
/* Note: 32 8-bit Registers on CM0 */
#define NVIC_IPR(ipr_id)    MMIO8(NVIC_BASE + 0x300 + (ipr_id))
#define NVIC_IRQ_COUNT 139
/* irq number -3 reserved */
#define NVIC_PENDSV_IRQ     -2
#define NVIC_SYSTICK_IRQ    -1

static inline void __dmb()
{
    __asm__ volatile ("dmb");
}

static inline void nvic_set_priority(uint8_t irqn, uint8_t priority)
{
    /* this is quite a hack and alludes to the negative interrupt numbers assigned
     * to the system interrupts. better handling would mean signed integers. */
    if (irqn >= NVIC_IRQ_COUNT) {
        /* Cortex-M  system interrupts */
        SCS_SHPR((irqn & 0xF) - 4) = priority;
    } else {
        /* Device specific interrupts */
        NVIC_IPR(irqn) = priority;
    }
}

/*---------------------------------------------------------------------------
 *
 * Disable the interrupt mask and enable interrupts globally
 */
static inline void cm_enable_interrupts(void)
{
    __asm__ volatile ("CPSIE I\n");
}

/*---------------------------------------------------------------------------
 *
 * Mask all interrupts globally
 */
static inline void cm_disable_interrupts(void)
{
    __asm__ volatile ("CPSID I\n");
}

/*---------------------------------------------------------------------------
 *
 * Disable the HardFault mask and enable fault interrupt globally
 */
static inline void cm_enable_faults(void)
{
    __asm__ volatile ("CPSIE F\n");
}

/*---------------------------------------------------------------------------
 *
 * Mask the HardFault interrupt globally
 */
static inline void cm_disable_faults(void)
{
    __asm__ volatile ("CPSID F\n");
}

/*---------------------------------------------------------------------------
 *
 * Checks, if interrupts are masked (disabled).
 *
 * @returns true, if interrupts are disabled.
 */
__attribute__((always_inline))
static inline bool cm_is_masked_interrupts(void)
{
    register uint32_t result;
    __asm__ volatile ("MRS %0, PRIMASK"  : "=r" (result));
    return result;
}

/*---------------------------------------------------------------------------
 *
 * Checks, if HardFault interrupt is masked (disabled).
 *
 * returns bool true, if HardFault interrupt is disabled.
 */
__attribute__((always_inline))
static inline bool cm_is_masked_faults(void)
{
    register uint32_t result;
    __asm__ volatile ("MRS %0, FAULTMASK"  : "=r" (result));
    return result;
}

/*---------------------------------------------------------------------------
 *
 * This function switches the mask of the interrupts. If mask is true, the
 * interrupts will be disabled. The result of this function can be used for
 * restoring previous state of the mask.
 *
 * param[in] mask uint32_t New state of the interrupt mask
 * returns uint32_t old state of the interrupt mask
 */
__attribute__((always_inline))
static inline uint32_t cm_mask_interrupts(uint32_t mask)
{
    register uint32_t old;
    __asm__ __volatile__("MRS %0, PRIMASK"  : "=r" (old));
    __asm__ __volatile__(""  : : : "memory");
    __asm__ __volatile__("MSR PRIMASK, %0" : : "r" (mask));
    return old;
}

/*---------------------------------------------------------------------------
 *
 * This function switches the mask of the HardFault interrupt. If mask is true,
 * the HardFault interrupt will be disabled. The result of this function can be
 * used for restoring previous state of the mask.
 *
 * param[in] mask uint32_t New state of the HardFault interrupt mask
 * returns uint32_t old state of the HardFault interrupt mask
 */
__attribute__((always_inline))
static inline uint32_t cm_mask_faults(uint32_t mask)
{
    register uint32_t old;
    __asm__ __volatile__ ("MRS %0, FAULTMASK"  : "=r" (old));
    __asm__ __volatile__ (""  : : : "memory");
    __asm__ __volatile__ ("MSR FAULTMASK, %0" : : "r" (mask));
    return old;
}

/*===========================================================================*/
/** Cortex Core Atomic support Defines
 *
 * Atomic operation support
 *
 */

#if !defined(__DOXYGEN__)
/* Do not populate this definition outside */
static inline uint32_t __cm_atomic_set(uint32_t *val)
{
    return cm_mask_interrupts(*val);
}

#define __CM_SAVER(state)                   \
    __val = (state),                    \
    __save __attribute__((__cleanup__(__cm_atomic_set))) =  \
    __cm_atomic_set(&__val)

#endif /* !defined(__DOXYGEN) */


/*---------------------------------------------------------------------------*/
/** Cortex M Atomic Declare block
 *
 * This macro disables interrupts for the next command or block of code. The
 * interrupt mask is automatically restored after exit of the boundary of the
 * code block. Therefore restore of interrupt is done automatically after call
 * of return or goto control sentence jumping outside of the block.
 *
 * warning : The usage of sentences break or continue is prohibited in the block
 * due to implementation of this macro!
 *
 * note : It is safe to use this block inside normal code and in interrupt
 * routine.
 *
 * example 1: Basic usage of atomic block
 *
 * uint64_t value;      // This value is used somewhere in interrupt
 *
 * ...
 *
 * CM_ATOMIC_BLOCK() {          // interrupts are masked in this block
 *     value = value * 1024 + 651;  // access value as atomic
 * }                    // interrupts is restored automatically
 *
 * example 2: Use of return inside block:
 *
 * uint64_t value;      // This value is used somewhere in interrupt
 *
 * ...
 *
 * uint64_t allocval(void)
 * {
 *     CM_ATOMIC_BLOCK() {      // interrupts are masked in this block
 *         value = value * 1024 + 651;  // do long atomic operation
 *         return value;        // interrupts is restored automatically
 *     }
 * }
 */
#if defined(__DOXYGEN__)
#define CM_ATOMIC_BLOCK()
#else /* defined(__DOXYGEN__) */
#define CM_ATOMIC_BLOCK()                       \
    for (uint32_t __CM_SAVER(true), __my = true; __my; __my = false)
#endif /* defined(__DOXYGEN__) */

/*---------------------------------------------------------------------------*/
/** Cortex M Atomic Declare context
 *
 * This macro disables interrupts in the current block of code from the place
 * where it is defined to the end of the block. The interrupt mask is
 * automatically restored after exit of the boundary of the code block.
 * Therefore restore of interrupt is done automatically after call of return,
 * continue, break, or goto control sentence jumping outside of the block.
 *
 * note : This function is intended for use in for- cycles to enable the use of
 * break and contine sentences inside the block, and for securing the atomic
 * reader-like functions.
 *
 * note : It is safe to use this block inside normal code and in interrupt
 * routine.
 *
 * example 1: Basic usage of atomic context
 *
 * uint64_t value;      // This value is used somewhere in interrupt
 *
 * ...
 *
 * for (int i=0;i < 100; i++) {
 *     CM_ATOMIC_CONTEXT();     // interrupts are masked in this block
 *     value += 100;            // access value as atomic
 *     if ((value % 16) == 0) {
 *         break;           // restore interrupts and break cycle
 *     }
 * }                    // interrupts is restored automatically
 *
 * example 2: Usage of atomic context inside atomic reader fcn.
 *
 * uint64_t value;      // This value is used somewhere in interrupt
 *
 * ...
 *
 * uint64_t getnextval(void)
 * {
 *     CM_ATOMIC_CONTEXT(); // interrupts are masked in this block
 *     value = value + 3;   // do long atomic operation
 *     return value;        // interrupts is restored automatically
 * }
 */

#endif /* CM3_H_ */
