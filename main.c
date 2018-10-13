/*
 * Copyright (c) 2015, Tido Klaassen. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. No personal names or organizations' names associated with the
 *    Atomthreads project may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE ATOMTHREADS PROJECT AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boards/ek-tm4c123gxl/gpio.h>
#include <boards/ek-tm4c123gxl/uart.h>
#include <stdint.h>
#include <stdio.h>
#include "atom.h"
#include "atomport-private.h"
#include "atomtimer.h"

#define STACK_SIZE      1024
#define THREAD_PRIO     42

static ATOM_TCB main_tcb;

static uint8_t thread_stacks[2][STACK_SIZE];

static void main_thread_func(uint32_t data);

/**
 * Example for a stand-alone board application
 */
extern int board_setup(void);

int main(void)
{
    int8_t status;

    board_setup();

    /**
     * Initialise OS and set up idle thread
     */
    status = atomOSInit(&thread_stacks[0][0], STACK_SIZE, FALSE);

    if(status == ATOM_OK) {
        /* Set up main thread */
        status = atomThreadCreate(&main_tcb, THREAD_PRIO, main_thread_func, 0,
                                  &thread_stacks[1][0], STACK_SIZE, TRUE);

        if(status == ATOM_OK) {
            atomOSStart();
        }
    }

    while(1) {
        ;
    }

    /* We will never get here */
    return 0;
}

extern void test_led_toggle(uint8_t gpios);
void PrintMemoryLayout(void);
char* GetSP(void);
static void main_thread_func(uint32_t data __maybe_unused)
{
    /* Print message */
    printf("Hello, world!\n\r");
//    PrintMemoryLayout();

    /* Loop forever and blink the LED */
    while(1) {
        test_led_toggle(GPIO2);
        atomTimerDelay(SYSTEM_TICKS_PER_SEC);
    }
}

void PrintMemoryLayout(void)
{
    extern unsigned long _text;
    extern unsigned long _etext;
    extern unsigned long _data;
    extern unsigned long _edata;
    extern unsigned long _bss;
    extern unsigned long _ebss;
    extern unsigned long _stack_bottom;
    extern unsigned long _stack_top;
    extern unsigned long _heap_bottom;
    extern char *heap_end;
    extern unsigned long _heap_top;

    printf("_text          = 0x%X\n\r",      (unsigned int) &_text);
    printf("_etext         = 0x%X\n\r",      (unsigned int) &_etext);
    printf("Sizeof(.text)  = %d bytes\n\r",  (&_etext - &_text) * 4);
    printf("---------------------------\n\r");
    printf("_data          = 0x%X\n\r",      (unsigned int) &_data);
    printf("_edata         = 0x%X\n\r",      (unsigned int) &_edata);
    printf("Sizeof(.data)  = %d bytes\n\r",  (&_edata - &_data) * 4);
    printf("---------------------------\n\r");
    printf("_bss           = 0x%X\n\r",      (unsigned int) &_bss);
    printf("_ebss          = 0x%X\n\r",      (unsigned int) &_ebss);
    printf("Sizeof(.bss)   = %d bytes\n\r",  (&_ebss - &_bss) * 4);
    printf("---------------------------\n\r");
    printf("_heap_bottom   = 0x%X\n\r",      (unsigned int) &_heap_bottom);
    printf("_heap_end      = 0x%X",        (unsigned int) heap_end);
    printf(" : usage: %d\n\r",               (heap_end - (char *) &_heap_bottom));
    printf("_heap_top      = 0x%X\n\r",      (unsigned int) &_heap_top);
    printf("Sizeof(heap)   = %d bytes\n\r",  (&_heap_top - &_heap_bottom) * 4);
    printf("---------------------------\n\r");
    printf("_stack_bottom  = 0x%X\n\r",      (unsigned int) &_stack_bottom);
    printf("StackPointer   = 0x%X",        (unsigned int) GetSP());
    printf(" : usage: %d\n\r",               ( (char *) &_stack_top - GetSP()));
    printf("_stack_top     = 0x%X\n\r",      (unsigned int) &_stack_top);
    printf("Sizeof(stack)  = %d bytes\n\r",  (&_stack_top - &_stack_bottom) * 4);
    printf("---------------------------\n\r");
}

char* GetSP(void)
{
    char *stack_ptr;
    __asm (
        "mov %[stack_ptr], sp \n"
        : [stack_ptr] "=r" (stack_ptr)
        :
    );
    return stack_ptr;
}
