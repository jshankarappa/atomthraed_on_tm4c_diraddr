#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>          //Needed for caddr_t
#include "uart.h"

//extern char *heap_end;
char *heap_end = 0;

caddr_t _sbrk(unsigned int incr)
{
    extern unsigned long __heap_start__;
    extern unsigned long __heap_end__;
    static char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = (caddr_t)&__heap_start__;
    }

    prev_heap_end = heap_end;

    if (heap_end + incr > (caddr_t)&__heap_end__) {
        return (caddr_t)0;
    }

    heap_end += incr;

    return (caddr_t) prev_heap_end;
}

int _close(int file)
{
    return -1;
}

int _fstat(int file)
{
    return -1;
}

int _isatty(int file)
{
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    return -1;
}

int _open(const char *name, int flags, int mode)
{
    return -1;
}

int _read(int file, char *ptr, int len)
{
    unsigned int i;
    for( i = 0; i < len; i++ ){
        ptr[i] = (char)uart_recv_blocking(UART0);
    }
    return len;
}

int _write(int file, char *ptr, unsigned int len)
{
    unsigned int i;
    for(i = 0; i < len; i++){
        uart_send_blocking(UART0, ptr[i]);
    }
    return i;
}
