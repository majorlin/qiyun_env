/*
 * Copyright (c) 2020 NXP
 * ******************************************************************************
 * File: syscall.c
 * Created Date: Thursday, January 9th 2020, 1:54:19 pm
 * Author: Major Lin
 * -----
 * Last Modified: Thu Feb 06 2020
 * Modified By: Major Lin
 * -----
 * 
 * -----
 * HISTORY:
 * Date      	By           	Comments
 * ----------	-------------	----------------------------------------------------------
 * ******************************************************************************
 */
#include <sys/stat.h>
#include <stdio.h>
#include <errno.h>
#include "sys/times.h"
#include "common.h"

void setUp(){}
void tearDown(){}

// external defined printf support functions
extern char env_get_char();
extern void env_put_char(char c);

int _close(int file) { return -1; }
int _isatty(int file) { return 1; }

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR; // Character device
    return 0;
}

int _write(int file, char *p, int len)
{
    // only write to stdout
    for(int i=0; i<len; i++) {
        env_put_char(p[i]);
    }
    return len;
}

int _read(int file, char *p, int len)
{
    int i = len;
    while(i > 0) {
        *p++ = env_get_char();
        i--;
    }
    return len - i;
}

void _exit(int status)
{
    char * result = status ? "FAIL" : "PASS";
    printf("****** <TEST_%s> *******\r\n", result);
    printf("****** <TEST_END> *******\r\n");
#ifdef QEMU_SIMULATOR
    asm("WFI");
#endif
    while (1);
}
int _lseek(int i, int j, int k)
{
    return -1;
}
/*!
 * @brief Function to override ARMGCC default function _sbrk
 *
 * _sbrk is called by malloc. ARMGCC default _sbrk compares "SP" register and
 * heap end, if heap end is larger than "SP", then _sbrk returns error and
 * memory allocation failed. This function changes to compare __HeapLimit with
 * heap end.
 */
caddr_t _sbrk(int incr)
{
    extern char end __asm("end");
    extern char heap_limit __asm("__HeapLimit");
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == NULL)
        heap_end = &end;

    prev_heap_end = heap_end;

    if ((unsigned int)heap_end + incr > (unsigned int)(&heap_limit))
    {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end = (char *)((unsigned int)heap_end + incr);

    return (caddr_t)prev_heap_end;
}


clock_t _times (struct tms *buffer)
{
  return -1;
}
