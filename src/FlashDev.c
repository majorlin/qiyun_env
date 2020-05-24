/*********************************************************************
 *            (c) 1995 - 2018 SEGGER Microcontroller GmbH             *
 *                        The Embedded Experts                        *
 *                           www.segger.com                           *
 **********************************************************************
 ----------------------------------------------------------------------
File    : FlashDev.c
Purpose : Flash device description Template
--------  END-OF-HEADER  ---------------------------------------------
*/

#include "FlashOS.h"

// struct FlashDevice const FlashDevice __attribute__ ((section ("DevDscr"))) =  {
struct FlashDevice const FlashDevice =  {
    ALGO_VERSION,              // Algo version
    "Internal flash", // Flash device name
    ONCHIP,                    // Flash device type
    0x00000000,                // Flash base address
    0x00020000,                // Total flash device size in Bytes (128 KB)
    16,                       // Page Size (number of bytes that will be passed to ProgramPage(). May be multiple of min alignment in order to reduce overhead for calling ProgramPage multiple times
    0,                         // Reserved, should be 0
    0xFF,                      // Flash erased value
    100,                       // Program page timeout in ms
    6000,                      // Erase sector timeout in ms
    //
    // Flash sector layout definition
    //
    {
        {0x00002000, 0x00000000},   // 16 *  8 KB =  128 KB
        {0xFFFFFFFF, 0xFFFFFFFF}
    }    // Indicates the end of the flash sector layout. Must be present.
};
