/*********************************************************************
*            (c) 1995 - 2018 SEGGER Microcontroller GmbH             *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
----------------------------------------------------------------------
File    : FlashPrg.c
Purpose : Implementation of RAMCode template
--------  END-OF-HEADER  ---------------------------------------------
*/
#include "FlashOS.h"

/*********************************************************************
*
*       Defines (configurable)
*
**********************************************************************
*/
#define PAGE_SIZE_SHIFT          (3)      // The smallest program unit (one page) is 8 byte in size
#define SECTOR_SIZE_SHIFT        (12)     // 4096 sector size
//
// Some flash types require a native verify function as the memory is not memory mapped available (e.g. eMMC flashes).
// If the verify function is implemented in the algorithm, it will be used by the J-Link DLL during compare / verify
// independent of what verify type is configured in the J-Link DLL.
// Please note, that SEGGER does not recommend to use this function if the flash can be memory mapped read
// as this may can slow-down the compare / verify step.
//
#define SUPPORT_NATIVE_VERIFY        (0)
#define SUPPORT_NATIVE_READ_FUNCTION (0)
#define SUPPORT_BLANK_CHECK          (0)
#define SUPPORT_ERASE_CHIP           (0)
#define SUPPORT_SEGGER_OPEN_Program  (1)
#define SUPPORT_SEGGER_OPEN_ERASE    (1)

/*********************************************************************
*
*       Types
*
**********************************************************************
*/

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
//
// We use this dummy variable to make sure that the PrgData
// section is present in the output elf-file as this section
// is mandatory in current versions of the J-Link DLL 
//
static volatile int _Dummy;

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

/*********************************************************************
*
*       _FeedWatchdog
*
*  Function description
*    Feeds the watchdog. Needs to be called during RAMCode execution
*    in case of an watchdog is active.
*/
static void _FeedWatchdog(void) {
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

/*********************************************************************
*
*       Init
*
*  Function description
*    Handles the initialization of the flash module.
*
*  Parameters
*    Addr: Flash base address
*    Freq: Clock frequency in Hz
*    Func: Specifies the action followed by Init() (e.g.: 1 - Erase, 2 - Program, 3 - Verify / Read)
*
*  Return value 
*    0 O.K.
*    1 Error
*/
int Init(U32 Addr, U32 Freq, U32 Func) {
  (void)Addr;
  (void)Freq;
  (void)Func;
  //
  // Init code
  //
  *(volatile U32*)(0x000004) = Func; // Dummy code, needs to be replaced with Init code.
  return 0;
}

/*********************************************************************
*
*       UnInit
*
*  Function description
*    Handles the de-initialization of the flash module.
*
*  Parameters
*    Func: Caller type (e.g.: 1 - Erase, 2 - Program, 3 - Verify)
*
*  Return value 
*    0 O.K.
*    1 Error
*/
int UnInit(U32 Func) {
  (void)Func;
  //
  // Uninit code
  //
  *(volatile U32*)(0x000008) = Func; // Dummy code, needs to be replaced with UnInit code.
  return 0;
}

/*********************************************************************
*
*       EraseSector
*
*  Function description
*    Erases one flash sector.
*
*  Parameters
*    SectorAddr: Absolute address of the sector to be erased
*
*  Return value 
*    0 O.K.
*    1 Error
*/
int EraseSector(U32 SectorAddr) {
  //
  // Erase sector code
  //
  *(volatile U32*)(0x00000C) = SectorAddr;  // Dummy code, needs to be replaced with erase sector code
  _FeedWatchdog();
  return 0;
}

/*********************************************************************
*
*       ProgramPage
*
*  Function description
*    Programs one flash page.
*
*  Parameters
*    DestAddr: Destination address
*    NumBytes: Number of bytes to be programmed (always a multiple of program page size, defined in FlashDev.c)
*    pSrcBuff: Point to the source buffer
*
*  Return value 
*    0 O.K.
*    1 Error
*/
int ProgramPage(U32 DestAddr, U32 NumBytes, U8 *pSrcBuff) {
  volatile U8 * pSrc;
  volatile U8 * pDest;
  U8 AccessWidth;
  U32 Status;
  U32 NumPages;
  U32 NumBytesAtOnce;
  int r;

  r           = -1;
  pSrc        = (volatile U8*)pSrcBuff;
  pDest       = (volatile U8*)DestAddr;
  //
  // RAMCode is able to program multiple pages
  //
  NumPages    = NumBytes >> PAGE_SIZE_SHIFT;
  //
  // Program page-wise
  //
  if (NumPages) {
    r = 0;
    do {
      NumBytesAtOnce = (1 << PAGE_SIZE_SHIFT);
      _FeedWatchdog();
      //
      // Program one page
      //
      do {
        // 
        // Program page code
        //
        *pDest++ = *pSrc++;
      } while(--NumBytesAtOnce);
    } while (--NumPages);
  }
  return r;
}

/*********************************************************************
*
*       Verify
*
*  Function description
*    Compares a specified number of bytes of a provided data
*    buffer with the content of the device
*
*  Parameters
*    Addr: Start address in memory which should be compared
*    NumBytes: Number of bytes to be compared
*    pBuff: Pointer to the data to be compared
*
*  Return value 
*    == (Addr + NumBytes): O.K.
*    != (Addr + NumBytes): *not* O.K. (ideally the fail address is returned)
*
*/
#if SUPPORT_NATIVE_VERIFY
U32 Verify(U32 Addr, U32 NumBytes, U8 *pBuff) {
  unsigned char *pFlash;
  unsigned long r;

  pFlash = (unsigned char *)Addr;
  r = Addr + NumBytes;
  do {
      if (*pFlash != *pBuff) {
        r = (unsigned long)pFlash;
        break;
      }
      pFlash++;
      pBuff++;
  } while (--NumBytes);
  return r;
}
#endif

/*********************************************************************
*
*       BlankCheck
*
*  Function description
*    Checks if a memory region is blank
*
*  Parameters
*    Addr: Blank check start address
*    NumBytes: Number of bytes to be checked
*    BlankData: Pointer to the destination data
*
*  Return value 
*    0: O.K., blank
*    1: O.K., *not* blank
*    < 0: Error
*
*/
#if SUPPORT_BLANK_CHECK
int BlankCheck(U32 Addr, U32 NumBytes, U8 BlankData) {
  U8* pData;
  
  pData = (U8*)Addr;
  do {
    if (*pData++ != BlankData) {
      return 1;
    }
  } while (--NumBytes);
  return 0;
}
#endif

/*********************************************************************
*
*       EraseChip
*
*  Function description
*    Erases the entire flash
*
*  Return value 
*    0: O.K.
*    1: Error
*/
#if SUPPORT_ERASE_CHIP
int EraseChip(void) {
  //
  // Erase chip code
  //
  *(volatile U32*)(0x000000) = SectorAddr;  // Dummy code, needs to be replaced with erase chip code
  _FeedWatchdog();
  return 0;
}
#endif

/*********************************************************************
*
*       SEGGER_OPEN_Read
*
*  Function description
*    Reads a specified number of bytes into the provided buffer
*
*  Parameters
*    Addr: Start read address
*    NumBytes: Number of bytes to be read
*    pBuff: Pointer to the destination data
*
*  Return value 
*    >= 0: O.K., NumBytes read
*    <  0: Error
*
*/
#if SUPPORT_NATIVE_READ_FUNCTION
int SEGGER_OPEN_Read(U32 Addr, U32 NumBytes, U8 *pDestBuff) {
  //
  // Read function
  // Add your code here...
  //
  return NumBytes;
}
#endif

/*********************************************************************
*
*       SEGGER_OPEN_Program
*
*  Function description
*    Programs a specified number of bytes into the target flash.
*    NumBytes is either FlashDevice.PageSize or a multiple of it.
*
*  Notes
*    (1) This function can rely on that at least FlashDevice.PageSize will be passed
*    (2) This function must be able to handle multiple of FlashDevice.PageSize
*
*  Parameters
*    Addr: Start read address
*    NumBytes: Number of bytes to be read
*    pBuff: Pointer to the destination data
*
*  Return value 
*    0 O.K.
*    1 Error
*
*/
#if SUPPORT_SEGGER_OPEN_Program
int SEGGER_OPEN_Program(U32 DestAddr, U32 NumBytes, U8 *pSrcBuff) {
  U32 NumPages;
  int r;

  NumPages = (NumBytes >> PAGE_SIZE_SHIFT);
  r = 0;
  do {
    r = ProgramPage(DestAddr, (1uL << PAGE_SIZE_SHIFT), pSrcBuff);
    if (r < 0) {
      return r;
    }
    DestAddr += (1uL << PAGE_SIZE_SHIFT);
    pSrcBuff += (1uL << PAGE_SIZE_SHIFT);
  } while (--NumPages);
  return r;
}
#endif

/*********************************************************************
*
*       SEGGER_OPEN_Erase
*
*  Function description
*    Erases one or more flash sectors
*
*  Notes
*    (1) This function can rely on that at least one sector will be passed
*    (2) This function must be able to handle multiple sectors at once
*    (3) This function can rely on that only multiple sectors of the same sector
*        size will be passed. (e.g. if the device has two sectors with different
*        sizes, the DLL will call this function two times with NumSectors = 1)
*
*  Parameters
*    SectorAddr: Address of the start sector to be erased
*    SectorIndex: Index of the start sector to be erased
*    NumSectors: Number of sectors to be erased. At least 1 sector is passed.
*
*  Return value 
*    0 O.K.
*    1 Error
*
*/
#if SUPPORT_SEGGER_OPEN_ERASE
int SEGGER_OPEN_Erase(U32 SectorAddr, U32 SectorIndex, U32 NumSectors) {
  (void)SectorAddr;
  U32 Status;
  int r;
  
  _FeedWatchdog();
  r = 0;
  do {
    EraseSector(SectorAddr);
    SectorAddr += (1 << SECTOR_SIZE_SHIFT);
  } while (--NumSectors);
  return r;
}
#endif

/**************************** End of file ***************************/
