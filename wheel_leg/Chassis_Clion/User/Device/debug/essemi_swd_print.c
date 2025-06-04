/**
  *********************************************************************************
  *
  * @file    essemi_swd_print.c
  * @brief   Implementation of essemi swd-print which allows
  *          real-time communication on targets which support
  *          debugger memory accesses while the CPU is running.
  *
  * @version V1.0
  * @date    2021.03
  * @author  AE Team
  * @note
  *
  * Copyright (C) Shanghai Eastsoft Microelectronics Co. Ltd. All rights reserved.
  ******************************************************************************
  */
#include "essemi_swd_print.h"
#include <string.h>  // for memcpy

/*********************************************************************
*
*       Configuration, default values
*
**********************************************************************
*/

#ifndef   BUFFER_SIZE_UP
#define BUFFER_SIZE_UP                                  1024  // Size of the buffer for terminal output of target, up to host
#endif

#ifndef   BUFFER_SIZE_DOWN
#define BUFFER_SIZE_DOWN                                16    // Size of the buffer for terminal input to target from host (Usually keyboard input)
#endif

#ifndef   ESSEMI_SWD_MAX_NUM_UP_BUFFERS
#define ESSEMI_SWD_MAX_NUM_UP_BUFFERS                    2    // Number of up-buffers (T->H) available on this target
#endif

#ifndef   ESSEMI_SWD_MAX_NUM_DOWN_BUFFERS
#define ESSEMI_SWD_MAX_NUM_DOWN_BUFFERS                  2    // Number of down-buffers (H->T) available on this target
#endif

#ifndef ESSEMI_SWD_BUFFER_SECTION
#if defined(ESSEMI_SWD_SECTION)
#define ESSEMI_SWD_BUFFER_SECTION ESSEMI_SWD_SECTION
#endif
#endif

#ifndef   ESSEMI_SWD_ALIGNMENT
#define ESSEMI_SWD_ALIGNMENT                            0
#endif

#ifndef   ESSEMI_SWD_BUFFER_ALIGNMENT
#define ESSEMI_SWD_BUFFER_ALIGNMENT                     0
#endif

#ifndef   ESSEMI_SWD_MODE_DEFAULT
#define ESSEMI_SWD_MODE_DEFAULT                         ESSEMI_SWD_MODE_NO_BLOCK_SKIP
#endif

#ifndef   ESSEMI_SWD_LOCK
#define ESSEMI_SWD_LOCK()
#endif

#ifndef   ESSEMI_SWD_UNLOCK
#define ESSEMI_SWD_UNLOCK()
#endif

#ifndef   STRLEN
#define STRLEN(a)                                       strlen((a))
#endif

#ifndef   STRCPY
#define STRCPY(pDest, pSrc, NumBytes)                   strcpy((pDest), (pSrc))
#endif

#ifndef   ESSEMI_SWD_MEMCPY_USE_BYTELOOP
#define ESSEMI_SWD_MEMCPY_USE_BYTELOOP                  0
#endif

#ifndef   ESSEMI_SWD_MEMCPY
#ifdef  MEMCPY
#define ESSEMI_SWD_MEMCPY(pDest, pSrc, NumBytes)      MEMCPY((pDest), (pSrc), (NumBytes))
#else
#define ESSEMI_SWD_MEMCPY(pDest, pSrc, NumBytes)      memcpy((pDest), (pSrc), (NumBytes))
#endif
#endif

#ifndef   MIN
#define MIN(a, b)         (((a) < (b)) ? (a) : (b))
#endif

#ifndef   MAX
#define MAX(a, b)         (((a) > (b)) ? (a) : (b))
#endif
//
// For some environments, NULL may not be defined until certain headers are included
//
#ifndef NULL
#define NULL 0
#endif

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#if (defined __ICCARM__) || (defined __ICCRX__)
#define SWD_PRAGMA(P) _Pragma(#P)
#endif

#if ESSEMI_SWD_ALIGNMENT || ESSEMI_SWD_BUFFER_ALIGNMENT
#if (defined __GNUC__)
#define ESSEMI_SWD_ALIGN(Var, Alignment) Var __attribute__ ((aligned (Alignment)))
#elif (defined __ICCARM__) || (defined __ICCRX__)
#define PRAGMA(A) _Pragma(#A)
#define ESSEMI_SWD_ALIGN(Var, Alignment) SWD_PRAGMA(data_alignment=Alignment) \
    Var
#elif (defined __CC_ARM)
#define ESSEMI_SWD_ALIGN(Var, Alignment) Var __attribute__ ((aligned (Alignment)))
#else
#error "Alignment not supported for this compiler."
#endif
#else
#define ESSEMI_SWD_ALIGN(Var, Alignment) Var
#endif

#if defined(ESSEMI_SWD_SECTION) || defined (ESSEMI_SWD_BUFFER_SECTION)
#if (defined __GNUC__)
#define ESSEMI_SWD_PUT_SECTION(Var, Section) __attribute__ ((section (Section))) Var
#elif (defined __ICCARM__) || (defined __ICCRX__)
#define ESSEMI_SWD_PUT_SECTION(Var, Section) SWD_PRAGMA(location=Section) \
    Var
#elif (defined __CC_ARM)
#define ESSEMI_SWD_PUT_SECTION(Var, Section) __attribute__ ((section (Section), zero_init))  Var
#else
#error "Section placement not supported for this compiler."
#endif
#else
#define ESSEMI_SWD_PUT_SECTION(Var, Section) Var
#endif


#if ESSEMI_SWD_ALIGNMENT
#define ESSEMI_SWD_CB_ALIGN(Var)  ESSEMI_SWD_ALIGN(Var, ESSEMI_SWD_ALIGNMENT)
#else
#define ESSEMI_SWD_CB_ALIGN(Var)  Var
#endif

#if ESSEMI_SWD_BUFFER_ALIGNMENT
#define ESSEMI_SWD_BUFFER_ALIGN(Var)  ESSEMI_SWD_ALIGN(Var, ESSEMI_SWD_BUFFER_ALIGNMENT)
#else
#define ESSEMI_SWD_BUFFER_ALIGN(Var)  Var
#endif


#if defined(ESSEMI_SWD_SECTION)
#define ESSEMI_SWD_PUT_CB_SECTION(Var) ESSEMI_SWD_PUT_SECTION(Var, ESSEMI_SWD_SECTION)
#else
#define ESSEMI_SWD_PUT_CB_SECTION(Var) Var
#endif

#if defined(ESSEMI_SWD_BUFFER_SECTION)
#define ESSEMI_SWD_PUT_BUFFER_SECTION(Var) ESSEMI_SWD_PUT_SECTION(Var, ESSEMI_SWD_BUFFER_SECTION)
#else
#define ESSEMI_SWD_PUT_BUFFER_SECTION(Var) Var
#endif

/*********************************************************************
*
*       Static const data
*
**********************************************************************
*/

static unsigned char _aTerminalId[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
//
// SWD Control Block and allocate buffers for channel 0
//
ESSEMI_SWD_PUT_CB_SECTION(ESSEMI_SWD_CB_ALIGN(ESSEMI_SWD_CB _ESSEMI_SWD));

ESSEMI_SWD_PUT_BUFFER_SECTION(ESSEMI_SWD_BUFFER_ALIGN(static char _acUpBuffer  [BUFFER_SIZE_UP]));
ESSEMI_SWD_PUT_BUFFER_SECTION(ESSEMI_SWD_BUFFER_ALIGN(static char _acDownBuffer[BUFFER_SIZE_DOWN]));

static unsigned char _ActiveTerminal;

/*********************************************************************
*
*       Static functions
*
**********************************************************************
*/

/*********************************************************************
*
*       _DoInit()
*
*  Function description
*    Initializes the control block an buffers.
*    May only be called via INIT() to avoid overriding settings.
*
*/
#define INIT()  do {                                            \
        if (_ESSEMI_SWD.acID[0] == '\0') { _DoInit(); }  \
    } while (0)
static void _DoInit(void)
{
    ESSEMI_SWD_CB *p;
    //
    // Initialize control block
    //
    p = &_ESSEMI_SWD;
    p->MaxNumUpBuffers    = ESSEMI_SWD_MAX_NUM_UP_BUFFERS;
    p->MaxNumDownBuffers  = ESSEMI_SWD_MAX_NUM_DOWN_BUFFERS;
    //
    // Initialize up buffer 0
    //
    p->aUp[0].sName         = "Terminal";
    p->aUp[0].pBuffer       = _acUpBuffer;
    p->aUp[0].SizeOfBuffer  = sizeof(_acUpBuffer);
    p->aUp[0].RdOff         = 0u;
    p->aUp[0].WrOff         = 0u;
    p->aUp[0].Flags         = ESSEMI_SWD_MODE_DEFAULT;
    //
    // Initialize down buffer 0
    //
    p->aDown[0].sName         = "Terminal";
    p->aDown[0].pBuffer       = _acDownBuffer;
    p->aDown[0].SizeOfBuffer  = sizeof(_acDownBuffer);
    p->aDown[0].RdOff         = 0u;
    p->aDown[0].WrOff         = 0u;
    p->aDown[0].Flags         = ESSEMI_SWD_MODE_DEFAULT;
    //
    // Finish initialization of the control block.
    // Copy Id string in three steps to make sure "ESSEMI SWD" is not found
    // in initializer memory (usually flash) by J-Link
    //
    STRCPY(&p->acID[7], "SWD", 9);
    SWD__DMB();
    STRCPY(&p->acID[0], "ESSEMI", 7);
    SWD__DMB();
    p->acID[6] = ' ';
    SWD__DMB();
}

/*********************************************************************
*
*       _WriteBlocking()
*
*  Function description
*    Stores a specified number of characters in ESSEMI SWD ring buffer
*    and updates the associated write pointer which is periodically
*    read by the host.
*    The caller is responsible for managing the write chunk sizes as
*    _WriteBlocking() will block until all data has been posted successfully.
*
*  Parameters
*    pRing        Ring buffer to post to.
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the ESSEMI SWD control block.
*
*  Return value
*    >= 0 - Number of bytes written into buffer.
*/
static unsigned _WriteBlocking(ESSEMI_SWD_BUFFER_UP *pRing, const char *pBuffer, unsigned NumBytes)
{
    unsigned NumBytesToWrite;
    unsigned NumBytesWritten;
    unsigned RdOff;
    unsigned WrOff;
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
    char    *pDst;
#endif
    //
    // Write data to buffer and handle wrap-around if necessary
    //
    NumBytesWritten = 0u;
    WrOff = pRing->WrOff;

    do
    {
        RdOff = pRing->RdOff;                         // May be changed by host (debug probe) in the meantime

        if (RdOff > WrOff)
        {
            NumBytesToWrite = RdOff - WrOff - 1u;
        }
        else
        {
            NumBytesToWrite = pRing->SizeOfBuffer - (WrOff - RdOff + 1u);
        }

        NumBytesToWrite = MIN(NumBytesToWrite, (pRing->SizeOfBuffer - WrOff));      // Number of bytes that can be written until buffer wrap-around
        NumBytesToWrite = MIN(NumBytesToWrite, NumBytes);
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
        pDst = pRing->pBuffer + WrOff;
        NumBytesWritten += NumBytesToWrite;
        NumBytes        -= NumBytesToWrite;
        WrOff           += NumBytesToWrite;

        while (NumBytesToWrite--)
        {
            *pDst++ = *pBuffer++;
        };

#else
        ESSEMI_SWD_MEMCPY(pRing->pBuffer + WrOff, pBuffer, NumBytesToWrite);

        NumBytesWritten += NumBytesToWrite;

        pBuffer         += NumBytesToWrite;

        NumBytes        -= NumBytesToWrite;

        WrOff           += NumBytesToWrite;

#endif
        if (WrOff == pRing->SizeOfBuffer)
        {
            WrOff = 0u;
        }

        SWD__DMB();
        pRing->WrOff = WrOff;
    }
    while (NumBytes);

    //
    return NumBytesWritten;
}

/*********************************************************************
*
*       _WriteNoCheck()
*
*  Function description
*    Stores a specified number of characters in ESSEMI SWD ring buffer
*    and updates the associated write pointer which is periodically
*    read by the host.
*    It is callers responsibility to make sure data actually fits in buffer.
*
*  Parameters
*    pRing        Ring buffer to post to.
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the ESSEMI SWD control block.
*
*  Notes
*    (1) If there might not be enough space in the "Up"-buffer, call _WriteBlocking
*/
static void _WriteNoCheck(ESSEMI_SWD_BUFFER_UP *pRing, const char *pData, unsigned NumBytes)
{
    unsigned NumBytesAtOnce;
    unsigned WrOff;
    unsigned Rem;
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
    char    *pDst;
#endif

    WrOff = pRing->WrOff;
    Rem = pRing->SizeOfBuffer - WrOff;

    if (Rem > NumBytes)
    {
        //
        // All data fits before wrap around
        //
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
        pDst = pRing->pBuffer + WrOff;
        WrOff += NumBytes;

        while (NumBytes--)
        {
            *pDst++ = *pData++;
        };

        SWD__DMB();

        pRing->WrOff = WrOff;

#else
        ESSEMI_SWD_MEMCPY(pRing->pBuffer + WrOff, pData, NumBytes);

        SWD__DMB();

        pRing->WrOff = WrOff + NumBytes;

#endif
    }
    else
    {
        //
        // We reach the end of the buffer, so need to wrap around
        //
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
        pDst = pRing->pBuffer + WrOff;
        NumBytesAtOnce = Rem;

        while (NumBytesAtOnce--)
        {
            *pDst++ = *pData++;
        };

        pDst = pRing->pBuffer;

        NumBytesAtOnce = NumBytes - Rem;

        while (NumBytesAtOnce--)
        {
            *pDst++ = *pData++;
        };

        SWD__DMB();

        pRing->WrOff = NumBytes - Rem;

#else
        NumBytesAtOnce = Rem;

        ESSEMI_SWD_MEMCPY(pRing->pBuffer + WrOff, pData, NumBytesAtOnce);

        NumBytesAtOnce = NumBytes - Rem;

        ESSEMI_SWD_MEMCPY(pRing->pBuffer, pData + Rem, NumBytesAtOnce);

        SWD__DMB();

        pRing->WrOff = NumBytesAtOnce;

#endif
    }
}

/*********************************************************************
*
*       _PostTerminalSwitch()
*
*  Function description
*    Switch terminal to the given terminal ID.  It is the caller's
*    responsibility to ensure the terminal ID is correct and there is
*    enough space in the buffer for this to complete successfully.
*
*  Parameters
*    pRing        Ring buffer to post to.
*    TerminalId   Terminal ID to switch to.
*/
static void _PostTerminalSwitch(ESSEMI_SWD_BUFFER_UP *pRing, unsigned char TerminalId)
{
    unsigned char ac[2];

    ac[0] = 0xFFu;
    ac[1] = _aTerminalId[TerminalId];  // Caller made already sure that TerminalId does not exceed our terminal limit
    _WriteBlocking(pRing, (const char *)ac, 2u);
}

/*********************************************************************
*
*       _GetAvailWriteSpace()
*
*  Function description
*    Returns the number of bytes that can be written to the ring
*    buffer without blocking.
*
*  Parameters
*    pRing        Ring buffer to check.
*
*  Return value
*    Number of bytes that are free in the buffer.
*/
static unsigned _GetAvailWriteSpace(ESSEMI_SWD_BUFFER_UP *pRing)
{
    unsigned RdOff;
    unsigned WrOff;
    unsigned r;
    //
    // Avoid warnings regarding volatile access order.  It's not a problem
    // in this case, but dampen compiler enthusiasm.
    //
    RdOff = pRing->RdOff;
    WrOff = pRing->WrOff;

    if (RdOff <= WrOff)
    {
        r = pRing->SizeOfBuffer - 1u - WrOff + RdOff;
    }
    else
    {
        r = RdOff - WrOff - 1u;
    }

    return r;
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       ESSEMI_SWD_ReadUpBufferNoLock()
*
*  Function description
*    Reads characters from ESSEMI real-time-terminal control block
*    which have been previously stored by the application.
*    Do not lock against interrupts and multiple access.
*    Used to do the same operation that J-Link does, to transfer
*    SWD data via other channels, such as TCP/IP or UART.
*
*  Parameters
*    BufferIndex  Index of Up-buffer to be used.
*    pBuffer      Pointer to buffer provided by target application, to copy characters from SWD-up-buffer to.
*    BufferSize   Size of the target application buffer.
*
*  Return value
*    Number of bytes that have been read.
*
*  Additional information
*    This function must not be called when J-Link might also do SWD.
*/
unsigned ESSEMI_SWD_ReadUpBufferNoLock(unsigned BufferIndex, void *pData, unsigned BufferSize)
{
    unsigned                NumBytesRem;
    unsigned                NumBytesRead;
    unsigned                RdOff;
    unsigned                WrOff;
    unsigned char          *pBuffer;
    ESSEMI_SWD_BUFFER_UP   *pRing;
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
    const char             *pSrc;
#endif
    //
    INIT();
    pRing = &_ESSEMI_SWD.aUp[BufferIndex];
    pBuffer = (unsigned char *)pData;
    RdOff = pRing->RdOff;
    WrOff = pRing->WrOff;
    NumBytesRead = 0u;

    //
    // Read from current read position to wrap-around of buffer, first
    //
    if (RdOff > WrOff)
    {
        NumBytesRem = pRing->SizeOfBuffer - RdOff;
        NumBytesRem = MIN(NumBytesRem, BufferSize);
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
        pSrc = pRing->pBuffer + RdOff;
        NumBytesRead += NumBytesRem;
        BufferSize   -= NumBytesRem;
        RdOff        += NumBytesRem;

        while (NumBytesRem--)
        {
            *pBuffer++ = *pSrc++;
        };

#else
        ESSEMI_SWD_MEMCPY(pBuffer, pRing->pBuffer + RdOff, NumBytesRem);

        NumBytesRead += NumBytesRem;

        pBuffer      += NumBytesRem;

        BufferSize   -= NumBytesRem;

        RdOff        += NumBytesRem;

#endif

        //
        // Handle wrap-around of buffer
        //
        if (RdOff == pRing->SizeOfBuffer)
        {
            RdOff = 0u;
        }
    }

    //
    // Read remaining items of buffer
    //
    NumBytesRem = WrOff - RdOff;
    NumBytesRem = MIN(NumBytesRem, BufferSize);

    if (NumBytesRem > 0u)
    {
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
        pSrc = pRing->pBuffer + RdOff;
        NumBytesRead += NumBytesRem;
        BufferSize   -= NumBytesRem;
        RdOff        += NumBytesRem;

        while (NumBytesRem--)
        {
            *pBuffer++ = *pSrc++;
        };

#else
        ESSEMI_SWD_MEMCPY(pBuffer, pRing->pBuffer + RdOff, NumBytesRem);

        NumBytesRead += NumBytesRem;

        pBuffer      += NumBytesRem;

        BufferSize   -= NumBytesRem;

        RdOff        += NumBytesRem;

#endif
    }

    //
    // Update read offset of buffer
    //
    if (NumBytesRead)
    {
        pRing->RdOff = RdOff;
    }

    //
    return NumBytesRead;
}

/*********************************************************************
*
*       ESSEMI_SWD_ReadNoLock()
*
*  Function description
*    Reads characters from ESSEMI real-time-terminal control block
*    which have been previously stored by the host.
*    Do not lock against interrupts and multiple access.
*
*  Parameters
*    BufferIndex  Index of Down-buffer to be used (e.g. 0 for "Terminal").
*    pBuffer      Pointer to buffer provided by target application, to copy characters from SWD-down-buffer to.
*    BufferSize   Size of the target application buffer.
*
*  Return value
*    Number of bytes that have been read.
*/
unsigned ESSEMI_SWD_ReadNoLock(unsigned BufferIndex, void *pData, unsigned BufferSize)
{
    unsigned                NumBytesRem;
    unsigned                NumBytesRead;
    unsigned                RdOff;
    unsigned                WrOff;
    unsigned char          *pBuffer;
    ESSEMI_SWD_BUFFER_DOWN *pRing;
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
    const char             *pSrc;
#endif
    //
    INIT();
    pRing = &_ESSEMI_SWD.aDown[BufferIndex];
    pBuffer = (unsigned char *)pData;
    RdOff = pRing->RdOff;
    WrOff = pRing->WrOff;
    NumBytesRead = 0u;

    //
    // Read from current read position to wrap-around of buffer, first
    //
    if (RdOff > WrOff)
    {
        NumBytesRem = pRing->SizeOfBuffer - RdOff;
        NumBytesRem = MIN(NumBytesRem, BufferSize);
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
        pSrc = pRing->pBuffer + RdOff;
        NumBytesRead += NumBytesRem;
        BufferSize   -= NumBytesRem;
        RdOff        += NumBytesRem;

        while (NumBytesRem--)
        {
            *pBuffer++ = *pSrc++;
        };

#else
        ESSEMI_SWD_MEMCPY(pBuffer, pRing->pBuffer + RdOff, NumBytesRem);

        NumBytesRead += NumBytesRem;

        pBuffer      += NumBytesRem;

        BufferSize   -= NumBytesRem;

        RdOff        += NumBytesRem;

#endif

        //
        // Handle wrap-around of buffer
        //
        if (RdOff == pRing->SizeOfBuffer)
        {
            RdOff = 0u;
        }
    }

    //
    // Read remaining items of buffer
    //
    NumBytesRem = WrOff - RdOff;
    NumBytesRem = MIN(NumBytesRem, BufferSize);

    if (NumBytesRem > 0u)
    {
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
        pSrc = pRing->pBuffer + RdOff;
        NumBytesRead += NumBytesRem;
        BufferSize   -= NumBytesRem;
        RdOff        += NumBytesRem;

        while (NumBytesRem--)
        {
            *pBuffer++ = *pSrc++;
        };

#else
        ESSEMI_SWD_MEMCPY(pBuffer, pRing->pBuffer + RdOff, NumBytesRem);

        NumBytesRead += NumBytesRem;

        pBuffer      += NumBytesRem;

        BufferSize   -= NumBytesRem;

        RdOff        += NumBytesRem;

#endif
    }

    if (NumBytesRead)
    {
        pRing->RdOff = RdOff;
    }

    //
    return NumBytesRead;
}

/*********************************************************************
*
*       ESSEMI_SWD_ReadUpBuffer
*
*  Function description
*    Reads characters from ESSEMI real-time-terminal control block
*    which have been previously stored by the application.
*    Used to do the same operation that J-Link does, to transfer
*    SWD data via other channels, such as TCP/IP or UART.
*
*  Parameters
*    BufferIndex  Index of Up-buffer to be used.
*    pBuffer      Pointer to buffer provided by target application, to copy characters from SWD-up-buffer to.
*    BufferSize   Size of the target application buffer.
*
*  Return value
*    Number of bytes that have been read.
*
*  Additional information
*    This function must not be called when J-Link might also do SWD.
*    This function locks against all other SWD operations. I.e. during
*    the read operation, writing is also locked.
*    If only one consumer reads from the up buffer,
*    call ESSEMI_SWD_ReadUpBufferNoLock() instead.
*/
unsigned ESSEMI_SWD_ReadUpBuffer(unsigned BufferIndex, void *pBuffer, unsigned BufferSize)
{
    unsigned NumBytesRead;
    //
    ESSEMI_SWD_LOCK();
    //
    // Call the non-locking read function
    //
    NumBytesRead = ESSEMI_SWD_ReadUpBufferNoLock(BufferIndex, pBuffer, BufferSize);
    //
    // Finish up.
    //
    ESSEMI_SWD_UNLOCK();
    //
    return NumBytesRead;
}

/*********************************************************************
*
*       ESSEMI_SWD_Read
*
*  Function description
*    Reads characters from ESSEMI real-time-terminal control block
*    which have been previously stored by the host.
*
*  Parameters
*    BufferIndex  Index of Down-buffer to be used (e.g. 0 for "Terminal").
*    pBuffer      Pointer to buffer provided by target application, to copy characters from SWD-down-buffer to.
*    BufferSize   Size of the target application buffer.
*
*  Return value
*    Number of bytes that have been read.
*/
unsigned ESSEMI_SWD_Read(unsigned BufferIndex, void *pBuffer, unsigned BufferSize)
{
    unsigned NumBytesRead;
    //
    ESSEMI_SWD_LOCK();
    //
    // Call the non-locking read function
    //
    NumBytesRead = ESSEMI_SWD_ReadNoLock(BufferIndex, pBuffer, BufferSize);
    //
    // Finish up.
    //
    ESSEMI_SWD_UNLOCK();
    //
    return NumBytesRead;
}

/*********************************************************************
*
*       ESSEMI_SWD_WriteWithOverwriteNoLock
*
*  Function description
*    Stores a specified number of characters in ESSEMI SWD
*    control block.
*    ESSEMI_SWD_WriteWithOverwriteNoLock does not lock the application
*    and overwrites data if the data does not fit into the buffer.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used (e.g. 0 for "Terminal").
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the ESSEMI SWD control block.
*
*  Notes
*    (1) If there is not enough space in the "Up"-buffer, data is overwritten.
*    (2) For performance reasons this function does not call Init()
*        and may only be called after SWD has been initialized.
*        Either by calling ESSEMI_SWD_Init() or calling another SWD API function first.
*    (3) Do not use ESSEMI_SWD_WriteWithOverwriteNoLock if a J-Link
*        connection reads SWD data.
*/
void ESSEMI_SWD_WriteWithOverwriteNoLock(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes)
{
    const char           *pData;
    ESSEMI_SWD_BUFFER_UP *pRing;
    unsigned              Avail;
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
    char                 *pDst;
#endif

    pData = (const char *)pBuffer;
    //
    // Get "to-host" ring buffer and copy some elements into local variables.
    //
    pRing = &_ESSEMI_SWD.aUp[BufferIndex];

    //
    // Check if we will overwrite data and need to adjust the RdOff.
    //
    if (pRing->WrOff == pRing->RdOff)
    {
        Avail = pRing->SizeOfBuffer - 1u;
    }
    else if (pRing->WrOff < pRing->RdOff)
    {
        Avail = pRing->RdOff - pRing->WrOff - 1u;
    }
    else
    {
        Avail = pRing->RdOff - pRing->WrOff - 1u + pRing->SizeOfBuffer;
    }

    if (NumBytes > Avail)
    {
        pRing->RdOff += (NumBytes - Avail);

        while (pRing->RdOff >= pRing->SizeOfBuffer)
        {
            pRing->RdOff -= pRing->SizeOfBuffer;
        }
    }

    //
    // Write all data, no need to check the RdOff, but possibly handle multiple wrap-arounds
    //
    Avail = pRing->SizeOfBuffer - pRing->WrOff;

    do
    {
        if (Avail > NumBytes)
        {
            //
            // Last round
            //
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
            pDst = pRing->pBuffer + pRing->WrOff;
            Avail = NumBytes;

            while (NumBytes--)
            {
                *pDst++ = *pData++;
            };

            SWD__DMB();

            pRing->WrOff += Avail;

#else
            ESSEMI_SWD_MEMCPY(pRing->pBuffer + pRing->WrOff, pData, NumBytes);

            SWD__DMB();

            pRing->WrOff += NumBytes;

#endif
            break;
        }
        else
        {
            //
            //  Wrap-around necessary, write until wrap-around and reset WrOff
            //
#if ESSEMI_SWD_MEMCPY_USE_BYTELOOP
            pDst = pRing->pBuffer + pRing->WrOff;
            NumBytes -= Avail;

            while (Avail--)
            {
                *pDst++ = *pData++;
            };

            SWD__DMB();

            pRing->WrOff = 0;

#else
            ESSEMI_SWD_MEMCPY(pRing->pBuffer + pRing->WrOff, pData, Avail);

            pData += Avail;

            SWD__DMB();

            pRing->WrOff = 0;

            NumBytes -= Avail;

#endif
            Avail = (pRing->SizeOfBuffer - 1);
        }
    }
    while (NumBytes);
}

/*********************************************************************
*
*       ESSEMI_SWD_WriteSkipNoLock
*
*  Function description
*    Stores a specified number of characters in ESSEMI SWD
*    control block which is then read by the host.
*    ESSEMI_SWD_WriteSkipNoLock does not lock the application and
*    skips all data, if the data does not fit into the buffer.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used (e.g. 0 for "Terminal").
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the ESSEMI SWD control block.
*                 MUST be > 0!!!
*                 This is done for performance reasons, so no initial check has do be done.
*
*  Return value
*    1: Data has been copied
*    0: No space, data has not been copied
*
*  Notes
*    (1) If there is not enough space in the "Up"-buffer, all data is dropped.
*    (2) For performance reasons this function does not call Init()
*        and may only be called after SWD has been initialized.
*        Either by calling ESSEMI_SWD_Init() or calling another SWD API function first.
*/
#if (SWD_USE_ASM == 0)
unsigned ESSEMI_SWD_WriteSkipNoLock(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes)
{
    const char           *pData;
    ESSEMI_SWD_BUFFER_UP *pRing;
    unsigned              Avail;
    unsigned              RdOff;
    unsigned              WrOff;
    unsigned              Rem;
    //
    // Cases:
    //   1) RdOff <= WrOff => Space until wrap-around is sufficient
    //   2) RdOff <= WrOff => Space after wrap-around needed (copy in 2 chunks)
    //   3) RdOff <  WrOff => No space in buf
    //   4) RdOff >  WrOff => Space is sufficient
    //   5) RdOff >  WrOff => No space in buf
    //
    // 1) is the most common case for large buffers and assuming that J-Link reads the data fast enough
    //
    pData = (const char *)pBuffer;
    pRing = &_ESSEMI_SWD.aUp[BufferIndex];
    RdOff = pRing->RdOff;
    WrOff = pRing->WrOff;

    if (RdOff <= WrOff)                                   // Case 1), 2) or 3)
    {
        Avail = pRing->SizeOfBuffer - WrOff - 1u;           // Space until wrap-around (assume 1 byte not usable for case that RdOff == 0)

        if (Avail >= NumBytes)                              // Case 1)?
        {
            CopyStraight:
            memcpy(pRing->pBuffer + WrOff, pData, NumBytes);
            SWD__DMB();
            pRing->WrOff = WrOff + NumBytes;
            return 1;
        }

        Avail += RdOff;                                     // Space incl. wrap-around

        if (Avail >= NumBytes)                              // Case 2? => If not, we have case 3) (does not fit)
        {
            Rem = pRing->SizeOfBuffer - WrOff;                // Space until end of buffer
            memcpy(pRing->pBuffer + WrOff, pData, Rem);       // Copy 1st chunk
            NumBytes -= Rem;

            //
            // Special case: First check that assumed RdOff == 0 calculated that last element before wrap-around could not be used
            // But 2nd check (considering space until wrap-around and until RdOff) revealed that RdOff is not 0, so we can use the last element
            // In this case, we may use a copy straight until buffer end anyway without needing to copy 2 chunks
            // Therefore, check if 2nd memcpy is necessary at all
            //
            if (NumBytes)
            {
                memcpy(pRing->pBuffer, pData + Rem, NumBytes);
            }

            SWD__DMB();
            pRing->WrOff = NumBytes;
            return 1;
        }
    }
    else                                                 // Potential case 4)
    {
        Avail = RdOff - WrOff - 1u;

        if (Avail >= NumBytes)                             // Case 4)? => If not, we have case 5) (does not fit)
        {
            goto CopyStraight;
        }
    }

    return 0;     // No space in buffer
}
#endif

/*********************************************************************
*
*       ESSEMI_SWD_WriteDownBufferNoLock
*
*  Function description
*    Stores a specified number of characters in ESSEMI SWD
*    control block inside a <Down> buffer.
*    ESSEMI_SWD_WriteDownBufferNoLock does not lock the application.
*    Used to do the same operation that J-Link does, to transfer
*    SWD data from other channels, such as TCP/IP or UART.
*
*  Parameters
*    BufferIndex  Index of "Down"-buffer to be used.
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the ESSEMI SWD control block.
*
*  Return value
*    Number of bytes which have been stored in the "Down"-buffer.
*
*  Notes
*    (1) Data is stored according to buffer flags.
*    (2) For performance reasons this function does not call Init()
*        and may only be called after SWD has been initialized.
*        Either by calling ESSEMI_SWD_Init() or calling another SWD API function first.
*
*  Additional information
*    This function must not be called when J-Link might also do SWD.
*/
unsigned ESSEMI_SWD_WriteDownBufferNoLock(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes)
{
    unsigned                Status;
    unsigned                Avail;
    const char             *pData;
    ESSEMI_SWD_BUFFER_UP   *pRing;

    pData = (const char *)pBuffer;
    //
    // Get "to-target" ring buffer.
    // It is save to cast that to a "to-host" buffer. Up and Down buffer differ in volatility of offsets that might be modified by J-Link.
    //
    pRing = (ESSEMI_SWD_BUFFER_UP *)&_ESSEMI_SWD.aDown[BufferIndex];

    //
    // How we output depends upon the mode...
    //
    switch (pRing->Flags)
    {
        case ESSEMI_SWD_MODE_NO_BLOCK_SKIP:
            //
            // If we are in skip mode and there is no space for the whole
            // of this output, don't bother.
            //
            Avail = _GetAvailWriteSpace(pRing);

            if (Avail < NumBytes)
            {
                Status = 0u;
            }
            else
            {
                Status = NumBytes;
                _WriteNoCheck(pRing, pData, NumBytes);
            }

            break;

        case ESSEMI_SWD_MODE_NO_BLOCK_TRIM:
            //
            // If we are in trim mode, trim to what we can output without blocking.
            //
            Avail = _GetAvailWriteSpace(pRing);
            Status = Avail < NumBytes ? Avail : NumBytes;
            _WriteNoCheck(pRing, pData, Status);
            break;

        case ESSEMI_SWD_MODE_BLOCK_IF_FIFO_FULL:
            //
            // If we are in blocking mode, output everything.
            //
            Status = _WriteBlocking(pRing, pData, NumBytes);
            break;

        default:
            Status = 0u;
            break;
    }

    //
    // Finish up.
    //
    return Status;
}

/*********************************************************************
*
*       ESSEMI_SWD_WriteNoLock
*
*  Function description
*    Stores a specified number of characters in ESSEMI SWD
*    control block which is then read by the host.
*    ESSEMI_SWD_WriteNoLock does not lock the application.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used (e.g. 0 for "Terminal").
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the ESSEMI SWD control block.
*
*  Return value
*    Number of bytes which have been stored in the "Up"-buffer.
*
*  Notes
*    (1) Data is stored according to buffer flags.
*    (2) For performance reasons this function does not call Init()
*        and may only be called after SWD has been initialized.
*        Either by calling ESSEMI_SWD_Init() or calling another SWD API function first.
*/
unsigned ESSEMI_SWD_WriteNoLock(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes)
{
    unsigned              Status;
    unsigned              Avail;
    const char           *pData;
    ESSEMI_SWD_BUFFER_UP *pRing;

    pData = (const char *)pBuffer;
    //
    // Get "to-host" ring buffer.
    //
    pRing = &_ESSEMI_SWD.aUp[BufferIndex];

    //
    // How we output depends upon the mode...
    //
    switch (pRing->Flags)
    {
        case ESSEMI_SWD_MODE_NO_BLOCK_SKIP:
            //
            // If we are in skip mode and there is no space for the whole
            // of this output, don't bother.
            //
            Avail = _GetAvailWriteSpace(pRing);

            if (Avail < NumBytes)
            {
                Status = 0u;
            }
            else
            {
                Status = NumBytes;
                _WriteNoCheck(pRing, pData, NumBytes);
            }

            break;

        case ESSEMI_SWD_MODE_NO_BLOCK_TRIM:
            //
            // If we are in trim mode, trim to what we can output without blocking.
            //
            Avail = _GetAvailWriteSpace(pRing);
            Status = Avail < NumBytes ? Avail : NumBytes;
            _WriteNoCheck(pRing, pData, Status);
            break;

        case ESSEMI_SWD_MODE_BLOCK_IF_FIFO_FULL:
            //
            // If we are in blocking mode, output everything.
            //
            Status = _WriteBlocking(pRing, pData, NumBytes);
            break;

        default:
            Status = 0u;
            break;
    }

    //
    // Finish up.
    //
    return Status;
}

/*********************************************************************
*
*       ESSEMI_SWD_WriteDownBuffer
*
*  Function description
*    Stores a specified number of characters in ESSEMI SWD control block in a <Down> buffer.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used (e.g. 0 for "Terminal").
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the ESSEMI SWD control block.
*
*  Return value
*    Number of bytes which have been stored in the "Down"-buffer.
*
*  Notes
*    (1) Data is stored according to buffer flags.
*
*  Additional information
*    This function must not be called when J-Link might also do SWD.
*    This function locks against all other SWD operations. I.e. during
*    the write operation, writing from the application is also locked.
*    If only one consumer writes to the down buffer,
*    call ESSEMI_SWD_WriteDownBufferNoLock() instead.
*/
unsigned ESSEMI_SWD_WriteDownBuffer(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes)
{
    unsigned Status;
    //
    INIT();
    ESSEMI_SWD_LOCK();
    //
    // Call the non-locking write function
    //
    Status = ESSEMI_SWD_WriteDownBufferNoLock(BufferIndex, pBuffer, NumBytes);
    //
    // Finish up.
    //
    ESSEMI_SWD_UNLOCK();
    //
    return Status;
}

/*********************************************************************
*
*       ESSEMI_SWD_Write
*
*  Function description
*    Stores a specified number of characters in ESSEMI SWD
*    control block which is then read by the host.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used (e.g. 0 for "Terminal").
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the ESSEMI SWD control block.
*
*  Return value
*    Number of bytes which have been stored in the "Up"-buffer.
*
*  Notes
*    (1) Data is stored according to buffer flags.
*/
unsigned ESSEMI_SWD_Write(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes)
{
    unsigned Status;
    //
    INIT();
    ESSEMI_SWD_LOCK();
    //
    // Call the non-locking write function
    //
    Status = ESSEMI_SWD_WriteNoLock(BufferIndex, pBuffer, NumBytes);
    //
    // Finish up.
    //
    ESSEMI_SWD_UNLOCK();
    //
    return Status;
}

/*********************************************************************
*
*       ESSEMI_SWD_WriteString
*
*  Function description
*    Stores string in ESSEMI SWD control block.
*    This data is read by the host.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used (e.g. 0 for "Terminal").
*    s            Pointer to string.
*
*  Return value
*    Number of bytes which have been stored in the "Up"-buffer.
*
*  Notes
*    (1) Data is stored according to buffer flags.
*    (2) String passed to this function has to be \0 terminated
*    (3) \0 termination character is *not* stored in SWD buffer
*/
unsigned ESSEMI_SWD_WriteString(unsigned BufferIndex, const char *s)
{
    unsigned Len;

    Len = STRLEN(s);
    return ESSEMI_SWD_Write(BufferIndex, s, Len);
}

/*********************************************************************
*
*       ESSEMI_SWD_PutCharSkipNoLock
*
*  Function description
*    Stores a single character/byte in ESSEMI SWD buffer.
*    ESSEMI_SWD_PutCharSkipNoLock does not lock the application and
*    skips the byte, if it does not fit into the buffer.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used (e.g. 0 for "Terminal").
*    c            Byte to be stored.
*
*  Return value
*    Number of bytes which have been stored in the "Up"-buffer.
*
*  Notes
*    (1) If there is not enough space in the "Up"-buffer, the character is dropped.
*    (2) For performance reasons this function does not call Init()
*        and may only be called after SWD has been initialized.
*        Either by calling ESSEMI_SWD_Init() or calling another SWD API function first.
*/

unsigned ESSEMI_SWD_PutCharSkipNoLock(unsigned BufferIndex, char c)
{
    ESSEMI_SWD_BUFFER_UP *pRing;
    unsigned              WrOff;
    unsigned              Status;
    //
    // Get "to-host" ring buffer.
    //
    pRing = &_ESSEMI_SWD.aUp[BufferIndex];
    //
    // Get write position and handle wrap-around if necessary
    //
    WrOff = pRing->WrOff + 1;

    if (WrOff == pRing->SizeOfBuffer)
    {
        WrOff = 0;
    }

    //
    // Output byte if free space is available
    //
    if (WrOff != pRing->RdOff)
    {
        pRing->pBuffer[pRing->WrOff] = c;
        SWD__DMB();
        pRing->WrOff = WrOff;
        Status = 1;
    }
    else
    {
        Status = 0;
    }

    //
    return Status;
}

/*********************************************************************
*
*       ESSEMI_SWD_PutCharSkip
*
*  Function description
*    Stores a single character/byte in ESSEMI SWD buffer.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used (e.g. 0 for "Terminal").
*    c            Byte to be stored.
*
*  Return value
*    Number of bytes which have been stored in the "Up"-buffer.
*
*  Notes
*    (1) If there is not enough space in the "Up"-buffer, the character is dropped.
*/

unsigned ESSEMI_SWD_PutCharSkip(unsigned BufferIndex, char c)
{
    ESSEMI_SWD_BUFFER_UP *pRing;
    unsigned              WrOff;
    unsigned              Status;
    //
    // Prepare
    //
    INIT();
    ESSEMI_SWD_LOCK();
    //
    // Get "to-host" ring buffer.
    //
    pRing = &_ESSEMI_SWD.aUp[BufferIndex];
    //
    // Get write position and handle wrap-around if necessary
    //
    WrOff = pRing->WrOff + 1;

    if (WrOff == pRing->SizeOfBuffer)
    {
        WrOff = 0;
    }

    //
    // Output byte if free space is available
    //
    if (WrOff != pRing->RdOff)
    {
        pRing->pBuffer[pRing->WrOff] = c;
        SWD__DMB();
        pRing->WrOff = WrOff;
        Status = 1;
    }
    else
    {
        Status = 0;
    }

    //
    // Finish up.
    //
    ESSEMI_SWD_UNLOCK();
    //
    return Status;
}

/*********************************************************************
*
*       ESSEMI_SWD_PutChar
*
*  Function description
*    Stores a single character/byte in ESSEMI SWD buffer.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used (e.g. 0 for "Terminal").
*    c            Byte to be stored.
*
*  Return value
*    Number of bytes which have been stored in the "Up"-buffer.
*
*  Notes
*    (1) Data is stored according to buffer flags.
*/

unsigned ESSEMI_SWD_PutChar(unsigned BufferIndex, char c)
{
    ESSEMI_SWD_BUFFER_UP *pRing;
    unsigned              WrOff;
    unsigned              Status;
    //
    // Prepare
    //
    INIT();
    ESSEMI_SWD_LOCK();
    //
    // Get "to-host" ring buffer.
    //
    pRing = &_ESSEMI_SWD.aUp[BufferIndex];
    //
    // Get write position and handle wrap-around if necessary
    //
    WrOff = pRing->WrOff + 1;

    if (WrOff == pRing->SizeOfBuffer)
    {
        WrOff = 0;
    }

    //
    // Wait for free space if mode is set to blocking
    //
    if (pRing->Flags == ESSEMI_SWD_MODE_BLOCK_IF_FIFO_FULL)
    {
        while (WrOff == pRing->RdOff)
        {
            ;
        }
    }

    //
    // Output byte if free space is available
    //
    if (WrOff != pRing->RdOff)
    {
        pRing->pBuffer[pRing->WrOff] = c;
        SWD__DMB();
        pRing->WrOff = WrOff;
        Status = 1;
    }
    else
    {
        Status = 0;
    }

    //
    // Finish up.
    //
    ESSEMI_SWD_UNLOCK();
    //
    return Status;
}

/*********************************************************************
*
*       ESSEMI_SWD_GetKey
*
*  Function description
*    Reads one character from the ESSEMI SWD buffer.
*    Host has previously stored data there.
*
*  Return value
*    <  0 -   No character available (buffer empty).
*    >= 0 -   Character which has been read. (Possible values: 0 - 255)
*
*  Notes
*    (1) This function is only specified for accesses to SWD buffer 0.
*/
int ESSEMI_SWD_GetKey(void)
{
    char c;
    int r;

    r = (int)ESSEMI_SWD_Read(0u, &c, 1u);

    if (r == 1)
    {
        r = (int)(unsigned char)c;
    }
    else
    {
        r = -1;
    }

    return r;
}

/*********************************************************************
*
*       ESSEMI_SWD_WaitKey
*
*  Function description
*    Waits until at least one character is avaible in the ESSEMI SWD buffer.
*    Once a character is available, it is read and this function returns.
*
*  Return value
*    >=0 -   Character which has been read.
*
*  Notes
*    (1) This function is only specified for accesses to SWD buffer 0
*    (2) This function is blocking if no character is present in SWD buffer
*/
int ESSEMI_SWD_WaitKey(void)
{
    int r;

    do
    {
        r = ESSEMI_SWD_GetKey();
    }
    while (r < 0);

    return r;
}

/*********************************************************************
*
*       ESSEMI_SWD_HasKey
*
*  Function description
*    Checks if at least one character for reading is available in the ESSEMI SWD buffer.
*
*  Return value
*    == 0 -     No characters are available to read.
*    == 1 -     At least one character is available.
*
*  Notes
*    (1) This function is only specified for accesses to SWD buffer 0
*/
int ESSEMI_SWD_HasKey(void)
{
    unsigned RdOff;
    int r;

    INIT();
    RdOff = _ESSEMI_SWD.aDown[0].RdOff;

    if (RdOff != _ESSEMI_SWD.aDown[0].WrOff)
    {
        r = 1;
    }
    else
    {
        r = 0;
    }

    return r;
}

/*********************************************************************
*
*       ESSEMI_SWD_HasData
*
*  Function description
*    Check if there is data from the host in the given buffer.
*
*  Return value:
*  ==0:  No data
*  !=0:  Data in buffer
*
*/
unsigned ESSEMI_SWD_HasData(unsigned BufferIndex)
{
    ESSEMI_SWD_BUFFER_DOWN *pRing;
    unsigned                v;

    pRing = &_ESSEMI_SWD.aDown[BufferIndex];
    v = pRing->WrOff;
    return v - pRing->RdOff;
}

/*********************************************************************
*
*       ESSEMI_SWD_HasDataUp
*
*  Function description
*    Check if there is data remaining to be sent in the given buffer.
*
*  Return value:
*  ==0:  No data
*  !=0:  Data in buffer
*
*/
unsigned ESSEMI_SWD_HasDataUp(unsigned BufferIndex)
{
    ESSEMI_SWD_BUFFER_UP *pRing;
    unsigned                v;

    pRing = &_ESSEMI_SWD.aUp[BufferIndex];
    v = pRing->RdOff;
    return pRing->WrOff - v;
}

/*********************************************************************
*
*       ESSEMI_SWD_AllocDownBuffer
*
*  Function description
*    Run-time configuration of the next down-buffer (H->T).
*    The next buffer, which is not used yet is configured.
*    This includes: Buffer address, size, name, flags, ...
*
*  Parameters
*    sName        Pointer to a constant name string.
*    pBuffer      Pointer to a buffer to be used.
*    BufferSize   Size of the buffer.
*    Flags        Operating modes. Define behavior if buffer is full (not enough space for entire message).
*
*  Return value
*    >= 0 - O.K. Buffer Index
*     < 0 - Error
*/
int ESSEMI_SWD_AllocDownBuffer(const char *sName, void *pBuffer, unsigned BufferSize, unsigned Flags)
{
    int BufferIndex;

    INIT();
    ESSEMI_SWD_LOCK();
    BufferIndex = 0;

    do
    {
        if (_ESSEMI_SWD.aDown[BufferIndex].pBuffer == NULL)
        {
            break;
        }

        BufferIndex++;
    }
    while (BufferIndex < _ESSEMI_SWD.MaxNumDownBuffers);

    if (BufferIndex < _ESSEMI_SWD.MaxNumDownBuffers)
    {
        _ESSEMI_SWD.aDown[BufferIndex].sName        = sName;
        _ESSEMI_SWD.aDown[BufferIndex].pBuffer      = (char *)pBuffer;
        _ESSEMI_SWD.aDown[BufferIndex].SizeOfBuffer = BufferSize;
        _ESSEMI_SWD.aDown[BufferIndex].RdOff        = 0u;
        _ESSEMI_SWD.aDown[BufferIndex].WrOff        = 0u;
        _ESSEMI_SWD.aDown[BufferIndex].Flags        = Flags;
        SWD__DMB();
    }
    else
    {
        BufferIndex = -1;
    }

    ESSEMI_SWD_UNLOCK();
    return BufferIndex;
}

/*********************************************************************
*
*       ESSEMI_SWD_AllocUpBuffer
*
*  Function description
*    Run-time configuration of the next up-buffer (T->H).
*    The next buffer, which is not used yet is configured.
*    This includes: Buffer address, size, name, flags, ...
*
*  Parameters
*    sName        Pointer to a constant name string.
*    pBuffer      Pointer to a buffer to be used.
*    BufferSize   Size of the buffer.
*    Flags        Operating modes. Define behavior if buffer is full (not enough space for entire message).
*
*  Return value
*    >= 0 - O.K. Buffer Index
*     < 0 - Error
*/
int ESSEMI_SWD_AllocUpBuffer(const char *sName, void *pBuffer, unsigned BufferSize, unsigned Flags)
{
    int BufferIndex;

    INIT();
    ESSEMI_SWD_LOCK();
    BufferIndex = 0;

    do
    {
        if (_ESSEMI_SWD.aUp[BufferIndex].pBuffer == NULL)
        {
            break;
        }

        BufferIndex++;
    }
    while (BufferIndex < _ESSEMI_SWD.MaxNumUpBuffers);

    if (BufferIndex < _ESSEMI_SWD.MaxNumUpBuffers)
    {
        _ESSEMI_SWD.aUp[BufferIndex].sName        = sName;
        _ESSEMI_SWD.aUp[BufferIndex].pBuffer      = (char *)pBuffer;
        _ESSEMI_SWD.aUp[BufferIndex].SizeOfBuffer = BufferSize;
        _ESSEMI_SWD.aUp[BufferIndex].RdOff        = 0u;
        _ESSEMI_SWD.aUp[BufferIndex].WrOff        = 0u;
        _ESSEMI_SWD.aUp[BufferIndex].Flags        = Flags;
        SWD__DMB();
    }
    else
    {
        BufferIndex = -1;
    }

    ESSEMI_SWD_UNLOCK();
    return BufferIndex;
}

/*********************************************************************
*
*       essemi_swd_configupbuffer
*
*  Function description
*    Run-time configuration of a specific up-buffer (T->H).
*    Buffer to be configured is specified by index.
*    This includes: Buffer address, size, name, flags, ...
*
*  Parameters
*    BufferIndex  Index of the buffer to configure.
*    sName        Pointer to a constant name string.
*    pBuffer      Pointer to a buffer to be used.
*    BufferSize   Size of the buffer.
*    Flags        Operating modes. Define behavior if buffer is full (not enough space for entire message).
*
*  Return value
*    >= 0 - O.K.
*     < 0 - Error
*
*  Additional information
*    Buffer 0 is configured on compile-time.
*    May only be called once per buffer.
*    Buffer name and flags can be reconfigured using the appropriate functions.
*/
int essemi_swd_configupbuffer(unsigned BufferIndex, const char *sName, void *pBuffer, unsigned BufferSize, unsigned Flags)
{
    int r;

    INIT();

    if (BufferIndex < (unsigned)_ESSEMI_SWD.MaxNumUpBuffers)
    {
        ESSEMI_SWD_LOCK();

        if (BufferIndex > 0u)
        {
            _ESSEMI_SWD.aUp[BufferIndex].sName        = sName;
            _ESSEMI_SWD.aUp[BufferIndex].pBuffer      = (char *)pBuffer;
            _ESSEMI_SWD.aUp[BufferIndex].SizeOfBuffer = BufferSize;
            _ESSEMI_SWD.aUp[BufferIndex].RdOff        = 0u;
            _ESSEMI_SWD.aUp[BufferIndex].WrOff        = 0u;
        }

        _ESSEMI_SWD.aUp[BufferIndex].Flags          = Flags;
        ESSEMI_SWD_UNLOCK();
        r =  0;
    }
    else
    {
        r = -1;
    }

    return r;
}

/*********************************************************************
*
*       essemi_swd_configdownbuffer
*
*  Function description
*    Run-time configuration of a specific down-buffer (H->T).
*    Buffer to be configured is specified by index.
*    This includes: Buffer address, size, name, flags, ...
*
*  Parameters
*    BufferIndex  Index of the buffer to configure.
*    sName        Pointer to a constant name string.
*    pBuffer      Pointer to a buffer to be used.
*    BufferSize   Size of the buffer.
*    Flags        Operating modes. Define behavior if buffer is full (not enough space for entire message).
*
*  Return value
*    >= 0  O.K.
*     < 0  Error
*
*  Additional information
*    Buffer 0 is configured on compile-time.
*    May only be called once per buffer.
*    Buffer name and flags can be reconfigured using the appropriate functions.
*/
int essemi_swd_configdownbuffer(unsigned BufferIndex, const char *sName, void *pBuffer, unsigned BufferSize, unsigned Flags)
{
    int r;

    INIT();

    if (BufferIndex < (unsigned)_ESSEMI_SWD.MaxNumDownBuffers)
    {
        ESSEMI_SWD_LOCK();

        if (BufferIndex > 0u)
        {
            _ESSEMI_SWD.aDown[BufferIndex].sName        = sName;
            _ESSEMI_SWD.aDown[BufferIndex].pBuffer      = (char *)pBuffer;
            _ESSEMI_SWD.aDown[BufferIndex].SizeOfBuffer = BufferSize;
            _ESSEMI_SWD.aDown[BufferIndex].RdOff        = 0u;
            _ESSEMI_SWD.aDown[BufferIndex].WrOff        = 0u;
        }

        _ESSEMI_SWD.aDown[BufferIndex].Flags          = Flags;
        SWD__DMB();
        ESSEMI_SWD_UNLOCK();
        r =  0;
    }
    else
    {
        r = -1;
    }

    return r;
}

/*********************************************************************
*
*       ESSEMI_SWD_SetNameUpBuffer
*
*  Function description
*    Run-time configuration of a specific up-buffer name (T->H).
*    Buffer to be configured is specified by index.
*
*  Parameters
*    BufferIndex  Index of the buffer to renamed.
*    sName        Pointer to a constant name string.
*
*  Return value
*    >= 0  O.K.
*     < 0  Error
*/
int ESSEMI_SWD_SetNameUpBuffer(unsigned BufferIndex, const char *sName)
{
    int r;

    INIT();

    if (BufferIndex < (unsigned)_ESSEMI_SWD.MaxNumUpBuffers)
    {
        ESSEMI_SWD_LOCK();
        _ESSEMI_SWD.aUp[BufferIndex].sName = sName;
        ESSEMI_SWD_UNLOCK();
        r =  0;
    }
    else
    {
        r = -1;
    }

    return r;
}

/*********************************************************************
*
*       ESSEMI_SWD_SetNameDownBuffer
*
*  Function description
*    Run-time configuration of a specific Down-buffer name (T->H).
*    Buffer to be configured is specified by index.
*
*  Parameters
*    BufferIndex  Index of the buffer to renamed.
*    sName        Pointer to a constant name string.
*
*  Return value
*    >= 0  O.K.
*     < 0  Error
*/
int ESSEMI_SWD_SetNameDownBuffer(unsigned BufferIndex, const char *sName)
{
    int r;

    INIT();

    if (BufferIndex < (unsigned)_ESSEMI_SWD.MaxNumDownBuffers)
    {
        ESSEMI_SWD_LOCK();
        _ESSEMI_SWD.aDown[BufferIndex].sName = sName;
        ESSEMI_SWD_UNLOCK();
        r =  0;
    }
    else
    {
        r = -1;
    }

    return r;
}

/*********************************************************************
*
*       ESSEMI_SWD_SetFlagsUpBuffer
*
*  Function description
*    Run-time configuration of specific up-buffer flags (T->H).
*    Buffer to be configured is specified by index.
*
*  Parameters
*    BufferIndex  Index of the buffer.
*    Flags        Flags to set for the buffer.
*
*  Return value
*    >= 0  O.K.
*     < 0  Error
*/
int ESSEMI_SWD_SetFlagsUpBuffer(unsigned BufferIndex, unsigned Flags)
{
    int r;

    INIT();

    if (BufferIndex < (unsigned)_ESSEMI_SWD.MaxNumUpBuffers)
    {
        ESSEMI_SWD_LOCK();
        _ESSEMI_SWD.aUp[BufferIndex].Flags = Flags;
        ESSEMI_SWD_UNLOCK();
        r =  0;
    }
    else
    {
        r = -1;
    }

    return r;
}

/*********************************************************************
*
*       ESSEMI_SWD_SetFlagsDownBuffer
*
*  Function description
*    Run-time configuration of specific Down-buffer flags (T->H).
*    Buffer to be configured is specified by index.
*
*  Parameters
*    BufferIndex  Index of the buffer to renamed.
*    Flags        Flags to set for the buffer.
*
*  Return value
*    >= 0  O.K.
*     < 0  Error
*/
int ESSEMI_SWD_SetFlagsDownBuffer(unsigned BufferIndex, unsigned Flags)
{
    int r;

    INIT();

    if (BufferIndex < (unsigned)_ESSEMI_SWD.MaxNumDownBuffers)
    {
        ESSEMI_SWD_LOCK();
        _ESSEMI_SWD.aDown[BufferIndex].Flags = Flags;
        ESSEMI_SWD_UNLOCK();
        r =  0;
    }
    else
    {
        r = -1;
    }

    return r;
}

/*********************************************************************
*
*       ESSEMI_SWD_Init
*
*  Function description
*    Initializes the SWD Control Block.
*    Should be used in RAM targets, at start of the application.
*
*/
void ESSEMI_SWD_Init(void)
{
    _DoInit();
}

/*********************************************************************
*
*       ESSEMI_SWD_SetTerminal
*
*  Function description
*    Sets the terminal to be used for output on channel 0.
*
*  Parameters
*    TerminalId  Index of the terminal.
*
*  Return value
*    >= 0  O.K.
*     < 0  Error (e.g. if SWD is configured for non-blocking mode and there was no space in the buffer to set the new terminal Id)
*/
int ESSEMI_SWD_SetTerminal(unsigned char TerminalId)
{
    unsigned char         ac[2];
    ESSEMI_SWD_BUFFER_UP *pRing;
    unsigned Avail;
    int r;
    //
    INIT();
    //
    r = 0;
    ac[0] = 0xFFu;

    if (TerminalId < sizeof(_aTerminalId))   // We only support a certain number of channels
    {
        ac[1] = _aTerminalId[TerminalId];
        pRing = &_ESSEMI_SWD.aUp[0];    // Buffer 0 is always reserved for terminal I/O, so we can use index 0 here, fixed
        ESSEMI_SWD_LOCK();    // Lock to make sure that no other task is writing into buffer, while we are and number of free bytes in buffer does not change downwards after checking and before writing

        if ((pRing->Flags & ESSEMI_SWD_MODE_MASK) == ESSEMI_SWD_MODE_BLOCK_IF_FIFO_FULL)
        {
            _ActiveTerminal = TerminalId;
            _WriteBlocking(pRing, (const char *)ac, 2u);
        }
        else                                                                                // Skipping mode or trim mode? => We cannot trim this command so handling is the same for both modes
        {
            Avail = _GetAvailWriteSpace(pRing);

            if (Avail >= 2)
            {
                _ActiveTerminal = TerminalId;    // Only change active terminal in case of success
                _WriteNoCheck(pRing, (const char *)ac, 2u);
            }
            else
            {
                r = -1;
            }
        }

        ESSEMI_SWD_UNLOCK();
    }
    else
    {
        r = -1;
    }

    return r;
}

/*********************************************************************
*
*       ESSEMI_SWD_TerminalOut
*
*  Function description
*    Writes a string to the given terminal
*     without changing the terminal for channel 0.
*
*  Parameters
*    TerminalId   Index of the terminal.
*    s            String to be printed on the terminal.
*
*  Return value
*    >= 0 - Number of bytes written.
*     < 0 - Error.
*
*/
int ESSEMI_SWD_TerminalOut(unsigned char TerminalId, const char *s)
{
    int                   Status;
    unsigned              FragLen;
    unsigned              Avail;
    ESSEMI_SWD_BUFFER_UP *pRing;
    //
    INIT();

    //
    // Validate terminal ID.
    //
    if (TerminalId < (char)sizeof(_aTerminalId))   // We only support a certain number of channels
    {
        //
        // Get "to-host" ring buffer.
        //
        pRing = &_ESSEMI_SWD.aUp[0];
        //
        // Need to be able to change terminal, write data, change back.
        // Compute the fixed and variable sizes.
        //
        FragLen = STRLEN(s);
        //
        // How we output depends upon the mode...
        //
        ESSEMI_SWD_LOCK();
        Avail = _GetAvailWriteSpace(pRing);

        switch (pRing->Flags & ESSEMI_SWD_MODE_MASK)
        {
            case ESSEMI_SWD_MODE_NO_BLOCK_SKIP:

                //
                // If we are in skip mode and there is no space for the whole
                // of this output, don't bother switching terminals at all.
                //
                if (Avail < (FragLen + 4u))
                {
                    Status = 0;
                }
                else
                {
                    _PostTerminalSwitch(pRing, TerminalId);
                    Status = (int)_WriteBlocking(pRing, s, FragLen);
                    _PostTerminalSwitch(pRing, _ActiveTerminal);
                }

                break;

            case ESSEMI_SWD_MODE_NO_BLOCK_TRIM:

                //
                // If we are in trim mode and there is not enough space for everything,
                // trim the output but always include the terminal switch.  If no room
                // for terminal switch, skip that totally.
                //
                if (Avail < 4u)
                {
                    Status = -1;
                }
                else
                {
                    _PostTerminalSwitch(pRing, TerminalId);
                    Status = (int)_WriteBlocking(pRing, s, (FragLen < (Avail - 4u)) ? FragLen : (Avail - 4u));
                    _PostTerminalSwitch(pRing, _ActiveTerminal);
                }

                break;

            case ESSEMI_SWD_MODE_BLOCK_IF_FIFO_FULL:
                //
                // If we are in blocking mode, output everything.
                //
                _PostTerminalSwitch(pRing, TerminalId);
                Status = (int)_WriteBlocking(pRing, s, FragLen);
                _PostTerminalSwitch(pRing, _ActiveTerminal);
                break;

            default:
                Status = -1;
                break;
        }

        //
        // Finish up.
        //
        ESSEMI_SWD_UNLOCK();
    }
    else
    {
        Status = -1;
    }

    return Status;
}

/*********************************************************************
*
*       ESSEMI_SWD_GetAvailWriteSpace
*
*  Function description
*    Returns the number of bytes available in the ring buffer.
*
*  Parameters
*    BufferIndex  Index of the up buffer.
*
*  Return value
*    Number of bytes that are free in the selected up buffer.
*/
unsigned ESSEMI_SWD_GetAvailWriteSpace(unsigned BufferIndex)
{
    return _GetAvailWriteSpace(&_ESSEMI_SWD.aUp[BufferIndex]);
}


/*********************************************************************
*
*       ESSEMI_SWD_GetBytesInBuffer()
*
*  Function description
*    Returns the number of bytes currently used in the up buffer.
*
*  Parameters
*    BufferIndex  Index of the up buffer.
*
*  Return value
*    Number of bytes that are used in the buffer.
*/
unsigned ESSEMI_SWD_GetBytesInBuffer(unsigned BufferIndex)
{
    unsigned RdOff;
    unsigned WrOff;
    unsigned r;
    //
    // Avoid warnings regarding volatile access order.  It's not a problem
    // in this case, but dampen compiler enthusiasm.
    //
    RdOff = _ESSEMI_SWD.aUp[BufferIndex].RdOff;
    WrOff = _ESSEMI_SWD.aUp[BufferIndex].WrOff;

    if (RdOff <= WrOff)
    {
        r = WrOff - RdOff;
    }
    else
    {
        r = _ESSEMI_SWD.aUp[BufferIndex].SizeOfBuffer - (WrOff - RdOff);
    }

    return r;
}

/*************************** End of file ****************************/
