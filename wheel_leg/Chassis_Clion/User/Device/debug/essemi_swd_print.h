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
#ifndef ESSEMI_SWD_H
#define ESSEMI_SWD_H

#include "essemi_swd_print_conf.h"



/*********************************************************************
*
*       Defines, defaults
*
**********************************************************************
*/
#ifndef SWD_USE_ASM
#if (defined __SES_ARM)                       // ESSEMI Embedded Studio
#define _CC_HAS_SWD_ASM_SUPPORT 1
#elif (defined __CROSSWORKS_ARM)              // Rowley Crossworks
#define _CC_HAS_SWD_ASM_SUPPORT 1
#elif (defined __ARMCC_VERSION)
#define _CC_HAS_SWD_ASM_SUPPORT 0
#elif (defined __GNUC__)                      // GCC
#define _CC_HAS_SWD_ASM_SUPPORT 1
#elif (defined __clang__)                     // Clang compiler
#define _CC_HAS_SWD_ASM_SUPPORT 1
    #elif ((defined __IASMARM__) || (defined __ICCARM__))  // IAR assembler/compiler
        #define _CC_HAS_SWD_ASM_SUPPORT 1
    #else
        #define _CC_HAS_SWD_ASM_SUPPORT 0
#endif
#if ((defined __IASMARM__) || (defined __ICCARM__))  // IAR assembler/compiler
//
        // IAR assembler / compiler
        //
        #if (defined __ARM7M__)                            // Needed for old versions that do not know the define yet
            #if (__CORE__ == __ARM7M__)                      // Cortex-M3
                #define _CORE_HAS_SWD_ASM_SUPPORT 1
            #endif
        #endif
        #if (defined __ARM7EM__)                           // Needed for old versions that do not know the define yet
            #if (__CORE__ == __ARM7EM__)                     // Cortex-M4/M7
                #define _CORE_HAS_SWD_ASM_SUPPORT 1
                #define _CORE_NEEDS_DMB 1
                #define SWD__DMB() asm("DMB");
            #endif
        #endif
        #if (defined __ARM8M_BASELINE__)                   // Needed for old versions that do not know the define yet
            #if (__CORE__ == __ARM8M_BASELINE__)             // Cortex-M23
                #define _CORE_HAS_SWD_ASM_SUPPORT 1
                #define _CORE_NEEDS_DMB 1
                #define SWD__DMB() asm("DMB");
            #endif
        #endif
        #if (defined __ARM8M_MAINLINE__)                   // Needed for old versions that do not know the define yet
            #if (__CORE__ == __ARM8M_MAINLINE__)             // Cortex-M33
                #define _CORE_HAS_SWD_ASM_SUPPORT 1
                #define _CORE_NEEDS_DMB 1
                #define SWD__DMB() asm("DMB");
            #endif
        #endif
#else
//
// GCC / Clang
//
#if (defined __ARM_ARCH_7M__)                 // Cortex-M3
#define _CORE_HAS_SWD_ASM_SUPPORT 1
#elif (defined __ARM_ARCH_7EM__)              // Cortex-M4/M7
#define _CORE_HAS_SWD_ASM_SUPPORT 1
#define _CORE_NEEDS_DMB           1
#define SWD__DMB() __asm volatile ("dmb\n" : : :);
#elif (defined __ARM_ARCH_8M_BASE__)          // Cortex-M23
#define _CORE_HAS_SWD_ASM_SUPPORT 1
            #define _CORE_NEEDS_DMB           1
            #define SWD__DMB() __asm volatile ("dmb\n" : : :);
        #elif (defined __ARM_ARCH_8M_MAIN__)          // Cortex-M33
            #define _CORE_HAS_SWD_ASM_SUPPORT 1
            #define _CORE_NEEDS_DMB           1
            #define SWD__DMB() __asm volatile ("dmb\n" : : :);
        #else
            #define _CORE_HAS_SWD_ASM_SUPPORT 0
#endif
#endif
//
// If IDE and core support the ASM version, enable ASM version by default
//
#ifndef _CORE_HAS_SWD_ASM_SUPPORT
#define _CORE_HAS_SWD_ASM_SUPPORT 0              // Default for unknown cores
#endif
#if (_CC_HAS_SWD_ASM_SUPPORT && _CORE_HAS_SWD_ASM_SUPPORT)
#define SWD_USE_ASM                           (1)
#else
#define SWD_USE_ASM                           (0)
#endif
#endif

//
// We need to know if a DMB is needed to make sure that on Cortex-M7 etc.
// the order of accesses to the ring buffers is guaranteed
// Needed for: Cortex-M7, Cortex-M23, Cortex-M33
//
#ifndef _CORE_NEEDS_DMB
#define _CORE_NEEDS_DMB 0
#endif

#ifndef SWD__DMB
#if _CORE_NEEDS_DMB
        #error "Don't know how to place inline assembly for DMB"
    #else
        #define SWD__DMB()
    #endif
#endif

#ifndef ESSEMI_SWD_ASM  // defined when ESSEMI_SWD.h is included from assembly file
#include <stdlib.h>
#include <stdarg.h>

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/

/*********************************************************************
*
*       Types
*
**********************************************************************
*/

//
// Description for a circular buffer (also called "ring buffer")
// which is used as up-buffer (T->H)
//
typedef struct
{
    const     char    *sName;         // Optional name. Standard names so far are: "Terminal", "SysView", "J-Scope_t4i4"
    char    *pBuffer;       // Pointer to start of buffer
    unsigned SizeOfBuffer;  // Buffer size in bytes. Note that one byte is lost, as this implementation does not fill up the buffer in order to avoid the problem of being unable to distinguish between full and empty.
    unsigned WrOff;         // Position of next item to be written by either target.
    volatile  unsigned RdOff;         // Position of next item to be read by host. Must be volatile since it may be modified by host.
    unsigned Flags;         // Contains configuration flags
} ESSEMI_SWD_BUFFER_UP;

//
// Description for a circular buffer (also called "ring buffer")
// which is used as down-buffer (H->T)
//
typedef struct
{
    const     char    *sName;         // Optional name. Standard names so far are: "Terminal", "SysView", "J-Scope_t4i4"
    char    *pBuffer;       // Pointer to start of buffer
    unsigned SizeOfBuffer;  // Buffer size in bytes. Note that one byte is lost, as this implementation does not fill up the buffer in order to avoid the problem of being unable to distinguish between full and empty.
    volatile  unsigned WrOff;         // Position of next item to be written by host. Must be volatile since it may be modified by host.
    unsigned RdOff;         // Position of next item to be read by target (down-buffer).
    unsigned Flags;         // Contains configuration flags
} ESSEMI_SWD_BUFFER_DOWN;

//
// SWD control block which describes the number of buffers available
// as well as the configuration for each buffer
//
//
typedef struct
{
    char                    acID[16];                                 // Initialized to "ESSEMI SWD"
    int                     MaxNumUpBuffers;                          // Initialized to ESSEMI_SWD_MAX_NUM_UP_BUFFERS (type. 2)
    int                     MaxNumDownBuffers;                        // Initialized to ESSEMI_SWD_MAX_NUM_DOWN_BUFFERS (type. 2)
    ESSEMI_SWD_BUFFER_UP    aUp[ESSEMI_SWD_MAX_NUM_UP_BUFFERS];       // Up buffers, transferring information up from target via debug probe to host
    ESSEMI_SWD_BUFFER_DOWN  aDown[ESSEMI_SWD_MAX_NUM_DOWN_BUFFERS];   // Down buffers, transferring information down from host via debug probe to target
} ESSEMI_SWD_CB;

/*********************************************************************
*
*       Global data
*
**********************************************************************
*/
extern ESSEMI_SWD_CB _ESSEMI_SWD;

/*********************************************************************
*
*       SWD API functions
*
**********************************************************************
*/
#ifdef __cplusplus
extern "C" {
#endif
int          ESSEMI_SWD_AllocDownBuffer(const char *sName, void *pBuffer, unsigned BufferSize, unsigned Flags);
int          ESSEMI_SWD_AllocUpBuffer(const char *sName, void *pBuffer, unsigned BufferSize, unsigned Flags);
int          essemi_swd_configupbuffer(unsigned BufferIndex, const char *sName, void *pBuffer, unsigned BufferSize, unsigned Flags);
int          essemi_swd_configdownbuffer(unsigned BufferIndex, const char *sName, void *pBuffer, unsigned BufferSize, unsigned Flags);
int          ESSEMI_SWD_GetKey(void);
unsigned     ESSEMI_SWD_HasData(unsigned BufferIndex);
int          ESSEMI_SWD_HasKey(void);
unsigned     ESSEMI_SWD_HasDataUp(unsigned BufferIndex);
void         ESSEMI_SWD_Init(void);
unsigned     ESSEMI_SWD_Read(unsigned BufferIndex,       void *pBuffer, unsigned BufferSize);
unsigned     ESSEMI_SWD_ReadNoLock(unsigned BufferIndex,       void *pData,   unsigned BufferSize);
int          ESSEMI_SWD_SetNameDownBuffer(unsigned BufferIndex, const char *sName);
int          ESSEMI_SWD_SetNameUpBuffer(unsigned BufferIndex, const char *sName);
int          ESSEMI_SWD_SetFlagsDownBuffer(unsigned BufferIndex, unsigned Flags);
int          ESSEMI_SWD_SetFlagsUpBuffer(unsigned BufferIndex, unsigned Flags);
int          ESSEMI_SWD_WaitKey(void);
unsigned     ESSEMI_SWD_Write(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes);
unsigned     ESSEMI_SWD_WriteNoLock(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes);
unsigned     ESSEMI_SWD_WriteSkipNoLock(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes);
unsigned     ESSEMI_SWD_ASM_WriteSkipNoLock(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes);
unsigned     ESSEMI_SWD_WriteString(unsigned BufferIndex, const char *s);
void         ESSEMI_SWD_WriteWithOverwriteNoLock(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes);
unsigned     ESSEMI_SWD_PutChar(unsigned BufferIndex, char c);
unsigned     ESSEMI_SWD_PutCharSkip(unsigned BufferIndex, char c);
unsigned     ESSEMI_SWD_PutCharSkipNoLock(unsigned BufferIndex, char c);
unsigned     ESSEMI_SWD_GetAvailWriteSpace(unsigned BufferIndex);
unsigned     ESSEMI_SWD_GetBytesInBuffer(unsigned BufferIndex);
//
// Function macro for performance optimization
//
#define      ESSEMI_SWD_HASDATA(n)       (_ESSEMI_SWD.aDown[n].WrOff - _ESSEMI_SWD.aDown[n].RdOff)

#if SWD_USE_ASM
#define ESSEMI_SWD_WriteSkipNoLock  ESSEMI_SWD_ASM_WriteSkipNoLock
#endif

/*********************************************************************
*
*       SWD transfer functions to send SWD data via other channels.
*
**********************************************************************
*/
unsigned     ESSEMI_SWD_ReadUpBuffer(unsigned BufferIndex, void *pBuffer, unsigned BufferSize);
unsigned     ESSEMI_SWD_ReadUpBufferNoLock(unsigned BufferIndex, void *pData, unsigned BufferSize);
unsigned     ESSEMI_SWD_WriteDownBuffer(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes);
unsigned     ESSEMI_SWD_WriteDownBufferNoLock(unsigned BufferIndex, const void *pBuffer, unsigned NumBytes);

#define      ESSEMI_SWD_HASDATA_UP(n)    (_ESSEMI_SWD.aUp[n].WrOff - _ESSEMI_SWD.aUp[n].RdOff)

/*********************************************************************
*
*       SWD "Terminal" API functions
*
**********************************************************************
*/
int     ESSEMI_SWD_SetTerminal(unsigned char TerminalId);
int     ESSEMI_SWD_TerminalOut(unsigned char TerminalId, const char *s);

/*********************************************************************
*
*       SWD printf functions (require essemi_swd_printf.c)
*
**********************************************************************
*/
int essemi_swd_printf(unsigned BufferIndex, const char *sFormat, ...);
int ESSEMI_SWD_vprintf(unsigned BufferIndex, const char *sFormat, va_list *pParamList);

#ifdef __cplusplus
}
#endif

#endif // ifndef(ESSEMI_SWD_ASM)

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

//
// Operating modes. Define behavior if buffer is full (not enough space for entire message)
//
#define ESSEMI_SWD_MODE_NO_BLOCK_SKIP         (0)     // Skip. Do not block, output nothing. (Default)
#define ESSEMI_SWD_MODE_NO_BLOCK_TRIM         (1)     // Trim: Do not block, output as much as fits.
#define ESSEMI_SWD_MODE_BLOCK_IF_FIFO_FULL    (2)     // Block: Wait until there is space in the buffer.
#define ESSEMI_SWD_MODE_MASK                  (3)

//
// Control sequences, based on ANSI.
// Can be used to control color, and clear the screen
//
#define SWD_CTRL_RESET                "\x1B[0m"         // Reset to default colors
#define SWD_CTRL_CLEAR                "\x1B[2J"         // Clear screen, reposition cursor to top left

#define SWD_CTRL_TEXT_BLACK           "\x1B[2;30m"
#define SWD_CTRL_TEXT_RED             "\x1B[2;31m"
#define SWD_CTRL_TEXT_GREEN           "\x1B[2;32m"
#define SWD_CTRL_TEXT_YELLOW          "\x1B[2;33m"
#define SWD_CTRL_TEXT_BLUE            "\x1B[2;34m"
#define SWD_CTRL_TEXT_MAGENTA         "\x1B[2;35m"
#define SWD_CTRL_TEXT_CYAN            "\x1B[2;36m"
#define SWD_CTRL_TEXT_WHITE           "\x1B[2;37m"

#define SWD_CTRL_TEXT_BRIGHT_BLACK    "\x1B[1;30m"
#define SWD_CTRL_TEXT_BRIGHT_RED      "\x1B[1;31m"
#define SWD_CTRL_TEXT_BRIGHT_GREEN    "\x1B[1;32m"
#define SWD_CTRL_TEXT_BRIGHT_YELLOW   "\x1B[1;33m"
#define SWD_CTRL_TEXT_BRIGHT_BLUE     "\x1B[1;34m"
#define SWD_CTRL_TEXT_BRIGHT_MAGENTA  "\x1B[1;35m"
#define SWD_CTRL_TEXT_BRIGHT_CYAN     "\x1B[1;36m"
#define SWD_CTRL_TEXT_BRIGHT_WHITE    "\x1B[1;37m"

#define SWD_CTRL_BG_BLACK             "\x1B[24;40m"
#define SWD_CTRL_BG_RED               "\x1B[24;41m"
#define SWD_CTRL_BG_GREEN             "\x1B[24;42m"
#define SWD_CTRL_BG_YELLOW            "\x1B[24;43m"
#define SWD_CTRL_BG_BLUE              "\x1B[24;44m"
#define SWD_CTRL_BG_MAGENTA           "\x1B[24;45m"
#define SWD_CTRL_BG_CYAN              "\x1B[24;46m"
#define SWD_CTRL_BG_WHITE             "\x1B[24;47m"

#define SWD_CTRL_BG_BRIGHT_BLACK      "\x1B[4;40m"
#define SWD_CTRL_BG_BRIGHT_RED        "\x1B[4;41m"
#define SWD_CTRL_BG_BRIGHT_GREEN      "\x1B[4;42m"
#define SWD_CTRL_BG_BRIGHT_YELLOW     "\x1B[4;43m"
#define SWD_CTRL_BG_BRIGHT_BLUE       "\x1B[4;44m"
#define SWD_CTRL_BG_BRIGHT_MAGENTA    "\x1B[4;45m"
#define SWD_CTRL_BG_BRIGHT_CYAN       "\x1B[4;46m"
#define SWD_CTRL_BG_BRIGHT_WHITE      "\x1B[4;47m"


#endif

/*************************** End of file ****************************/
