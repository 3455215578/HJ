/**
  *********************************************************************************
  *
  * @file    essemi_swd_print_conf.h
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

#ifndef ESSEMI_SWD_CONF_H
#define ESSEMI_SWD_CONF_H

#ifdef __IAR_SYSTEMS_ICC__
#include <intrinsics.h>
#endif

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/
#ifndef   ESSEMI_SWD_MAX_NUM_UP_BUFFERS
#define ESSEMI_SWD_MAX_NUM_UP_BUFFERS             (3)     // Max. number of up-buffers (T->H) available on this target    (Default: 3)
#endif

#ifndef   ESSEMI_SWD_MAX_NUM_DOWN_BUFFERS
#define ESSEMI_SWD_MAX_NUM_DOWN_BUFFERS           (3)     // Max. number of down-buffers (H->T) available on this target  (Default: 3)
#endif

#ifndef   BUFFER_SIZE_UP
#define BUFFER_SIZE_UP                            (1024)  // Size of the buffer for terminal output of target, up to host (Default: 1k)
#endif

#ifndef   BUFFER_SIZE_DOWN
#define BUFFER_SIZE_DOWN                          (16)    // Size of the buffer for terminal input to target from host (Usually keyboard input) (Default: 16)
#endif

#ifndef   essemi_swd_printf_BUFFER_SIZE
#define essemi_swd_printf_BUFFER_SIZE             (64u)    // Size of buffer for SWD printf to bulk-send chars via SWD     (Default: 64)
#endif

#ifndef   ESSEMI_SWD_MODE_DEFAULT
#define ESSEMI_SWD_MODE_DEFAULT                   ESSEMI_SWD_MODE_NO_BLOCK_SKIP // Mode for pre-initialized terminal channel (buffer 0)
#endif

/*********************************************************************
*
*       SWD memcpy configuration
*
*       memcpy() is good for large amounts of data,
*       but the overhead is big for small amounts, which are usually stored via SWD.
*       With ESSEMI_SWD_MEMCPY_USE_BYTELOOP a simple byte loop can be used instead.
*
*       ESSEMI_SWD_MEMCPY() can be used to replace standard memcpy() in SWD functions.
*       This is may be required with memory access restrictions,
*       such as on Cortex-A devices with MMU.
*/
#ifndef   ESSEMI_SWD_MEMCPY_USE_BYTELOOP
#define ESSEMI_SWD_MEMCPY_USE_BYTELOOP              0 // 0: Use memcpy/ESSEMI_SWD_MEMCPY, 1: Use a simple byte-loop
#endif
//
// Example definition of ESSEMI_SWD_MEMCPY to external memcpy with GCC toolchains and Cortex-A targets
//
//#if ((defined __SES_ARM) || (defined __CROSSWORKS_ARM) || (defined __GNUC__)) && (defined (__ARM_ARCH_7A__))
//  #define ESSEMI_SWD_MEMCPY(pDest, pSrc, NumBytes)      ESSEMI_memcpy((pDest), (pSrc), (NumBytes))
//#endif

//
// Target is not allowed to perform other SWD operations while string still has not been stored completely.
// Otherwise we would probably end up with a mixed string in the buffer.
// If using  SWD from within interrupts, multiple tasks or multi processors, define the ESSEMI_SWD_LOCK() and ESSEMI_SWD_UNLOCK() function here.
//
// ESSEMI_SWD_MAX_INTERRUPT_PRIORITY can be used in the sample lock routines on Cortex-M3/4.
// Make sure to mask all interrupts which can send SWD data, i.e. generate SystemView events, or cause task switches.
// When high-priority interrupts must not be masked while sending SWD data, ESSEMI_SWD_MAX_INTERRUPT_PRIORITY needs to be adjusted accordingly.
// (Higher priority = lower priority number)
// Default value for embOS: 128u
// Default configuration in FreeRTOS: configMAX_SYSCALL_INTERRUPT_PRIORITY: ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
// In case of doubt mask all interrupts: 1 << (8 - BASEPRI_PRIO_BITS) i.e. 1 << 5 when 3 bits are implemented in NVIC
// or define ESSEMI_SWD_LOCK() to completely disable interrupts.
//
#ifndef   ESSEMI_SWD_MAX_INTERRUPT_PRIORITY
#define ESSEMI_SWD_MAX_INTERRUPT_PRIORITY         (0x20)   // Interrupt priority to lock on ESSEMI_SWD_LOCK on Cortex-M3/4 (Default: 0x20)
#endif

/*********************************************************************
*
*       SWD lock configuration for ESSEMI Embedded Studio,
*       Rowley CrossStudio and GCC
*/
#if ((defined(__SES_ARM) || defined(__SES_RISCV) || defined(__CROSSWORKS_ARM) || defined(__GNUC__) || defined(__clang__)) && !defined (__CC_ARM) && !defined(WIN32))
#if (defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_8M_BASE__))
#define ESSEMI_SWD_LOCK()   {                                                                   \
        unsigned int LockState;                                         \
        __asm volatile ("mrs   %0, primask  \n\t"                         \
                        "movs  r1, $1       \n\t"                         \
                        "msr   primask, r1  \n\t"                         \
                        : "=r" (LockState)                                \
                        :                                                 \
                        : "r1"                                            \
                       );

#define ESSEMI_SWD_UNLOCK()   __asm volatile ("msr   primask, %0  \n\t"                         \
        :                                                 \
        : "r" (LockState)                                 \
        :                                                 \
                                             );                                                \
}
#elif (defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_MAIN__))
#ifndef   ESSEMI_SWD_MAX_INTERRUPT_PRIORITY
#define ESSEMI_SWD_MAX_INTERRUPT_PRIORITY   (0x20)
#endif
#define ESSEMI_SWD_LOCK()   {                                                                   \
        unsigned int LockState;                                         \
        __asm volatile ("mrs   %0, basepri  \n\t"                         \
                        "mov   r1, %1       \n\t"                         \
                        "msr   basepri, r1  \n\t"                         \
                        : "=r" (LockState)                                \
                        : "i"(ESSEMI_SWD_MAX_INTERRUPT_PRIORITY)          \
                        : "r1"                                            \
                       );

#define ESSEMI_SWD_UNLOCK()   __asm volatile ("msr   basepri, %0  \n\t"                         \
        :                                                 \
        : "r" (LockState)                                 \
        :                                                 \
                                             );                                                \
}

#elif defined(__ARM_ARCH_7A__)
#define ESSEMI_SWD_LOCK() {                                                \
        unsigned int LockState;                       \
        __asm volatile ("mrs r1, CPSR \n\t"           \
                        "mov %0, r1 \n\t"             \
                        "orr r1, r1, #0xC0 \n\t"      \
                        "msr CPSR_c, r1 \n\t"         \
                        : "=r" (LockState)            \
                        :                             \
                        : "r1"                        \
                       );

#define ESSEMI_SWD_UNLOCK() __asm volatile ("mov r0, %0 \n\t"              \
        "mrs r1, CPSR \n\t"            \
        "bic r1, r1, #0xC0 \n\t"       \
        "and r0, r0, #0xC0 \n\t"       \
        "orr r1, r1, r0 \n\t"          \
        "msr CPSR_c, r1 \n\t"          \
        :                              \
        : "r" (LockState)              \
        : "r0", "r1"                   \
                                           );                             \
}
#elif defined(__riscv) || defined(__riscv_xlen)
#define ESSEMI_SWD_LOCK()  {                                               \
        unsigned int LockState;                       \
        __asm volatile ("csrr  %0, mstatus  \n\t"     \
                        "csrci mstatus, 8   \n\t"     \
                        "andi  %0, %0,  8   \n\t"     \
                        : "=r" (LockState)            \
                        :                             \
                        :                             \
                       );

#define ESSEMI_SWD_UNLOCK()    __asm volatile ("csrr  a1, mstatus  \n\t"     \
        "or    %0, %0, a1   \n\t"     \
        "csrs  mstatus, %0  \n\t"     \
        :                             \
        : "r"  (LockState)            \
        : "a1"                        \
                                              );                             \
}
#else
#define ESSEMI_SWD_LOCK()
#define ESSEMI_SWD_UNLOCK()
#endif
#endif

/*********************************************************************
*
*       SWD lock configuration for IAR EWARM
*/
#ifdef __ICCARM__
#if (defined (__ARM6M__)          && (__CORE__ == __ARM6M__))             ||                      \
      (defined (__ARM8M_BASELINE__) && (__CORE__ == __ARM8M_BASELINE__))
#define ESSEMI_SWD_LOCK()   {                                                                   \
        unsigned int LockState;                                           \
        LockState = __get_PRIMASK();                                      \
        __set_PRIMASK(1);

#define ESSEMI_SWD_UNLOCK()   __set_PRIMASK(LockState);                                         \
    }
#elif (defined (__ARM7EM__)         && (__CORE__ == __ARM7EM__))          ||                      \
        (defined (__ARM7M__)          && (__CORE__ == __ARM7M__))           ||                      \
        (defined (__ARM8M_MAINLINE__) && (__CORE__ == __ARM8M_MAINLINE__))  ||                      \
        (defined (__ARM8M_MAINLINE__) && (__CORE__ == __ARM8M_MAINLINE__))
#ifndef   ESSEMI_SWD_MAX_INTERRUPT_PRIORITY
    #define ESSEMI_SWD_MAX_INTERRUPT_PRIORITY   (0x20)
#endif
#define ESSEMI_SWD_LOCK()   {                                                                   \
        unsigned int LockState;                                           \
        LockState = __get_BASEPRI();                                      \
        __set_BASEPRI(ESSEMI_SWD_MAX_INTERRUPT_PRIORITY);

#define ESSEMI_SWD_UNLOCK()   __set_BASEPRI(LockState);                                         \
    }
#endif
#endif

/*********************************************************************
*
*       SWD lock configuration for IAR RX
*/
#ifdef __ICCRX__
#define ESSEMI_SWD_LOCK()   {                                                                     \
        unsigned long LockState;                                            \
        LockState = __get_interrupt_state();                                \
        __disable_interrupt();

#define ESSEMI_SWD_UNLOCK()   __set_interrupt_state(LockState);                                   \
    }
#endif

/*********************************************************************
*
*       SWD lock configuration for IAR RL78
*/
#ifdef __ICCRL78__
#define ESSEMI_SWD_LOCK()   {                                                                     \
        __istate_t LockState;                                               \
        LockState = __get_interrupt_state();                                \
        __disable_interrupt();

#define ESSEMI_SWD_UNLOCK()   __set_interrupt_state(LockState);                                   \
    }
#endif

/*********************************************************************
*
*       SWD lock configuration for KEIL ARM
*/
#ifdef __CC_ARM
#if (defined __TARGET_ARCH_6S_M)
#define ESSEMI_SWD_LOCK()   {                                                                   \
        unsigned int LockState;                                           \
        register unsigned char PRIMASK __asm( "primask");                 \
        LockState = PRIMASK;                                              \
        PRIMASK = 1u;                                                     \
        __schedule_barrier();

#define ESSEMI_SWD_UNLOCK()   PRIMASK = LockState;                                              \
    __schedule_barrier();                                             \
    }
#elif (defined(__TARGET_ARCH_7_M) || defined(__TARGET_ARCH_7E_M))
#ifndef   ESSEMI_SWD_MAX_INTERRUPT_PRIORITY
    #define ESSEMI_SWD_MAX_INTERRUPT_PRIORITY   (0x20)
#endif
#define ESSEMI_SWD_LOCK()   {                                                                   \
        unsigned int LockState;                                           \
        register unsigned char BASEPRI __asm( "basepri");                 \
        LockState = BASEPRI;                                              \
        BASEPRI = ESSEMI_SWD_MAX_INTERRUPT_PRIORITY;                      \
        __schedule_barrier();

#define ESSEMI_SWD_UNLOCK()   BASEPRI = LockState;                                              \
    __schedule_barrier();                                             \
    }
#endif
#endif

/*********************************************************************
*
*       SWD lock configuration for TI ARM
*/
#ifdef __TI_ARM__
#if defined (__TI_ARM_V6M0__)
#define ESSEMI_SWD_LOCK()   {                                                                   \
        unsigned int LockState;                                           \
        LockState = __get_PRIMASK();                                      \
        __set_PRIMASK(1);

#define ESSEMI_SWD_UNLOCK()   __set_PRIMASK(LockState);                                         \
    }
#elif (defined (__TI_ARM_V7M3__) || defined (__TI_ARM_V7M4__))
#ifndef   ESSEMI_SWD_MAX_INTERRUPT_PRIORITY
    #define ESSEMI_SWD_MAX_INTERRUPT_PRIORITY   (0x20)
#endif
#define ESSEMI_SWD_LOCK()   {                                                                   \
        unsigned int LockState;                                           \
        LockState = _set_interrupt_priority(ESSEMI_SWD_MAX_INTERRUPT_PRIORITY);

#define ESSEMI_SWD_UNLOCK()   _set_interrupt_priority(LockState);                               \
    }
#endif
#endif

/*********************************************************************
*
*       SWD lock configuration for CCRX
*/
#ifdef __RX
#define ESSEMI_SWD_LOCK()   {                                                                     \
        unsigned long LockState;                                            \
        LockState = get_psw() & 0x010000;                                   \
        clrpsw_i();

#define ESSEMI_SWD_UNLOCK()   set_psw(get_psw() | LockState);                                     \
    }
#endif

/*********************************************************************
*
*       SWD lock configuration for embOS Simulation on Windows
*       (Can also be used for generic SWD locking with embOS)
*/
#if defined(WIN32) || defined(ESSEMI_SWD_LOCK_EMBOS)

void OS_SIM_EnterCriticalSection(void);
void OS_SIM_LeaveCriticalSection(void);

#define ESSEMI_SWD_LOCK()       {                                                                   \
        OS_SIM_EnterCriticalSection();

#define ESSEMI_SWD_UNLOCK()       OS_SIM_LeaveCriticalSection();                                    \
    }
#endif

/*********************************************************************
*
*       SWD lock configuration fallback
*/
#ifndef   ESSEMI_SWD_LOCK
#define ESSEMI_SWD_LOCK()                // Lock SWD (nestable)   (i.e. disable interrupts)
#endif

#ifndef   ESSEMI_SWD_UNLOCK
#define ESSEMI_SWD_UNLOCK()              // Unlock SWD (nestable) (i.e. enable previous interrupt lock state)
#endif

#endif
/*************************** End of file ****************************/
