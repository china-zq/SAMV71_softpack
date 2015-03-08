/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011, Atmel Corporation
 *
 * All rights reserved.
 *

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#ifndef _COMPILER_H_
#define _COMPILER_H_

/*
 * Peripherals registers definitions
 */
#include "include/samv7/samv71.h"


//_____ D E C L A R A T I O N S ____________________________________________

#ifndef __ASSEMBLY__

#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

/* Define WEAK attribute */
#if defined   ( __CC_ARM   )
    #define WEAK __attribute__ ((weak))
#elif defined ( __ICCARM__ )
    #define WEAK __weak
#elif defined (  __GNUC__  )
    #define WEAK __attribute__ ((weak))
#endif

/* Define NO_INIT attribute */
#if defined   ( __CC_ARM   )
    #define NO_INIT
#elif defined ( __ICCARM__ )
    #define NO_INIT __no_init
#elif defined (  __GNUC__  )
    #define NO_INIT
#endif
   
/* Define RAMFUNC attribute */
#ifdef __ICCARM__
#define RAMFUNC __ramfunc
#else
#define RAMFUNC __attribute__ ((section (".ramfunc")))
#endif

/* Define NO_INIT attribute */
#if defined   ( __CC_ARM   )
    #define memory_sync()        __dsb(15);__isb(15);
#elif defined ( __ICCARM__ )
    #define memory_sync()        __DSB();__ISB();
#elif defined (  __GNUC__  )
    #define memory_sync()        __DSB();__ISB();
#endif

/* Define NO_INIT attribute */
#if defined   ( __CC_ARM   )
    #define memory_barrier()        __dmb(15);
#elif defined ( __ICCARM__ )
    #define memory_barrier()        __DMB();
#elif defined (  __GNUC__  )
    #define memory_barrier()        __DMB();
#endif

/*! \name Token Paste
 *
 * Paste N preprocessing tokens together, these tokens being allowed to be \#defined.
 *
 * May be used only within macros with the tokens passed as arguments if the tokens are \#defined.
 *
 * For example, writing TPASTE2(U, WIDTH) within a macro \#defined by
 * UTYPE(WIDTH) and invoked as UTYPE(UL_WIDTH) with UL_WIDTH \#defined as 32 is
 * equivalent to writing U32.
 */
//! @{
#define TPASTE2( a, b)                            a##b
#define TPASTE3( a, b, c)                         a##b##c
//! @}

/*! \name Absolute Token Paste
 *
 * Paste N preprocessing tokens together, these tokens being allowed to be \#defined.
 *
 * No restriction of use if the tokens are \#defined.
 *
 * For example, writing ATPASTE2(U, UL_WIDTH) anywhere with UL_WIDTH \#defined
 * as 32 is equivalent to writing U32.
 */
//! @{
#define ATPASTE2( a, b)                           TPASTE2( a, b)
#define ATPASTE3( a, b, c)                        TPASTE3( a, b, c)
//! @}


/**
 * \def barrier
 * \brief Memory barrier
 */
#if defined(__CC_ARM)
#  define barrier()
#elif defined(__GNUC__)
#  define barrier()        asm volatile("" ::: "memory")
#elif defined(__ICCAVR32__)
#  define barrier()        __asm__ __volatile__ ("")
#elif defined(__ICCARM__)
#  define barrier()        __asm__ __volatile__ ("dmb")
#endif

/**
 * \brief Emit the compiler pragma \a arg.
 *
 * \param arg The pragma directive as it would appear after \e \#pragma
 * (i.e. not stringified).
 */
#define COMPILER_PRAGMA(arg)            _Pragma(#arg)

/**
 * \def COMPILER_PACK_SET(alignment)
 * \brief Set maximum alignment for subsequent structure and union
 * definitions to \a alignment.
 */
#define COMPILER_PACK_SET(alignment)   COMPILER_PRAGMA(pack(alignment))

/**
 * \def COMPILER_PACK_RESET()
 * \brief Set default alignment for subsequent structure and union
 * definitions.
 */
#define COMPILER_PACK_RESET()          COMPILER_PRAGMA(pack())


/**
 * \brief Set aligned boundary.
 */
#if defined   ( __CC_ARM   )
    #define COMPILER_ALIGNED(a)    __attribute__((__aligned__(a)))
#elif defined ( __ICCARM__ )
    #define COMPILER_ALIGNED(a)    COMPILER_PRAGMA(data_alignment = a)
#elif defined (  __GNUC__  )
    #define COMPILER_ALIGNED(a)    __attribute__((__aligned__(a)))
#endif

/**
 * \brief Set word-aligned boundary.
 */

#if defined   ( __CC_ARM   )
    #define COMPILER_WORD_ALIGNED    __attribute__((__aligned__(4)))
#elif defined ( __ICCARM__ )
    #define COMPILER_WORD_ALIGNED    COMPILER_PRAGMA(data_alignment = 4)
#elif defined (  __GNUC__  )
    #define COMPILER_WORD_ALIGNED    __attribute__((__aligned__(4)))
#endif


/**
 * \name System Register Access
 * @{
 */
#if defined(__GNUC__) || defined(__DOXYGEN__)
/**
  * \brief Get value of system register
 *
 * \param reg Address of the system register of which to get the value.
 *
 * \return Value of system register \a reg.
 */
#  define sysreg_read(reg)               __builtin_mfsr(reg)

/**
 * \brief Set value of system register
 *
 * \param reg Address of the system register of which to set the value.
 * \param val Value to set the system register \a reg to.
 */
#  define sysreg_write(reg, val)         __builtin_mtsr(reg, val)

#elif defined(__ICCARM__)
#  define sysreg_read(reg)               __get_system_register(reg)
#  define sysreg_write(reg, val)         __set_system_register(reg, val)
#endif

// Deprecated definitions
#define Get_system_register(reg)         sysreg_read(reg)
#define Set_system_register(reg, val)    sysreg_write(reg, val)
//! @}



//_____ M A C R O S ________________________________________________________

/*! \name Usual Constants
 */
//! @{
#define DISABLE   0
#define ENABLE    1
#define DISABLED  0
#define ENABLED   1
#define OFF       0
#define ON        1
#define FALSE     0
#define TRUE      1
#ifndef __cplusplus
#if !defined(__bool_true_false_are_defined)
#define false     FALSE
#define true      TRUE
#endif
#endif
#define KO        0
#define OK        1
#define PASS      0
#define FAIL      1
#define LOW       0
#define HIGH      1
#define CLR       0
#define SET       1
//! @}


//! \name Optimization Control
//@{

/**
 * \def likely(exp)
 * \brief The expression \a exp is likely to be true
 */
#ifndef likely
# define likely(exp)    (exp)
#endif

/**
 * \def unlikely(exp)
 * \brief The expression \a exp is unlikely to be true
 */
#ifndef unlikely
# define unlikely(exp)  (exp)
#endif

/**
 * \def is_constant(exp)
 * \brief Determine if an expression evaluates to a constant value.
 *
 * \param exp Any expression
 *
 * \return true if \a exp is constant, false otherwise.
 */
#ifdef __GNUC__
# define is_constant(exp)       __builtin_constant_p(exp)
#else
# define is_constant(exp)       (0)
#endif


/*! \brief This macro makes the CPU take a small break for a few cycles. This should
 *         be used when waiting for an event. It will reduce the internal bus load.
 *
 * "sub pc, pc, -4" (or "sub pc, -2") forces the IF stage to wait until the result
 * of the calculation before it can fetch the next instruction. This makes sure
 * there are nothing stuck in the LS pipe when you start a new iteration and guarantees
 * to flush the pipeline without having any other effect.
 * (A nop doesn't have any effect on the IF stage.)
 */
#if (defined __GNUC__)
# define cpu_relax()             __asm__ __volatile__("sub pc, pc, -4" ::: "memory", "cc")
#elif (defined __ICCAVR32__)
# define cpu_relax()             __asm__ __volatile__("sub pc, pc, -4")
#endif


/*! \brief This macro is used to test fatal errors.
 *
 * The macro tests if the expression is FALSE. If it is, a fatal error is
 * detected and the application hangs up.
 *
 * \param expr  Expression to evaluate and supposed to be non-zero.
 */
#ifdef _ASSERT_ENABLE_
  #define Assert(expr) \
  {\
    if (!(expr)) while (TRUE);\
  }
#else
  #define Assert(expr)
#endif

#endif // __ASSEMBLY__

#endif  // _COMPILER_H_
