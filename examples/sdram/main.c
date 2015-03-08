/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Atmel Corporation
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

/**
 *  \page sdram SDRAM example
 *
 *  \section Purpose
 *
 *  The SDRAM example will help new users get familiar with Atmel's
 *  samv7 family of micro-controllers. This basic application shows Shows how 
 *  to initialize and perform read and write a SDRAM memory.
 *
 *  \section Requirements
 *
 *  This package can be used with SAM V71 Xplained Ultra board.
 *
 *  \section Description
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board. Please
 *     refer to the Getting Started with SAM V71 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rates
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -#In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- SDRAM Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 *  \section References
 *  - sdram/main.c
 *  - trace.h
 */

/** \file
 *
 *  This file contains all the specific code for the SDRAM example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
#define SDRAM_BA0 (1 << 20)
#define SDRAM_BA1 (1 << 21)

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Test SDRAM access
 * \param baseAddr Base address of SDRAM
 * \param size  Size of memory in byte
 * \return 1: OK, 0: Error
 */
static uint32_t _sdramAccess(uint32_t baseAddr, uint32_t size)
{
    uint32_t i;
    uint32_t *ptr = (uint32_t *) baseAddr;
    uint32_t ret = 1;
    for (i = 0; i < size << 2; ++i) {
        if (i & 1) {
            ptr[i] = 0x55AA55AA ;
        }
        else {
            ptr[i] = 0xAA55AA55 ;
        }
    }
    printf("\r\n");
    for (i = 0; i <  size << 2; ++i) {
        if (i & 1) {
            if (ptr[i] != 0x55AA55AA ){
                printf("-E- Expected:0x55AA55AA, read %x @ %x \n\r" ,ptr[i], baseAddr + i);
                ret = 0;
            }
        }
        else {
            if (ptr[i] != 0xAA55AA55) {
                printf("-E- Expected:0xAA55AA55, read %x @ %x \n\r" ,ptr[i], baseAddr + i);
                ret = 0;
            }
        }
    }
    TRACE_INFO("Test done!\n\r");
    return ret;
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Enable I and D cache */
    SCB_EnableICache();
    SCB_EnableDCache();

    /* Output example information */
    printf( "\n\r-- SDRAM Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    TRACE_INFO("Configuring External SDRAM \n\r");
    /* SDRAM timing configuration */
    BOARD_ConfigureSdram();

    /* Full test SDRAM  */
    TRACE_INFO("Starting memory validation of External SDRAM \n\r");

    if (_sdramAccess(SDRAM_CS_ADDR, 0x200000))
    {
      TRACE_INFO("Test succeeded!");
    }
    else
    {
      TRACE_INFO("Test failed!");
    }
    return 1;
}

