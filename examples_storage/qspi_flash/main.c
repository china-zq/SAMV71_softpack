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
 * \page qspi_flash QSPI with Serialflash Example
 *
 * \section Purpose
 *
 * This example demonstrates how to setup the QSPI in order to initialize, read
 * and write a serial dataflash.
 *
 * \section Requirements
 *
 * This package can be used with SAM V71 Xplained Ultra board.
 *
 * \section Description
 *
 * The demonstration program tests the serial dataflash present on the
 * evaluation kit by erasing and writing each one of its pages. It also gives
 * read/write bandwidth by the test.
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board. Please
 *     refer to the Getting Started with SAM V71 Microcontrollers.pdf
 * -# Optionally, on the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# Upon startup, the application will output the following lines on the terminal window:
 *    \code
 *    -- QSPI Serialflash Example xxx --
 *    -- SAMxxxxx-xx
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    QSPI drivers initialized
 *    \endcode
 * -# The program will connect to the serial firmware dataflash through the QSPI
 *    and start sending commands to it. It will perform the following:
 *    - Read the JEDEC identifier of the device to auto detect it
 *      The next line should indicate if the serial dataflash has been
 *      correctly identified. 
 * \section References
 * - qspi_flash/main.c
 * - qspi.c
 * - s25fl1.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the spi_serialflash example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <board.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "stdlib.h"
/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Polling or interrupt mode */
#define POLLING_MODE   0

/** Maximum device page size in bytes. */
#define MAXPAGESIZE     256

/** SPI peripheral pins to configure to access the serial flash. */
#define QSPI_PINS        PINS_QSPI

#define BUFFER_SIZE     512
/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Global DMA driver instance for all DMA transfers in application. */
static sXdmad xDmad;

/** Pins to configure for the application. */
static Pin pins[] = QSPI_PINS;

/** buffer for test QSPI flash */
static uint8_t Buffer[BUFFER_SIZE], TestBuffer[BUFFER_SIZE];

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

#if POLLING_MODE == 0
/**
 * ISR for DMA interrupt
 */
void XDMAC_Handler(void)
{
    XDMAD_Handler(&xDmad);
}

#endif


void QSPI_Handler(void)
{  
    printf("QSPI handler \n\r");
    if(QSPI_GetStatus(QSPI) & QSPI_SR_INSTRE)
    {
        INSTRE_Flag = 1;
        printf("INSTR Done \n\r");
    }
}

static void _fillupbuffer(uint8_t *pBuff, uint32_t size)
{
    int i;
    for(i=0; i<size;)        // one page
    {
        pBuff[i++] = 0xAA; 
        pBuff[i++] = 0x55; 
        pBuff[i++] = 0x03; 
        pBuff[i++] = 0x0c;
        pBuff[i++] = 0x11; 
        pBuff[i++] = 0x00; 
        pBuff[i++] = 0x22; 
        pBuff[i++] = 0x44;
    }
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for SPI with Serialflash example.
 * Initializes the serial flash and performs several tests on it.
 *
 * \return Unused (ANSI-C compatibility).
 */

int main(void)
{
    uint32_t i,j, Fault;
    uint32_t deviceId;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- QSPI Serialflash Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;
    SCB_EnableICache();        
    SCB_EnableDCache();
    TimeTick_Configure();

    /* Initialize the SPI and serial flash */
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    ENABLE_PERIPHERAL(ID_QSPI);
    QSPI_Configure(QSPI, (QSPI_MR_SMM_MEMORY | QSPI_MR_CSMODE_SYSTEMATICALLY | QSPI_MR_DLYCS((uint32_t)1000) ));
    QSPI_ConfigureClock(QSPI, (QSPI_SCR_SCBR(1) | QSPI_SCR_DLYBS(1000)) );

    QSPI_Enable(QSPI); 

    S25FL1D_InitFlashInterface();
    printf("QSPI drivers initialized\n\r");


    while(1)
    {
        deviceId = S25FL1D_ReadJedecId();
        printf("ID read: Manufacture ID = 0x%x, Device Type = 0x%x, Capacity = 0x%x\n\r",
                   (uint8_t)(deviceId>>16), (uint8_t)(deviceId>>8), (uint8_t)(deviceId>>0));
        break;
    }
    /* erase entire chip  */
    S25FL1D_EraseChip();

    /* enable quad mode */
    S25FL1D_EnableQuadMode();

    /* fill up the buffer*/
    _fillupbuffer(TestBuffer, BUFFER_SIZE);

    printf("Writing buffer to Flash memory...... \n\r");

    /* write the buffer to flash mem */
    for(i=0; i<0x200000;)              
    {
        S25FL1D_Write(TestBuffer, BUFFER_SIZE, i);  
        i += BUFFER_SIZE;
    }

    printf("Writing flash memory...... \n\r");

    printf("Reading buffer from Flash memory using QuadSPI output...... \n\r");
    /* read and compare data from flash memory */
    for(i=0; i<0x200000;)
    {
        S25FL1D_ReadQuadIO(Buffer, BUFFER_SIZE, i, 0);
        for( j=0; j< BUFFER_SIZE; j++)
        {
            if(Buffer[j] != TestBuffer[j])
            {       
                if(Fault==0)
                    printf("\n\rData does not match @ 0x%x ", i);
                Fault++;
            }
            else
            {
                if(Fault > 1)
                    printf("upto 0x%x \r\n", i);
                Fault = 0;
            }
        }
        i +=BUFFER_SIZE;
    }

    printf("Erasing a block(64 KB) @ Add 0x10000 \n\r");
    S25FL1D_Erase64KBlock(0x10000);
    printf("Verify the block(64 KB) @ Add 0x10000 \n\r");
    for(i=0x10000; i<0x10000 + 64*1024;)
    {
        S25FL1D_ReadQuadIO(Buffer, BUFFER_SIZE, i, 0);
        for( j=0; j< BUFFER_SIZE; j++)
        {
            if(Buffer[j] != 0xFF)
            {
                if(Fault<1)
                    printf("\n\rData does not match @ 0x%x ", i);
                Fault++;
            }
            else
            {
                if(Fault > 1)
                    printf("upto 0x%x \r\n", i);
                Fault = 0;
            }
        }
        i +=BUFFER_SIZE;
    }

    printf("Test Completed \n\r");

    S25FL1D_ReadQuadIO(Buffer, BUFFER_SIZE, 0, 0);

    while(1);

}
