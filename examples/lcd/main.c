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
 *  \page lcd LCD example
 *
 *  \section Purpose
 *
 *  This example demonstrates how to configure the LCD with SPI interface
 *
 *  \section Requirements
 *
 *  This package can be used with SAM V71 Xplained Ultra board with maXtouch xplained LCD board.
 *  LCD board must be set to 3-wire SPI configuration with the help of switch behind LCD.
 *  It should be in IM and IM2 should be in On position and IM1 should be in Off.
 *  Connect the LCD to EXT1 connectors, if connecting to EXT2 connectors please change the NPCS signal from 1 to 3
 *
 *  \section Description
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board. Please
 *     refer to the Getting Started with SAM V71 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- LCD Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 *  \section References
 *  - lcd/main.c
 *  - trace.h
 */

/** \file
 *
 *  This file contains all the specific code for the xplained LCD example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "image.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
const uint8_t HeadLineOffset = 25;

/** Image buffer (16-bits color). */
const uint32_t gImageBuffer[DEMO_IMAGE_HEIGHT * DEMO_IMAGE_WIDTH] = DEMO_IMAGE;


/**
 *  \brief LCD Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
    uint32_t i, j, HorizExtraX0, HorizExtraX1, VertExtraX0, VertExtraX1, HorizExtraY0, HorizExtraY1, VertExtraY0, VertExtraY1;
    int32_t dX, dY;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    SCB_EnableICache();
    SCB_EnableDCache();

    /* Output example information */
    printf( "-- LCD Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Configure systick for 1 ms. */
    TimeTick_Configure( );

    /* Initialize LCD and its interface */
    LCDD_Initialize();

    LCDD_DrawRectangleWithFill(0, 0, BOARD_LCD_WIDTH-1, BOARD_LCD_HEIGHT-1, RGB_24_TO_18BIT(COLOR_BLACK));
    LCD_DrawString(50, 5, "LCD maXtouch Example",  RGB_24_TO_18BIT(COLOR_WHITE));


    /* Test basic color space translation and LCD_DrawFilledRectangle */
    LCDD_DrawRectangleWithFill(0, HeadLineOffset, BOARD_LCD_WIDTH-1, BOARD_LCD_HEIGHT-1, RGB_24_TO_18BIT(COLOR_WHITE));


    LCDD_DrawRectangleWithFill(BOARD_LCD_WIDTH-5, BOARD_LCD_HEIGHT-5, 4, 4 + HeadLineOffset, RGB_24_TO_18BIT(COLOR_BLACK));

    LCDD_DrawRectangleWithFill(8, 8+HeadLineOffset, BOARD_LCD_WIDTH-9, BOARD_LCD_HEIGHT-9, RGB_24_TO_18BIT(COLOR_BLUE));

    LCDD_DrawRectangleWithFill(12, 12+HeadLineOffset, BOARD_LCD_WIDTH-13, BOARD_LCD_HEIGHT-13, RGB_24_TO_18BIT(COLOR_RED));

    LCDD_DrawRectangleWithFill(16, 14+HeadLineOffset, BOARD_LCD_WIDTH-17, BOARD_LCD_HEIGHT-17, RGB_24_TO_18BIT(COLOR_GREEN));

    /* Test horizontal/vertical LCD_drawLine  */
    LCDD_DrawLine(0, BOARD_LCD_HEIGHT/2, BOARD_LCD_WIDTH-1, BOARD_LCD_HEIGHT/2, RGB_24_TO_18BIT(COLOR_RED));
    LCDD_DrawLine(BOARD_LCD_WIDTH/2, 0 + HeadLineOffset , BOARD_LCD_WIDTH/2, BOARD_LCD_HEIGHT-1, RGB_24_TO_18BIT(COLOR_RED));

    /* Test Bresenham LCD_drawLine  */
    LCDD_DrawLine(0, 0 , BOARD_LCD_WIDTH-1, BOARD_LCD_HEIGHT-1, RGB_24_TO_18BIT(COLOR_RED));
    LCDD_DrawLine(0, BOARD_LCD_HEIGHT-1, BOARD_LCD_WIDTH-1, 0, RGB_24_TO_18BIT(COLOR_RED));

    /* Test LCD_DrawRectangle */
    LCDD_DrawRectangle(BOARD_LCD_WIDTH/4, BOARD_LCD_HEIGHT/4, BOARD_LCD_WIDTH*3/4, BOARD_LCD_HEIGHT*3/4, RGB_24_TO_18BIT(COLOR_RED));
    LCDD_DrawRectangle(BOARD_LCD_WIDTH*2/3, BOARD_LCD_HEIGHT*2/3, BOARD_LCD_WIDTH/3, BOARD_LCD_HEIGHT/3, RGB_24_TO_18BIT(COLOR_RED));

    /* Test LCD_DrawFilledCircle */
    LCD_DrawFilledCircle(BOARD_LCD_WIDTH*3/4, BOARD_LCD_HEIGHT*3/4, BOARD_LCD_WIDTH/4, RGB_24_TO_18BIT(COLOR_BLUE));
    LCD_DrawFilledCircle(BOARD_LCD_WIDTH/2, BOARD_LCD_HEIGHT/2, BOARD_LCD_HEIGHT/4, RGB_24_TO_18BIT(COLOR_GREEN));

    LCD_DrawFilledCircle(BOARD_LCD_WIDTH/4, BOARD_LCD_HEIGHT*3/4, BOARD_LCD_HEIGHT/4, RGB_24_TO_18BIT(COLOR_RED));
    LCD_DrawFilledCircle(BOARD_LCD_WIDTH*3/4, BOARD_LCD_HEIGHT/4, BOARD_LCD_WIDTH/4, RGB_24_TO_18BIT(COLOR_YELLOW));


    /* Test LCD_DrawPicture */
    LCDD_DrawImage(50, 50, (LcdColor_t *)gImageBuffer ,  (50 + DEMO_IMAGE_WIDTH-1), (50 + DEMO_IMAGE_HEIGHT-1));

    LCDD_DrawRectangleWithFill(0, 0, BOARD_LCD_WIDTH-1, BOARD_LCD_HEIGHT-1, RGB_24_TO_18BIT(COLOR_BLACK));

    /** Move picture across the screen */
    dX=2; dY=2;j=0; i=0;
    HorizExtraX0 = 0; HorizExtraY0 =0; HorizExtraX1 = DEMO_IMAGE_WIDTH; HorizExtraY1 =DEMO_IMAGE_HEIGHT;
    VertExtraX0 = 0; VertExtraY0 =0; VertExtraX1 = DEMO_IMAGE_WIDTH; VertExtraY1 =DEMO_IMAGE_HEIGHT;
    for(; ;)
    {
        for(; i<(BOARD_LCD_WIDTH-DEMO_IMAGE_WIDTH-1);)
        {

            LCDD_DrawImage(i, j, (LcdColor_t *)gImageBuffer, (i + DEMO_IMAGE_WIDTH-1), (j + DEMO_IMAGE_HEIGHT-1));
            LCDD_DrawRectangleWithFill(HorizExtraX0, HorizExtraY0, HorizExtraX1, HorizExtraY1, RGB_24_TO_18BIT(COLOR_BLACK));
            LCDD_DrawRectangleWithFill(VertExtraX0,  VertExtraY0,   VertExtraX1, VertExtraY1, RGB_24_TO_18BIT(COLOR_BLACK)); 
            j +=dY;
            i +=dX;

            if(dY > 0)
            {
                HorizExtraX0 = i-dX; HorizExtraY0 = j-dY; HorizExtraX1 = (HorizExtraX0 + DEMO_IMAGE_WIDTH); HorizExtraY1 = j; 
            }
            else
            {
                HorizExtraX0 = i-dX; HorizExtraY0 = (j-dY + DEMO_IMAGE_HEIGHT); HorizExtraX1 = (HorizExtraX0 + DEMO_IMAGE_WIDTH); HorizExtraY1 = (j + DEMO_IMAGE_HEIGHT); 
            }

            if(dX > 0)
            {
                VertExtraX0 = i-dX; VertExtraY0 = j-dY;   VertExtraX1 = i; VertExtraY1 = (VertExtraY0 + DEMO_IMAGE_HEIGHT);
            }
            else
            {
                VertExtraX0 = (i-dX + DEMO_IMAGE_WIDTH); VertExtraY0 = j-dY;   VertExtraX1 = (i+ DEMO_IMAGE_WIDTH); VertExtraY1 = (VertExtraY0 + DEMO_IMAGE_HEIGHT);
            }

            if((i >= (BOARD_LCD_WIDTH-DEMO_IMAGE_WIDTH-1)) && (j < (BOARD_LCD_HEIGHT-DEMO_IMAGE_HEIGHT-1)))
            {
                i -=dX;dX=-dX;
            }
            else if((i < (BOARD_LCD_WIDTH-DEMO_IMAGE_WIDTH-1)) && (j >= (BOARD_LCD_HEIGHT-DEMO_IMAGE_HEIGHT-1)))
            {
                j -=dY;
                dY=-dY;
            }

        }
        i=0;
        j=0;
        LCDD_DrawRectangleWithFill(0, 0, BOARD_LCD_WIDTH-1, BOARD_LCD_HEIGHT-1, RGB_24_TO_18BIT(COLOR_BLACK));
    }
}
