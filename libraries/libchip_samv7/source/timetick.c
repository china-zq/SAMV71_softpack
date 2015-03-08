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
 *  \file
 *  Implement the System Timer.
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/** Tick Counter united by ms */
static volatile uint32_t _dwTickCount = 0 ;
static volatile uint32_t _dwTickTimer = 0 ;
static uint8_t SysTickConfigured = 0 ;

/*----------------------------------------------------------------------------
 *         Exported Functions
 *----------------------------------------------------------------------------*/


/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 *  Increments the timestamp counter.
 */
void SysTick_Handler( void )
{
    _dwTickCount ++;
    if(_dwTickTimer)
      _dwTickTimer --;
}


/**
 *  \brief Configures the System Timer.
 *  Systick interrupt handler will generates 1ms interrupt and increase a
 *  tickCount.
 *  \note IRQ handler must be configured before invoking this function.
 */
uint32_t TimeTick_Configure( void )
{
    uint8_t Mdiv_Val;
    uint32_t Pck;
    _dwTickCount = 0 ;

    TRACE_INFO( "Configure system tick to get 1ms tick period.\n\r" ) ;
    /* check if there is MDIV value */
    Mdiv_Val = ( (PMC->PMC_MCKR & PMC_MCKR_MDIV_Msk) >> PMC_MCKR_MDIV_Pos);

    if(Mdiv_Val ==0)
    {
      Pck = BOARD_MCK;
    }
    if(Mdiv_Val ==3)
    {
      Pck = BOARD_MCK * Mdiv_Val;
    }
    else
    {
      Pck = BOARD_MCK * (Mdiv_Val*2);
    }

    /* Configure SysTick for 1 ms. */
    if ( SysTick_Config( Pck/1000 ) )
    {
        TRACE_ERROR("SysTick configuration error\n\r" ) ;
        SysTickConfigured = 0;
        return 1;
    }
    SysTickConfigured = 1;
    return 0;
}

/**
 * \brief Get Delayed number of tick
 * \param startTick Start tick point.
 * \param endTick   End tick point.
 */
uint32_t GetDelayInTicks(uint32_t startTick, uint32_t endTick)
{
  
    if(!SysTickConfigured)
      TimeTick_Configure();
    if (endTick >= startTick) return (endTick - startTick);
    return (endTick + (0xFFFFFFFF - startTick) + 1);
    
}

/**
 *  \brief Get current Tick Count, in ms.
 */
uint32_t GetTickCount( void )
{
    if(!SysTickConfigured)
      TimeTick_Configure();
    
    return _dwTickTimer ;
   
}

/**
 *  \brief Set an initial value for Tick Count, in ms.
 *  \param Timeout    The initial value for Tick Count.
 */
void SetTickCount( uint32_t Timeout )
{
  if(!SysTickConfigured)
      TimeTick_Configure();
  
  _dwTickTimer = Timeout ;
  
}

/**
 *  \brief Sync Wait for several ms
 *  \param dwMs    Waiting time in ms.
 */
void Wait( volatile uint32_t dwMs )
{
    uint32_t dwStart ;
    uint32_t dwCurrent ;

    if(!SysTickConfigured)
      TimeTick_Configure();
    
    dwStart = _dwTickCount ;
    do
    {
      dwCurrent = _dwTickCount ;
    } while ( dwCurrent - dwStart < dwMs ) ;
    
}

/**
 *  \brief Sync Sleep for several ms
 *  \param dwMs    Sleeping time in ms.
 */
void Sleep( volatile uint32_t dwMs )
{
    uint32_t dwStart ;
    uint32_t dwCurrent ;
     
    if(!SysTickConfigured)
      TimeTick_Configure();
   
    __ASM("CPSIE   I");   
    dwStart = _dwTickCount ;
    do
    {
        dwCurrent = _dwTickCount ;

        if ( dwCurrent - dwStart > dwMs )
        {
            break ;
        }
        __ASM("WFI");
    } while( 1 ) ;
    
}

