/* ----------------------------------------------------------------------------
 *         SAM Software Package License 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2010, Atmel Corporation
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

/** \file */

#ifndef USBHS_H
#define USBHS_H
/** addtogroup usbd_hal
 *@{
 */

#define  USB_DEVICE_HS_SUPPORT

//! Control endpoint size
#define  USB_DEVICE_EP_CTRL_SIZE       64

/** Indicates chip has an UDP High Speed. */
#define CHIP_USB_UDP

/** Indicates chip has an internal pull-up. */
#define CHIP_USB_PULLUP_INTERNAL

/** Number of USB endpoints */
#define CHIP_USB_NUMENDPOINTS   10

/** Endpoints max paxcket size */
#define CHIP_USB_ENDPOINTS_MAXPACKETSIZE(ep) \
   ((ep == 0) ? 64 : 1024)

/** Endpoints Number of Bank */
#define CHIP_USB_ENDPOINTS_BANKS(ep)            ((ep==0)?1:((ep<=2)?3:2))
     
     
#define CHIP_USB_ENDPOINTS_HBW(ep)              ((((ep)>=1) &&((ep)<=2))?true:false)

/** Endpoints DMA support */
#define CHIP_USB_ENDPOINTS_DMA(ep)              ((((ep)>=1)&&((ep)<=6))?true:false)
    
         
typedef enum
{
  HOST_MODE= 0,
  DEVICE_MODE=1
}USB_Mode_t;

/*---------------------------------------------------------------------------
 *      Internal Functions
 *---------------------------------------------------------------------------*/

     
 /*--------------------------------------------------------
 * =========== USB Global functions ======================
 *---------------------------------------------------------*/


/**
=================================  
        USBHS_CTRL 
=================================
**/

/**
 * \brief Freeze or unfreeze USB clock
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param Enable Enable or disable
 */
__STATIC_INLINE void USBHS_FreezeClock(Usbhs *pUsbhs, uint8_t Enable)
{
      
    if(Enable)
    {
      pUsbhs->USBHS_CTRL |= USBHS_CTRL_FRZCLK;
    }
    else
    {
      pUsbhs->USBHS_CTRL &= ~((uint32_t)USBHS_CTRL_FRZCLK);
    }
}


/**
 * \brief Enables or disables USB 
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param Enable Enable or disable
 */

__STATIC_INLINE void USBHS_UsbEnable(Usbhs *pUsbhs, uint8_t Enable)
{      
    if(Enable)
    {
      pUsbhs->USBHS_CTRL |= USBHS_CTRL_USBE;
    }
    else
    {
      pUsbhs->USBHS_CTRL &= ~((uint32_t)USBHS_CTRL_USBE);
    }
}


/**
 * \brief Device or Host Mode
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param Mode   Device or Host Mode
 */

__STATIC_INLINE void USBHS_UsbMode(Usbhs *pUsbhs, USB_Mode_t Mode)
{      
    if(Mode)
    {
      pUsbhs->USBHS_CTRL |= USBHS_CTRL_UIMOD_DEVICE;
    }
    else
    {
      pUsbhs->USBHS_CTRL &= ~((uint32_t)USBHS_CTRL_UIMOD_DEVICE);
    }
}



/********************* USBHS_SR  *****************/

/**
 * \brief Check if clock is usable or not
 * \param pUsbhs   Pointer to an USBHS instance.
 * \return 1 if USB clock is usable
 */

__STATIC_INLINE uint8_t USBHS_ISUsableClock(Usbhs *pUsbhs)
{ 
    return (( pUsbhs->USBHS_SR & USBHS_SR_CLKUSABLE) >> 14);
}


/**
 * \brief Raise interrupt for endpoint.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \return USB status
 */

__STATIC_INLINE uint32_t USBHS_ReadStatus(Usbhs *pUsbhs)
{   
    return (pUsbhs->USBHS_SR);   
}


/**
 * \brief Enable or disable USB address
 * \param pUsbhs   Pointer to an USBHS instance.
 * \return USB speed status
 */

__STATIC_INLINE uint32_t USBHS_GetUsbSpeed(Usbhs *pUsbhs)
{
    return ( (pUsbhs->USBHS_SR & USBHS_SR_SPEED_Msk) );
}

/********************* USBHS_SCR  *****************/

/**
 * \brief Raise interrupt for endpoint.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param AckType Interrupt Acknowledgetype
 */

__STATIC_INLINE void USBHS_Ack(Usbhs *pUsbhs, uint32_t AckType)
{   
    pUsbhs->USBHS_SCR |= AckType;
}


/********************* USBHS_SFR  *****************/

/**
 * \brief Raise interrupt for endpoint.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param SetStatus Set USB status
 */

__STATIC_INLINE void USBHS_Set(Usbhs *pUsbhs, uint32_t SetStatus)
{   
    pUsbhs->USBHS_SFR |= SetStatus;
}


     
 /*--------------------------------------------------------
 * =========== USB Device functions ======================
 *---------------------------------------------------------*/

/**
 * \brief Enable or disable USB address
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param SetStatus Set USB status
 */

__STATIC_INLINE void USBHS_EnableAddress(Usbhs *pUsbhs, uint8_t Enable)
{
    if(Enable)
    {
      pUsbhs->USBHS_DEVCTRL |= USBHS_DEVCTRL_ADDEN;
    }
    else
    {
      pUsbhs->USBHS_DEVCTRL &= ~((uint32_t)USBHS_DEVCTRL_ADDEN);
    }
}

/**
 * \brief Configure USB address and enable or disable it
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param Addr   USB device status
 */

__STATIC_INLINE void USBHS_SetAddress(Usbhs *pUsbhs, uint8_t Addr)
{  
    pUsbhs->USBHS_DEVCTRL = USBHS_DEVCTRL_UADD(Addr);
    pUsbhs->USBHS_DEVCTRL |= USBHS_DEVCTRL_ADDEN;
}

/**
 * \brief Get USB address
 * \param pUsbhs   Pointer to an USBHS instance.
 */

__STATIC_INLINE uint8_t USBHS_GetAddress(Usbhs *pUsbhs)
{  
    return ( pUsbhs->USBHS_DEVCTRL & USBHS_DEVCTRL_UADD_Msk);
}

/**
 * \brief Attach or detach USB.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param Enable Attachs or detach USB device
 */

__STATIC_INLINE void USBHS_DetachUsb(Usbhs *pUsbhs, uint8_t Enable)
{
    if(Enable)
    {
      pUsbhs->USBHS_DEVCTRL |= USBHS_DEVCTRL_DETACH;
    }
    else
    {
      pUsbhs->USBHS_DEVCTRL &= ~((uint32_t)USBHS_DEVCTRL_DETACH);
    }
  
}

/**
 * \brief Force Low Speed mode
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param Enable Enables the Full speed
 */

__STATIC_INLINE void USBHS_ForceLowSpeed(Usbhs *pUsbhs, uint8_t Enable)
{
    if(Enable)
    {
      pUsbhs->USBHS_DEVCTRL |= USBHS_DEVCTRL_LS;
    }
    else
    {
      pUsbhs->USBHS_DEVCTRL &= ~((uint32_t)USBHS_DEVCTRL_LS);
    }
  
}

/**
 * \brief Disable/Enables High Speed mode
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param Enable Enables/disable option
 */

__STATIC_INLINE void USBHS_EnableHighSpeed(Usbhs *pUsbhs, uint8_t Enable)
{
    if(Enable)
    {
      pUsbhs->USBHS_DEVCTRL |= USBHS_DEVCTRL_SPDCONF_NORMAL;
    }
    else
    {
      pUsbhs->USBHS_DEVCTRL |= USBHS_DEVCTRL_SPDCONF_FORCED_FS;
    }
  
}

/**
 * \brief Set Remote WakeUp mode
 * \param pUsbhs   Pointer to an USBHS instance.
 */

__STATIC_INLINE void USBHS_SetRemoteWakeUp(Usbhs *pUsbhs)
{
    pUsbhs->USBHS_DEVCTRL |= USBHS_DEVCTRL_RMWKUP;
}

/**
 * \brief Disable/Enables Test mode
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param mode Enables/disable option
 */

__STATIC_INLINE void USBHS_EnableTestMode(Usbhs *pUsbhs, uint32_t mode)
{
    pUsbhs->USBHS_DEVCTRL |= mode;
}


/**
 * \brief Disable/Enables HS Test mode
 * \param pUsbhs   Pointer to an USBHS instance.
 */

__STATIC_INLINE void USBHS_EnableHSTestMode(Usbhs *pUsbhs)
{
    pUsbhs->USBHS_DEVCTRL |= USBHS_DEVCTRL_SPDCONF_HIGH_SPEED;
}

/**
 * \brief Read status for an interrupt
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param IntType Interrupt type
 */

__STATIC_INLINE uint32_t USBHS_ReadIntStatus(Usbhs *pUsbhs, uint32_t IntType)
{   
    return (pUsbhs->USBHS_DEVISR & IntType);    
}

/**
 * \brief Read status for an Endpoint
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param EpNum  Endpoint
 */

__STATIC_INLINE uint32_t USBHS_ReadEpIntStatus(Usbhs *pUsbhs, uint8_t EpNum)
{   
    return (pUsbhs->USBHS_DEVISR & ( USBHS_DEVISR_PEP_0 << EpNum) );    
}

/**
 * \brief Read status for a DMA Endpoint
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param DmaNum  DMA Endpoint
 */
__STATIC_INLINE uint32_t USBHS_ReadDmaIntStatus(Usbhs *pUsbhs, uint8_t DmaNum)
{   
    return (pUsbhs->USBHS_DEVISR & ( USBHS_DEVISR_DMA_1 << DmaNum) );    
}

/**
 * \brief Acknowledge interrupt for endpoint.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param IntType Interrupt Type
 */

__STATIC_INLINE void USBHS_AckInt(Usbhs *pUsbhs, uint32_t IntType)
{   
    pUsbhs->USBHS_DEVICR |=  IntType;    
}

/**
 * \brief Raise interrupt for endpoint.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param IntType Interrupt Type
 */


__STATIC_INLINE void USBHS_RaiseInt(Usbhs *pUsbhs, uint32_t IntType)
{   
    pUsbhs->USBHS_DEVIFR |=  IntType;    
}

/**
 * \brief Raise DMA interrupt for endpoint.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param IntType Interrupt Type
 */
__STATIC_INLINE void USBHS_RaiseDmaInt(Usbhs *pUsbhs, uint8_t Dma)
{   
    assert(Dma< USBHSDEVDMA_NUMBER);
    pUsbhs->USBHS_DEVIFR |=  ( USBHS_DEVIFR_DMA_1 << Dma );    
}

/**
 * \brief check for interrupt of endpoint.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param IntType Interrupt Type
 */

__STATIC_INLINE uint32_t USBHS_IsIntEnable(Usbhs *pUsbhs, uint32_t IntType)
{
    return (pUsbhs->USBHS_DEVIMR &  IntType);    
}

/**
 * \brief Check if endpoint's interrupt is enabled for a given endpoint number
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param EpNum Endpoint number
 */

__STATIC_INLINE uint32_t USBHS_IsIntEnableEP(Usbhs *pUsbhs, uint8_t EpNum)
{
    return (pUsbhs->USBHS_DEVIMR &  (USBHS_DEVIMR_PEP_0 << EpNum ));    
}


/**
 * \brief Check if endpoint's DMA interrupt is enabled for a given endpoint DMA number
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param DmaNum Endpoint's DMA number
 */

__STATIC_INLINE uint32_t USBHS_IsDmaIntEnable(Usbhs *pUsbhs, uint8_t DmaNum)
{
    return (pUsbhs->USBHS_DEVIMR &  (USBHS_DEVIMR_DMA_1 << DmaNum));    
}


/**
 * \brief Enables Interrupt
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param IntType Interrupt Type
 */
__STATIC_INLINE void USBHS_EnableInt(Usbhs *pUsbhs, uint32_t IntType)
{   
    pUsbhs->USBHS_DEVIER |=  IntType;    
}

/**
 * \brief Enables interrupt for a given endpoint.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param DmaNum Endpoint's DMA number
 */
__STATIC_INLINE void USBHS_EnableIntEP(Usbhs *pUsbhs, uint8_t EpNum)
{   
    pUsbhs->USBHS_DEVIER |=  (USBHS_DEVIER_PEP_0 << EpNum);    
}

/**
 * \brief Enables DMA interrupt for a given endpoint.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param DmaEp  Endpoint's DMA interrupt number
 */

__STATIC_INLINE void USBHS_EnableDMAIntEP(Usbhs *pUsbhs, uint32_t DmaEp)
{   
    assert(DmaEp< USBHSDEVDMA_NUMBER);
    pUsbhs->USBHS_DEVIER |=  (USBHS_DEVIER_DMA_1 << DmaEp);      
}
  
 /**
 * \brief Disables interrupt for endpoint.
 * \param pUsbhs   Pointer to an USBHS instance.
 * \param IntType Int type
 */  

__STATIC_INLINE void USBHS_DisableInt(Usbhs *pUsbhs, uint32_t IntType)
{   
    pUsbhs->USBHS_DEVIDR |=  IntType;    
}

 /**
 * \brief Disables interrupt for endpoint.
 * \param pUsbhs  Pointer to an USBHS instance.
 * \param Ep    Endpoint number
 */ 

__STATIC_INLINE void USBHS_DisableIntEP(Usbhs *pUsbhs, uint8_t Ep)
{   
    pUsbhs->USBHS_DEVIDR |=  (USBHS_DEVIDR_PEP_0 << Ep);    
}

 /**
 * \brief Disables DMA interrupt for endpoint.
 * \param pUsbhs  Pointer to an USBHS instance.
 * \param DmaEp Endpoint's DMA number
 */
__STATIC_INLINE void USBHS_DisableDMAIntEP(Usbhs *pUsbhs, uint8_t DmaEp)
{   
    assert(DmaEp< USBHSDEVDMA_NUMBER);
    pUsbhs->USBHS_DEVIDR |=  (USBHS_DEVIDR_DMA_1 << DmaEp);    
}


 /**
 * \brief Enables or disables endpoint.
 * \param pUsbhs  Pointer to an USBHS instance.
 * \param Enable Enable/disable endpoint
 */

__STATIC_INLINE void USBHS_EnableEP(Usbhs *pUsbhs, uint8_t Ep, uint8_t Enable)
{
    if(Enable)
    {
      pUsbhs->USBHS_DEVEPT |=  (USBHS_DEVEPT_EPEN0 << Ep);
    }
    else
    {
      pUsbhs->USBHS_DEVEPT &=  ~(uint32_t)(USBHS_DEVEPT_EPEN0 << Ep);
    }
    
}


 /**
 * \brief Rests Endpoint
 * \param pUsbhs  Pointer to an USBHS instance.
 * \param Ep    Endpoint Number
 */

__STATIC_INLINE void USBHS_ResetEP(Usbhs *pUsbhs, uint8_t Ep)
{
   pUsbhs->USBHS_DEVEPT |=  (USBHS_DEVEPT_EPRST0 << Ep);
   
   pUsbhs->USBHS_DEVEPT &=  ~(uint32_t)(USBHS_DEVEPT_EPRST0 << Ep);
    
}

 /**
 * \brief Checks if Endpoint is enable
 * \param pUsbhs  Pointer to an USBHS instance.
 * \param Ep    Endpoint Number
 */

__STATIC_INLINE uint32_t USBHS_IsEPEnabled(Usbhs *pUsbhs, uint8_t Ep)
{
   return (pUsbhs->USBHS_DEVEPT & (USBHS_DEVEPT_EPEN0 << Ep) );
    
}


 /**
 * \brief Get MicrFrame number
 * \param pUsbhs  Pointer to an USBHS instance.
 * \retruns Micro frame number
 */
__STATIC_INLINE uint8_t USBHS_GetMicroFrameNum(Usbhs *pUsbhs)
{
    return (pUsbhs->USBHS_DEVFNUM & USBHS_DEVFNUM_MFNUM_Msk);    
}


 /**
 * \brief Get Frame number
 * \param pUsbhs  Pointer to an USBHS instance.
 * \retruns frame number
 */
__STATIC_INLINE uint8_t USBHS_GetFrameNum(Usbhs *pUsbhs)
{
    return ( (pUsbhs->USBHS_DEVFNUM & USBHS_DEVFNUM_FNUM_Msk) >> USBHS_DEVFNUM_FNUM_Pos);    
}

 /**
 * \brief Get Frame number CRC error
 * \param pUsbhs  Pointer to an USBHS instance.
 * \retruns Frame number error status
 */
__STATIC_INLINE uint8_t USBHS_GetFrameNumCrcErr(Usbhs *pUsbhs)
{
    return ( (pUsbhs->USBHS_DEVFNUM & USBHS_DEVFNUM_FNCERR) >> 15);    
}

 /*-----------------------------------------
 * =========== USB Device's Endpoint functions ========
 *------------------------------------------*/

/**
 * Set Endpoints configuration
 * Bank size, type and direction
 */
__STATIC_INLINE void USBHS_ConfigureEPs(Usbhs *pUsbhs, const uint8_t Ep,  const uint8_t Type, const uint8_t Dir, const uint8_t Size, const uint8_t Bank)
{  
    
    pUsbhs->USBHS_DEVEPTCFG[Ep] |=  ((Size << USBHS_DEVEPTCFG_EPSIZE_Pos) & USBHS_DEVEPTCFG_EPSIZE_Msk);
    pUsbhs->USBHS_DEVEPTCFG[Ep] |=  ((Dir << 8 ) & USBHS_DEVEPTCFG_EPDIR);
    pUsbhs->USBHS_DEVEPTCFG[Ep] |=  (( (Type) << USBHS_DEVEPTCFG_EPTYPE_Pos) & USBHS_DEVEPTCFG_EPTYPE_Msk);
    pUsbhs->USBHS_DEVEPTCFG[Ep] |=  (( (Bank) << USBHS_DEVEPTCFG_EPBK_Pos) & USBHS_DEVEPTCFG_EPBK_Msk);
}


/**
 * Enable or disable Auto switch of banks
 */
__STATIC_INLINE void USBHS_AutoSwitchBankEnable(Usbhs *pUsbhs, uint8_t Ep, uint8_t Enable)
{
  if(Enable)
  {
   pUsbhs->USBHS_DEVEPTCFG[Ep] |=USBHS_DEVEPTCFG_AUTOSW;
  }
  else
  {
   pUsbhs->USBHS_DEVEPTCFG[Ep] &= ~((uint32_t)USBHS_DEVEPTCFG_AUTOSW);
  }
}


/**
 * Allocate Endpoint memory
 */
__STATIC_INLINE void USBHS_AllocateMemory(Usbhs *pUsbhs, uint8_t Ep)
{  
    pUsbhs->USBHS_DEVEPTCFG[Ep] |=USBHS_DEVEPTCFG_ALLOC;
}


/**
 * Free allocated Endpoint memory
 */
__STATIC_INLINE void USBHS_FreeMemory(Usbhs *pUsbhs, uint8_t Ep)
{  
    pUsbhs->USBHS_DEVEPTCFG[Ep] &= ~((uint32_t)USBHS_DEVEPTCFG_ALLOC);
}


/**
 * Get Endpoint configuration
 */
__STATIC_INLINE uint32_t USBHS_GetConfigureEPs(Usbhs *pUsbhs, uint8_t Ep, uint32_t IntType)
{  
    return ((pUsbhs->USBHS_DEVEPTCFG[Ep] ) & IntType);
}

/**
 * Get Endpoint Type
 */
__STATIC_INLINE uint8_t USBHS_GetEpType(Usbhs *pUsbhs, uint8_t Ep)
{  
    return ((pUsbhs->USBHS_DEVEPTCFG[Ep] & USBHS_DEVEPTCFG_EPTYPE_Msk) >> USBHS_DEVEPTCFG_EPTYPE_Pos);
}

/**
 * Get Endpoint Size
 */
__STATIC_INLINE uint32_t USBHS_GetEpSize(Usbhs *pUsbhs, uint8_t Ep)
{  
    return ( 8 << ( (pUsbhs->USBHS_DEVEPTCFG[Ep] & USBHS_DEVEPTCFG_EPSIZE_Msk) >> USBHS_DEVEPTCFG_EPSIZE_Pos) );
}


/**
 * Sets ISO endpoint's Number of Transfer for High Speed
 */
__STATIC_INLINE void USBHS_SetIsoTrans(Usbhs *pUsbhs, uint8_t Ep, uint8_t nbTrans)
{  
    pUsbhs->USBHS_DEVEPTCFG[Ep] |= USBHS_DEVEPTCFG_NBTRANS(nbTrans) ;
}

/**
 * Check for interrupt types enabled for a given endpoint
 */
__STATIC_INLINE uint32_t USBHS_IsEpIntEnable(Usbhs *pUsbhs, uint8_t Ep, uint32_t EpIntType)
{  
    return (pUsbhs->USBHS_DEVEPTIMR[Ep] & EpIntType);
}


/**
 * Enables an interrupt type for a given endpoint
 */
__STATIC_INLINE void USBHS_EnableEPIntType(Usbhs *pUsbhs, uint8_t Ep, uint32_t EpInt)
{   
    pUsbhs->USBHS_DEVEPTIER[Ep] |=  EpInt;    
}

/**
 * Disables an interrupt type for a given endpoint
 */
__STATIC_INLINE void USBHS_DisableEPIntType(Usbhs *pUsbhs, uint8_t Ep, uint32_t EpInt)
{   
    pUsbhs->USBHS_DEVEPTIDR[Ep] |=  EpInt;    
}

/**
 * Clears register/acknowledge for a given endpoint
 */
__STATIC_INLINE void USBHS_AckEpInterrupt(Usbhs *pUsbhs, uint8_t Ep, uint32_t EpInt)
{   
    pUsbhs->USBHS_DEVEPTICR[Ep] |=  EpInt;    
}

/**
 * Sets/Raise register for a given endpoint
 */
__STATIC_INLINE void USBHS_RaiseEPInt(Usbhs *pUsbhs, uint8_t Ep, uint32_t EpInt)
{   
    pUsbhs->USBHS_DEVEPTIFR[Ep] |=  EpInt;    
}

/**
 * Gets interrupt status for a given EP
 */
__STATIC_INLINE uint32_t USBHS_ReadEPStatus(Usbhs *pUsbhs, uint8_t Ep, uint32_t EpInt)
{   
    return (pUsbhs->USBHS_DEVEPTISR[Ep] & EpInt);    
}

/**
 * Check if given endpoint's bank is free
 */
__STATIC_INLINE uint8_t USBHS_IsBankFree(Usbhs *pUsbhs, uint8_t Ep)
{
  if( (pUsbhs->USBHS_DEVEPTISR[Ep] & USBHS_DEVEPTISR_NBUSYBK_Msk) >> USBHS_DEVEPTISR_NBUSYBK_Pos)
  {
    return false;
  }
  else
  {
    return true;
  }
}

/**
 * Read endpoint's bank number in use
 */
__STATIC_INLINE uint8_t USBHS_NumOfBanksInUse(Usbhs *pUsbhs, uint8_t Ep)
{   
    return ( (pUsbhs->USBHS_DEVEPTISR[Ep] & USBHS_DEVEPTISR_NBUSYBK_Msk) >> USBHS_DEVEPTISR_NBUSYBK_Pos);
}


/**
 * Read endpoint's bank number in use
 */
__STATIC_INLINE uint16_t USBHS_ByteCount(Usbhs *pUsbhs, uint8_t Ep)
{   
    return (uint16_t)( (pUsbhs->USBHS_DEVEPTISR[Ep] & USBHS_DEVEPTISR_BYCT_Msk) >> USBHS_DEVEPTISR_BYCT_Pos);
}

 /*--------------------------------------------------------
 * =========== USB Device's Ep's DMA functions =========
 *---------------------------------------------------------*/

 /**
 * \brief Sets DMA next descriptor address
 * \param pUsbDma  USBHS device DMA instance
 * \param Desc NDA addrs
 */ 
__STATIC_INLINE void USBHS_SetDmaNDA(UsbhsDevdma *pUsbDma, uint32_t Desc)
{
  pUsbDma->USBHS_DEVDMANXTDSC = Desc;
}

 /**
 * \brief Gets DMA next descriptor address
 * \param pUsbDma  USBHS device DMA instance
 * \return Next DMA descriptor
 */ 
__STATIC_INLINE uint32_t USBHS_GetDmaNDA(UsbhsDevdma *pUsbDma)
{
   return (pUsbDma->USBHS_DEVDMANXTDSC);
}

 /**
 * \brief Sets USBHS's DMA Buffer addresse
 * \param pUsbDma  USBHS device DMA instance
 * \param Addr  DMA's buffer Addrs
 */ 
__STATIC_INLINE void USBHS_SetDmaBuffAdd(UsbhsDevdma *pUsbDma, uint32_t Addr)
{
  pUsbDma->USBHS_DEVDMAADDRESS = Addr;
}


 /**
 * \brief Gets USBHS's DMA Buffer addresse
 * \param pUsbDma  USBHS device DMA instance
 * \return DMA addrs
 */ 
__STATIC_INLINE uint32_t USBHS_GetDmaBuffAdd(UsbhsDevdma *pUsbDma)
{
   return (pUsbDma->USBHS_DEVDMAADDRESS);
}

 /**
 * \brief Setup the USBHS DMA
 * \param pUsbDma  USBHS device DMA instance
 * \param Cfg  DMA's configuration
 */ 
__STATIC_INLINE void USBHS_ConfigureDma(UsbhsDevdma *pUsbDma, uint32_t Cfg)
{
  pUsbDma->USBHS_DEVDMACONTROL |= Cfg;
}

 /**
 * \brief Get DMA configuration
 * \param pUsbDma  USBHS device DMA instance
 * \return DMA control setup
 */ 
__STATIC_INLINE uint32_t USBHS_GetDmaConfiguration(UsbhsDevdma *pUsbDma)
{
   return (pUsbDma->USBHS_DEVDMACONTROL);
}


 /**
 * \brief Set DMA status
 * \param pUsbDma  USBHS device DMA instance
 * \Status Set DMA status
 */ 
__STATIC_INLINE void USBHS_SetDmaStatus(UsbhsDevdma *pUsbDma, uint32_t Status)
{
  pUsbDma->USBHS_DEVDMASTATUS = Status;
}


 /**
 * \brief Get Dma Status
 * \param pUsbDma  USBHS device DMA instance
 * \return Dma status
 */ 
__STATIC_INLINE uint32_t USBHS_GetDmaStatus(UsbhsDevdma *pUsbDma)
{
   return (pUsbDma->USBHS_DEVDMASTATUS);
}


 /**
 * \brief Get DMA buffer's count
 * \param pUsbDma  USBHS device DMA instance
 * \return Buffer count
 */ 
__STATIC_INLINE uint16_t USBHS_GetDmaBuffCount(UsbhsDevdma *pUsbDma)
{
   return ( (pUsbDma->USBHS_DEVDMASTATUS & USBHS_DEVDMASTATUS_BUFF_COUNT_Msk) >> USBHS_DEVDMASTATUS_BUFF_COUNT_Pos);
}


 /*--------------------------------------------------------
 * =========== USB Host Functions  ========================
 *---------------------------------------------------------*/


 /**
 * \brief Sets USB host's speed
 * \param pUsbhs  USBHS host instance
 * \param UsbSpeed USB host speed
 */ 
__STATIC_INLINE void USBHS_SetHostSpeed(Usbhs *pUsbhs, uint16_t UsbSpeed)
{
   pUsbhs->USBHS_HSTCTRL |= UsbSpeed;
}

 /**
 * \brief Sets USB host's control
 * \param pUsbhs  USBHS host instance
 * \param cfg USB host control
 */ 
__STATIC_INLINE void USBHS_SetHostConfiguration(Usbhs *pUsbhs, uint32_t cfg)
{
   pUsbhs->USBHS_HSTCTRL |= cfg;
}

/**@}*/
#endif /* #ifndef USBHS_H */
