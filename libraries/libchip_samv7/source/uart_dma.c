/* ----------------------------------------------------------------------------
 *         SAM Software Package License 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, Atmel Corporation
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
 * \addtogroup uart_dma_module UART xDMA driver
 * \ingroup lib_uart
 * \section Usage
 *
 * <ul>
 * <li> UARTD_Configure() initializes and configures the UART peripheral and xDMA for data transfer.</li>
 * <li> Configures the parameters for the device corresponding to the cs value by UARTD_ConfigureCS(). </li>
 * <li> Starts a UART master transfer. This is a non blocking function UARTD_SendData(). It will
 * return as soon as the transfer is started..</li>
 * </ul>
 *
 */

/**
 * \file
 *
 * Implementation for the UART with xDMA driver.
 *
 */


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "chip.h"
#include "string.h"
#include "stdlib.h"
/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/


/** xDMA Link List size for uart transaction */
COMPILER_ALIGNED(4) static LinkedListDescriporView1 *pLLIviewRx = NULL;
COMPILER_ALIGNED(4) static LinkedListDescriporView1 *pLLIviewTx = NULL;

/*----------------------------------------------------------------------------
 *        Macros
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Local Variables
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/


/**
 * \brief UART xDMA Rx callback
 * Invoked on UART DMA reception done.
 * \param channel DMA channel.
 * \param pArg Pointer to callback argument - Pointer to UARTDma instance.   
 */ 
static void UARTD_Rx_Cb(uint32_t channel, UartDma* pArg)
{

    UartChannel *pUartdCh = pArg->pRxChannel;
    if (channel != pUartdCh->ChNum)
        return;

    /* Release the DMA channels */
    XDMAD_FreeChannel(pArg->pXdmad, pUartdCh->ChNum);
    pUartdCh->sempaphore = 1;
    memory_barrier();
}

/**
 * \brief USART xDMA Rx callback
 * Invoked on USART DMA reception done.
 * \param channel DMA channel.
 * \param pArg Pointer to callback argument - Pointer to USARTDma instance.   
 */ 
static void UARTD_Tx_Cb(uint32_t channel, UartDma* pArg)
{
    UartChannel *pUartdCh = pArg->pTxChannel;
    if (channel != pUartdCh->ChNum)
        return;

    /* Release the DMA channels */
    XDMAD_FreeChannel(pArg->pXdmad, pUartdCh->ChNum);
    pUartdCh->sempaphore = 1;
    memory_barrier();
}


/**
 * \brief Configure the UART Rx DMA mode.
 *
 * \param pUartHw   Pointer to UART instance
 * \param pXdmad    Pointer to XDMA instance
 * \param pUsartRx  Pointer to Usart Rx channel
 * \returns 0 if the dma multibuffer configuration successfully; otherwise returns
 * USARTD_ERROR_XXX.
 */
static uint8_t _configureUartRxDma(Uart *pUartHw, void *pXdmad, UartChannel *pUartRx)
{
    sXdmadCfg xdmadRxCfg;
    uint32_t xdmaCndc, xdmaInt;
    uint32_t uartId, LLI_Size;
    uint8_t *pBuff = 0, i;
    if ((unsigned int)pUartHw == (unsigned int)UART0 ) uartId = ID_UART0;
    if ((unsigned int)pUartHw == (unsigned int)UART1 ) uartId = ID_UART1;
    if ((unsigned int)pUartHw == (unsigned int)UART2 ) uartId = ID_UART2;
    if ((unsigned int)pUartHw == (unsigned int)UART3 ) uartId = ID_UART3;
    if ((unsigned int)pUartHw == (unsigned int)UART4 ) uartId = ID_UART4;

    /* Setup RX  */
    if(pUartRx->dmaProgrammingMode < XDMAD_LLI)         // if it is not LLI list
    {
      xdmadRxCfg.mbr_ubc = XDMA_UBC_NVIEW_NDV0 |
          XDMA_UBC_NDE_FETCH_DIS|
          XDMA_UBC_NDEN_UPDATED |
          pUartRx->BuffSize;
      xdmadRxCfg.mbr_da = (uint32_t)pUartRx->pBuff;

      xdmadRxCfg.mbr_sa = (uint32_t)&pUartHw->UART_RHR;
      xdmadRxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
          XDMAC_CC_MBSIZE_SINGLE |
          XDMAC_CC_DSYNC_PER2MEM |
          XDMAC_CC_CSIZE_CHK_1 |
          XDMAC_CC_DWIDTH_BYTE |
          XDMAC_CC_SIF_AHB_IF1 |
          XDMAC_CC_DIF_AHB_IF0 |
          XDMAC_CC_SAM_FIXED_AM |
          XDMAC_CC_DAM_INCREMENTED_AM |
          XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(  uartId, XDMAD_TRANSFER_RX ));

      xdmadRxCfg.mbr_bc = 0;
      xdmadRxCfg.mbr_sus = 0;
      xdmadRxCfg.mbr_dus =0;
      xdmaCndc = 0;
      
      /* Put all interrupts on for non LLI list setup of DMA */
      xdmaInt =  (XDMAC_CIE_BIE   |
                   XDMAC_CIE_DIE   |
                   XDMAC_CIE_FIE   |
                   XDMAC_CIE_RBIE  |
                   XDMAC_CIE_WBIE  |
                   XDMAC_CIE_ROIE);
    }
     
    if(pUartRx->dmaProgrammingMode == XDMAD_LLI)
    {
        LLI_Size = pUartRx->dmaLLI_Size;
        pBuff = pUartRx->pBuff;
        if(pLLIviewRx == NULL)
        {
          pLLIviewRx = malloc(sizeof(LinkedListDescriporView1)*LLI_Size);
        }
        xdmadRxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
                               XDMAC_CC_MBSIZE_SINGLE |
                               XDMAC_CC_DSYNC_PER2MEM |
                               XDMAC_CC_MEMSET_NORMAL_MODE |
                               XDMAC_CC_CSIZE_CHK_1 |
                               XDMAC_CC_DWIDTH_BYTE |
                               XDMAC_CC_SIF_AHB_IF1 |
                               XDMAC_CC_DIF_AHB_IF0 |
                               XDMAC_CC_SAM_FIXED_AM |
                               XDMAC_CC_DAM_INCREMENTED_AM |
                               XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(  uartId, XDMAD_TRANSFER_RX ));
        xdmadRxCfg.mbr_bc = 0;
        for (i = 0; i < LLI_Size; i++) {
             pLLIviewRx[i].mbr_ubc = XDMA_UBC_NVIEW_NDV1 |
                                   XDMA_UBC_NSEN_UNCHANGED | 
                                   XDMA_UBC_NDEN_UPDATED |
                                   ((i== LLI_Size- 1)? ( (pUartRx->dmaRingBuffer)? XDMA_UBC_NDE_FETCH_EN : 0):  XDMA_UBC_NDE_FETCH_EN) | pUartRx->BuffSize;
                pLLIviewRx[i].mbr_sa = (uint32_t)&pUartHw->UART_RHR;
                pLLIviewRx[i].mbr_da = (uint32_t)pBuff;
                pLLIviewRx[i].mbr_nda = (i == ( LLI_Size - 1))? ( (pUartRx->dmaRingBuffer)? (uint32_t)&pLLIviewRx[ 0] : 0 ):(uint32_t)&pLLIviewRx[ i + 1 ];
                pBuff += pUartRx->BuffSize;
            } 
        xdmaCndc = XDMAC_CNDC_NDVIEW_NDV1 | 
                   XDMAC_CNDC_NDE_DSCR_FETCH_EN |
                   XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED|
                   XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED ;
        
        xdmaInt = ((pUartRx->dmaRingBuffer)? XDMAC_CIE_BIE : XDMAC_CIE_LIE);
        
    }
    if (XDMAD_ConfigureTransfer( pXdmad, pUartRx->ChNum, &xdmadRxCfg, xdmaCndc, (uint32_t)&pLLIviewRx[0], xdmaInt))
         return USARTD_ERROR;

    return 0;
}


/**
 * \brief Configure the UART Tx DMA mode.
 *
 * \param pUartHw   Pointer to UART instance
 * \param pXdmad    Pointer to XDMA instance
 * \param pUsartTx  Pointer to Usart Tx channel
 * \returns 0 if the dma multibuffer configuration successfully; otherwise returns
 * USARTD_ERROR_XXX.
 */
static uint8_t _configureUartTxDma(Uart *pUartHw, void *pXdmad, UartChannel *pUartTx)
{
    sXdmadCfg xdmadTxCfg;
    uint32_t xdmaCndc, xdmaInt;
    uint32_t uartId, LLI_Size;
    uint8_t *pBuff = 0, i;
    if ((unsigned int)pUartHw == (unsigned int)UART0 ) uartId = ID_UART0;
    if ((unsigned int)pUartHw == (unsigned int)UART1 ) uartId = ID_UART1;
    if ((unsigned int)pUartHw == (unsigned int)UART2 ) uartId = ID_UART2;
    if ((unsigned int)pUartHw == (unsigned int)UART3 ) uartId = ID_UART3;
    if ((unsigned int)pUartHw == (unsigned int)UART4 ) uartId = ID_UART4;

    /* Setup TX  */ 
    if(pUartTx->dmaProgrammingMode < XDMAD_LLI)
    {
      xdmadTxCfg.mbr_ubc =    XDMA_UBC_NVIEW_NDV0 |
                              XDMA_UBC_NDE_FETCH_DIS|
                              XDMA_UBC_NSEN_UPDATED |  pUartTx->BuffSize;

      xdmadTxCfg.mbr_sa = (uint32_t)pUartTx->pBuff;
      xdmadTxCfg.mbr_da = (uint32_t)&pUartHw->UART_THR;
      xdmadTxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
          XDMAC_CC_MBSIZE_SINGLE |
          XDMAC_CC_DSYNC_MEM2PER |
          XDMAC_CC_CSIZE_CHK_1 |
          XDMAC_CC_DWIDTH_BYTE|
          XDMAC_CC_SIF_AHB_IF0 |
          XDMAC_CC_DIF_AHB_IF1 |
          XDMAC_CC_SAM_INCREMENTED_AM |
          XDMAC_CC_DAM_FIXED_AM |
          XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(  uartId, XDMAD_TRANSFER_TX ));

      xdmadTxCfg.mbr_bc = 0;
      xdmadTxCfg.mbr_sus = 0;
      xdmadTxCfg.mbr_dus =0;
      xdmaCndc = 0;

      xdmaInt =  (XDMAC_CIE_BIE   |
                   XDMAC_CIE_DIE   |
                   XDMAC_CIE_FIE   |
                   XDMAC_CIE_RBIE  |
                   XDMAC_CIE_WBIE  |
                   XDMAC_CIE_ROIE);
    }
    
    if(pUartTx->dmaProgrammingMode == XDMAD_LLI)
    {
        LLI_Size = pUartTx->dmaLLI_Size;
        pBuff = pUartTx->pBuff;
        if(pLLIviewTx == NULL)
        {
          pLLIviewTx = malloc(sizeof(LinkedListDescriporView1)*LLI_Size);
        }
        xdmadTxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
                               XDMAC_CC_MBSIZE_SINGLE |
                               XDMAC_CC_DSYNC_MEM2PER |
                               XDMAC_CC_MEMSET_NORMAL_MODE |
                               XDMAC_CC_CSIZE_CHK_1 |
                               XDMAC_CC_DWIDTH_BYTE |
                               XDMAC_CC_SIF_AHB_IF0 |
                               XDMAC_CC_DIF_AHB_IF1 |
                               XDMAC_CC_SAM_INCREMENTED_AM |
                               XDMAC_CC_DAM_FIXED_AM |
                               XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(  uartId, XDMAD_TRANSFER_TX ));
        xdmadTxCfg.mbr_bc = 0;
        for (i = 0; i < LLI_Size; i++) {
             pLLIviewTx[i].mbr_ubc = XDMA_UBC_NVIEW_NDV1 |
                                   XDMA_UBC_NSEN_UPDATED | 
                                   XDMA_UBC_NDEN_UNCHANGED |
                                   ((i== LLI_Size- 1)? ( (pUartTx->dmaRingBuffer)? XDMA_UBC_NDE_FETCH_EN : 0):  XDMA_UBC_NDE_FETCH_EN) | pUartTx->BuffSize;
                pLLIviewTx[i].mbr_da = (uint32_t)&pUartHw->UART_THR;
                pLLIviewTx[i].mbr_sa = (uint32_t)pBuff;
                pLLIviewTx[i].mbr_nda = (i == ( LLI_Size - 1))? ( (pUartTx->dmaRingBuffer)? (uint32_t)&pLLIviewTx[ 0] : 0 ):(uint32_t)&pLLIviewTx[ i + 1 ];
                pBuff += pUartTx->BuffSize;
            } 
        xdmaCndc = XDMAC_CNDC_NDVIEW_NDV1 | 
                   XDMAC_CNDC_NDE_DSCR_FETCH_EN |
                   XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED|
                   XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED ;
        
        xdmaInt = ((pUartTx->dmaRingBuffer)? XDMAC_CIE_BIE : XDMAC_CIE_LIE);       
    }
    if (XDMAD_ConfigureTransfer( pXdmad, pUartTx->ChNum, &xdmadTxCfg, xdmaCndc, (uint32_t)&pLLIviewTx[0], xdmaInt))
         return USARTD_ERROR;
    SCB_CleanInvalidateDCache();
    return 0;
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Initializes the UartDma structure and the corresponding UART & DMA hardware.
 * select value.
 * The driver will uses DMA channel 0 for RX and DMA channel 1 for TX.
 * The DMA channels are freed automatically when no UART command processing.
 *
 * \param pUartd    Pointer to a UartDma instance.
 * \param pUartHw   Associated UART peripheral.
 * \param uartId    UART peripheral identifier.
 * \param uartMode  UART peripheral identifier.*
 * \param baud      UART baud rate
 * \param clk       UART ref clock
 * \param pXdmad    Pointer to a Dmad instance. 
 */
uint32_t UARTD_Configure( UartDma *pUartd ,
        Uart *pUartHw ,
        uint8_t uartId,
        uint32_t uartMode,
        uint32_t baud,
        uint32_t clk,
        sXdmad *pXdmad )
{
    /* Initialize the UART structure */
    pUartd->pUartHw = pUartHw;
    pUartd->uartId  = uartId;
    pUartd->pXdmad = pXdmad;

    /* Enable the UART Peripheral ,Execute a software reset of the UART, Configure UART in Master Mode*/
    UART_Configure ( pUartHw, uartMode, baud, clk);
    
    /* Enables the USART to transfer data. */
    UART_SetTransmitterEnabled ( pUartHw , 1);
    
    /* Enables the USART to receive data. */
    UART_SetReceiverEnabled ( pUartHw , 1);

    /* Driver initialize */
    XDMAD_Initialize(  pUartd->pXdmad, 0 );
    /* Configure and enable interrupt on RC compare */ 
    NVIC_ClearPendingIRQ(XDMAC_IRQn);
    NVIC_SetPriority( XDMAC_IRQn ,1);
    
    
    
    return 0;
}


/**
 * \brief This function initialize the appropriate DMA channel for Rx channel of UART
 *
 * \param pUartd     Pointer to a UartDma instance.
 * \param pRxCh      Pointer to TxChannel configuration
 * \returns          0 if the transfer has been started successfully; otherwise returns
 * UARTD_ERROR_LOCK is the driver is in use, or UARTD_ERROR if the command is not
 * valid.
 */
uint32_t UARTD_EnableRxChannels( UartDma *pUartd, UartChannel *pRxCh)
{
    Uart *pUartHw = pUartd->pUartHw;
    uint32_t Channel;

    /* Init USART Rx Channel. */
    pUartd->pRxChannel = pRxCh;
        
    /* Allocate a DMA channel for UART RX. */
    Channel =  XDMAD_AllocateChannel( pUartd->pXdmad, pUartd->uartId, XDMAD_TRANSFER_MEMORY);
    if ( Channel == XDMAD_ALLOC_FAILED ) 
    {
        return UARTD_ERROR;
    }

    pRxCh->ChNum = Channel ;
    
     /* Setup callbacks for UART RX */
    if(pRxCh->callback)
    {
      XDMAD_SetCallback(pUartd->pXdmad, pRxCh->ChNum, (XdmadTransferCallback)pRxCh->callback, pRxCh->pArgument);
    }
    else
    {
      XDMAD_SetCallback(pUartd->pXdmad, pRxCh->ChNum, (XdmadTransferCallback)UARTD_Rx_Cb, pUartd);
    }
    
    if (XDMAD_PrepareChannel( pUartd->pXdmad, pRxCh->ChNum ))
        return UARTD_ERROR;

    if (_configureUartRxDma(pUartHw, pUartd->pXdmad, pRxCh))
        return UARTD_ERROR_LOCK;

    /* Check if DMA IRQ is enable; if not Enable it */
    if(!(NVIC_GetActive(XDMAC_IRQn)))
    {
      /* Enable interrupt  */ 
      NVIC_EnableIRQ(XDMAC_IRQn);  
    }
    
    return 0;
}


/**
 * \brief This function initialize the appropriate DMA channel for Tx channel of UART
 *
 * \param pUartd     Pointer to a UartDma instance.
 * \param pTxCh      Pointer to RxChannel configuration
 * \returns          0 if the transfer has been started successfully; otherwise returns
 * UARTD_ERROR_LOCK is the driver is in use, or UARTD_ERROR if the command is not
 * valid.
 */
uint32_t UARTD_EnableTxChannels( UartDma *pUartd, UartChannel *pTxCh)
{
    Uart *pUartHw = pUartd->pUartHw;
    uint32_t Channel;

    /* Init USART Tx Channel. */
    pUartd->pTxChannel = pTxCh;
    
    /* Allocate a DMA channel for UART TX. */
    Channel =  XDMAD_AllocateChannel( pUartd->pXdmad, XDMAD_TRANSFER_MEMORY, pUartd->uartId);
    if ( pTxCh->ChNum == XDMAD_ALLOC_FAILED ) 
    {
        return USARTD_ERROR;
    }

    pTxCh->ChNum = Channel ;

    /* Setup callbacks for UART TX */
    if(pUartd->pTxChannel->callback)
    {
      XDMAD_SetCallback(pUartd->pXdmad, pTxCh->ChNum, (XdmadTransferCallback)pTxCh->callback, pTxCh->pArgument);
    }
    else
    {
      XDMAD_SetCallback(pUartd->pXdmad, pTxCh->ChNum, (XdmadTransferCallback)UARTD_Tx_Cb, pUartd);
    }
    
    if ( XDMAD_PrepareChannel( pUartd->pXdmad, pTxCh->ChNum ))
        return USARTD_ERROR;

    if (_configureUartTxDma(pUartHw, pUartd->pXdmad, pTxCh))
        return USARTD_ERROR_LOCK;

    /* Check if DMA IRQ is enable; if not Enable it */
    if(!(NVIC_GetActive(XDMAC_IRQn)))
    {
      /* Enable interrupt  */ 
      NVIC_EnableIRQ(XDMAC_IRQn);  
    }
    
    return 0;
}

/**
 * \brief Starts a UART master transfer. This is a non blocking function. It will
 *  return as soon as the transfer is started.
 *
 * \param pUartd  Pointer to a UartDma instance.
 * \returns 0 if the transfer has been started successfully; otherwise returns
 * UARTD_ERROR_LOCK is the driver is in use, or UARTD_ERROR if the command is not
 * valid.
 */
uint32_t UARTD_SendData( UartDma *pUartd)
{

    /* Start DMA 0(RX) && 1(TX) */
    pUartd->pTxChannel->sempaphore=0;
    memory_barrier();
    if (XDMAD_StartTransfer( pUartd->pXdmad, pUartd->pTxChannel->ChNum )) 
        return USARTD_ERROR_LOCK;
    
    return 0;
}

/**
 * \brief Starts a UART master transfer. This is a non blocking function. It will
 *  return as soon as the transfer is started.
 *
 * \param pUartd  Pointer to a UartDma instance.
 * \returns 0 if the transfer has been started successfully; otherwise returns
 * UARTD_ERROR_LOCK is the driver is in use, or UARTD_ERROR if the command is not
 * valid.
 */
uint32_t UARTD_RcvData( UartDma *pUartd)
{    

    pUartd->pRxChannel->sempaphore=0;
    memory_barrier();
    /* Start DMA 0(RX) && 1(TX) */
    if (XDMAD_StartTransfer( pUartd->pXdmad, pUartd->pRxChannel->ChNum )) 
        return USARTD_ERROR_LOCK;
    
    return 0;
}
