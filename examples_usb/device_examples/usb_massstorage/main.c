/*----------------------------------------------------------------------------
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
/** \cond usb_massstorage
 * \page usb_massstorage USB Device Mass Storage Example
 *
 * \section Purpose
 *
 * The USB Mass storage Example will help you to get familiar with the
 * USB Device Port(UDP) on Samv7 Microcontrollers. Also
 * it can help you to be familiar with the USB Framework that is used for
 * rapid development of USB-compliant class drivers such as USB Mass
 * Storage class (MSD).
 *
 * \section Requirements
 *
 * This package can be used with all Atmel Xplained board that have USB interface
 *
 * \section Description
 *
 * The demo simulates a SD/MMC USB disk.
 *
 * When the board running this program connected to a host (PC for example), with
 * USB cable, the board appears as a USB Disk for the host. Then the host can
 * format/read/write on the disk.
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board. Please
 *     refer to the Getting Started with SAM V71 Microcontrollers.pdf
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *     \code
 *     -- USB Device Mass Storage Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the host
 *    reports a new USB %device attachment and Disk installation.
 *  . Then new "USB Mass Storage Device" and
 *    "ATMEL Mass Storage MSD USB Device" and "Generic volume" appear in
 *    hardware %device list.
 * -# You can find the new disk on host, and to create/write file to it.
 *
 * \section References
 * - usb_massstorage/main.c
 * - pio.h
 * - pio_it.h
 * - memories: Storage Media interface for MSD
 * - usb: USB Framework, USB MSD driver and UDP interface driver
 *    - \ref usbd_framework
 *       - \ref usbd_api
 *    - \ref usbd_msd
 *       - \ref usbd_msd_drv
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_massstorage.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "libstoragemedia.h"
#include "libsdmmc.h"

#include "MSDDriver.h"
#include "MSDLun.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Compiling Options
 *----------------------------------------------------------------------------*/

/* No transfer speed dump */
#define DBG_SPEED_OFF

/* LUN using SDcard */
#define LUN_SDCARD

//#define LUN_RAMDISK

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Maximum number of LUNs which can be defined. */
#define MAX_LUNS            1

/** Media index for different disks */
#define DRV_SDMMC           0    /**< SD card */
#define DRV_RAMDISK         1    /**< RAM disk */
#define DRV_NAND            2    /**< Nand flash */


/** RamDisk address base (in bytes) */
#define RAMDISK_BASE        (SDRAM_CS_ADDR)

/** RamDisk size (in bytes) */
/** RamDisk size: 20K (WinXP can not format the disk if lower than 20K) */
#define RAMDISK_SIZE        (10*1024*1024)

/** Size of the reserved DDRAM (32M) */
#define DDRAM_RESERVE_SIZE  (1*1024*1024)

/** Delay for MSD refresh (*4ms) */
#define MSD_REFRESH_DELAY    250

/** Delay for display view update (*250ms) */
#define UPDATE_DELAY        4

/** Size of one block in bytes. */
#define BLOCK_SIZE          512

/** Size of the MSD IO buffer in bytes (250K, more the better). */
#define MSD_BUFFER_SIZE     ( 500 * BLOCK_SIZE  )


#if defined sram
/* Define Buffer address */
#if defined   ( __CC_ARM   )
    #define BUF_START_ADDR    (0x20410000)
#elif defined ( __ICCARM__ )
    #define BUF_START_ADDR    (0x20410000)
#elif defined (  __GNUC__  )
    #define BUF_START_ADDR    (0x20420000)
#endif
#endif

#if defined flash
/* Define Buffer address */
#if defined   ( __CC_ARM   )
    #define BUF_START_ADDR     (0x20401800)
#elif defined ( __ICCARM__ )
    #define BUF_START_ADDR     (0x20404000)
#elif defined (  __GNUC__  )
    #define BUF_START_ADDR     (0x20404000)
#endif
#endif


/*----------------------------------------------------------------------------
 *        Global variables
 *----------------------------------------------------------------------------*/

/** MSD Driver Descriptors List */
extern const USBDDriverDescriptors msdDriverDescriptors;

/** SD card pins instance. */
static const Pin pinsSd[] = {BOARD_MCI_PINS_SLOTA, BOARD_MCI_PIN_CK};

/** SD card detection pin instance. */
static const Pin pinsCd[] = {BOARD_MCI_PIN_CD};

/** Available media. */
sMedia medias[MAX_LUNS];

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** DMA driver instance */
static sXdmad dmaDrv;

/** Device LUNs. */
MSDLun luns[MAX_LUNS];

/** SDCard driver instance. */
static sSdCard sdDrv[BOARD_NUM_MCI];

/** MCI driver instance. */
/** SDCard driver instance. */
static sMcid mciDrv[BOARD_NUM_MCI];

/** LUN read/write buffer. */
uint8_t * msdBuffer;

/** Total data write to disk */
uint32_t msdWriteTotal = 0;

/** Delay TO event */
uint8_t  msdRefresh = 0;


/**
 * XDMA0 interrupt handler.
 */
void XDMAC_Handler(void)
{
    XDMAD_Handler(&dmaDrv);
}

/**
 * MCI interrupt handler. Forwards the event to the MCI driver handlers.
 */
void HSMCI_Handler(void)
{
    uint32_t i;
    for (i = 0; i < BOARD_NUM_MCI; i ++)
    {
        MCID_Handler(&mciDrv[i]);
    }
}

/*-----------------------------------------------------------------------------
 *         Callback re-implementation
 *-----------------------------------------------------------------------------*/

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
    MSDDriver_RequestHandler(request);
}

/**
 * Invoked when the configuration of the device changes. Resets the mass
 * storage driver.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(unsigned char cfgnum)
{
    MSDDriver_ConfigurationChangeHandler(cfgnum);
}

/*----------------------------------------------------------------------------
 *        Callbacks
 *----------------------------------------------------------------------------*/
/**
 * Invoked when the MSD finish a READ/WRITE.
 * \param flowDirection 1 - device to host (READ10)
 *                      0 - host to device (WRITE10)
 * \param dataLength Length of data transferred in bytes.
 * \param fifoNullCount Times that FIFO is NULL to wait
 * \param fifoFullCount Times that FIFO is filled to wait
 */
static void MSDCallbacks_Data(uint8_t flowDirection,
                       uint32_t  dataLength,
                       uint32_t  fifoNullCount,
                       uint32_t  fifoFullCount)
{
    fifoNullCount = fifoNullCount; /* dummy */
    fifoFullCount = fifoFullCount;  /*dummy */
    if (!flowDirection)
    {
        msdWriteTotal += dataLength;
    }
}
/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * Configure for SD detect pin
 */
static void CardDetectConfigure(void)
{
    PIO_Configure(pinsCd, PIO_LISTSIZE(pinsCd));
    /* No protection detect pin */
}

/**
 * Check if the card is connected.
 * \param iMci Controller number.
 * Return 1 if card is inserted.
 */
static uint8_t CardIsConnected(uint8_t iMci)
{
    return PIO_Get(&pinsCd[iMci]) ? 0 : 1;
}

/**
 * Run init on the inserted card
 * \param iMci Controller number.
 */
static void CardInit(uint8_t iMci)
{
    sSdCard *pSd = &sdDrv[iMci];
    uint8_t error;
    uint8_t retry = 2;
    while(retry --)
    {
        error = SD_Init(pSd);
        if (error == SDMMC_OK) break;
    }
    if (error)
    {
        TRACE_ERROR("SD/MMC card initialization failed: %d\n\r", error);
        return;
    }
    TRACE_INFO(" SD/MMC card initialization successful\n\r");
    if (SD_GetCardType(pSd) & CARD_TYPE_bmSDMMC)
    {
        TRACE_INFO(" MEM Card OK, size: %d MB", (int)SD_GetTotalSizeKB(pSd)/1000);
        TRACE_INFO(", %d * %dB\n\r", (int)SD_GetNumberBlocks(pSd), (int)SD_GetBlockSize(pSd));
    }
    if (SD_GetCardType(pSd) & CARD_TYPE_bmSDIO)
    {
        TRACE_ERROR("-E- IO Card Detected \n\r");
    }
}

/**
 * Initialize PIOs
 */
static void _ConfigurePIOs(void)
{
    /* Configure SDcard pins */
    PIO_Configure(pinsSd, PIO_LISTSIZE(pinsSd));
    /* Configure SD card detection */
    CardDetectConfigure();
}

/**
 * Initialize driver instances.
 */
static void _ConfigureDrivers(void)
{
     uint32_t i;
    /* Initialize the DMA driver */
    XDMAD_Initialize(&dmaDrv,0);

    NVIC_EnableIRQ( XDMAC_IRQn );

    /* Initialize the HSMCI driver */
    MCID_Init(&mciDrv[0], HSMCI, ID_HSMCI, BOARD_MCK, &dmaDrv, 0 ) ;

    NVIC_EnableIRQ( HSMCI_IRQn );

    /* Initialize SD driver */
    for (i = 0; i < BOARD_NUM_MCI; i ++)
    {
        SDD_InitializeSdmmcMode(&sdDrv[i], &mciDrv[i], 0);
    }
}


/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/
#if defined LUN_RAMDISK
/**
 * Initialize DDRAM to assign RamDisk block
 */
static void RamDiskInit(void)
{

    TRACE_INFO("RamDisk @ %x, size %d\n\r", (SDRAM_CS_ADDR ), RAMDISK_SIZE);
    msdBuffer = (uint8_t *)BUF_START_ADDR;
    MEDRamDisk_Initialize(&(medias[DRV_RAMDISK]),
                           BLOCK_SIZE,
                           (SDRAM_CS_ADDR) / BLOCK_SIZE,
                           RAMDISK_SIZE / BLOCK_SIZE,
                           1);
    LUN_Init(&(luns[0]),
             &(medias[0]),
             msdBuffer, MSD_BUFFER_SIZE,
             0, 0, 0, 0,
             MSDCallbacks_Data);
}
#endif

#if defined LUN_SDCARD
static void SDDiskInit(sSdCard *pSd)
{
    uint8_t sdConnected[BOARD_NUM_MCI];
    uint8_t i;
    msdBuffer = (uint8_t *)BUF_START_ADDR;
    /* Infinite loop */
    for (i = 0; i < BOARD_NUM_MCI; i ++) sdConnected[i]=0;
    for (i = 0; i < BOARD_NUM_MCI; i ++)
    {
        if (CardIsConnected(i)) {
            if (sdConnected[i] == 0) {
                sdConnected[i] = 1;
                printf("-I- connect to solt %x \n\r",i);
                CardInit(i);
                pSd = &sdDrv[i];
                SD_DumpCID(pSd->CID);
                SD_DumpCSD(pSd->CSD);
                SD_DumpExtCSD(pSd->EXT);
                MEDSdusb_Initialize(&medias[0], pSd);
            }
        }
        else if (sdConnected[i]) {
            sdConnected[i] = 0;
            printf("** Card %d Disconnected\n\r",i);
        }
    }
    
    LUN_Init(&(luns[0]),
             &(medias[0]),
             msdBuffer, MSD_BUFFER_SIZE,
             0, 0, 0, 0,
             MSDCallbacks_Data);
}
#endif

/**
 * Initialize MSD Media & LUNs
 */
static void _MemoriesInitialize(sSdCard *pSd)
{
    uint32_t i ;

    /* Reset all LUNs */
    for (i = 0; i < MAX_LUNS; i ++)
        LUN_Init(&luns[i], 0, 0, 0, 0, 0, 0, 0, 0);

#if defined LUN_SDCARD
    /*Initialize SD Card  */
    SDDiskInit(pSd);
#endif

#if defined LUN_RAMDISK
    /*Initialize RAM disk */
    RamDiskInit();
#endif

    gNbMedias = 1;
}
/**
 * Configure USBHS settings for USB device
 */
static void _ConfigureUotghs(void)
{
    /* UTMI parallel mode, High/Full/Low Speed */
    /* UUSBCK not used in this configuration (High Speed) */
    PMC->PMC_SCDR = PMC_SCDR_USBCLK;
    /* USB clock register: USB Clock Input is UTMI PLL */
    PMC->PMC_USB = PMC_USB_USBS;
    /* Enable peripheral clock for USBHS */
    PMC_EnablePeripheral(ID_USBHS);    
    USBHS->USBHS_CTRL = USBHS_CTRL_UIMOD_DEVICE;
    /* Enable PLL 480 MHz */
    PMC->CKGR_UCKR = CKGR_UCKR_UPLLEN | CKGR_UCKR_UPLLCOUNT(0xF);
    /* Wait that PLL is considered locked by the PMC */
    while( !(PMC->PMC_SR & PMC_SR_LOCKU) );

    /* IRQ */
    NVIC_EnableIRQ(USBHS_IRQn) ;
}
/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief usb_massstorage Application entry point.
 *
 * Configures UART,
 * Configures TC0, USB MSD Driver and run it.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main( void )
{
    sSdCard *pSd = 0;
 
    /* Disable watchdog */
    WDT_Disable( WDT ) ;
        
    SCB_EnableICache();
    SCB_EnableDCache();

#if defined LUN_RAMDISK
     /* Enable SDRAM */
    BOARD_ConfigureSdram();
#endif

    TRACE_INFO("-- USB Device Mass Storage Example %s --\n\r", SOFTPACK_VERSION);
    TRACE_INFO("-- %s\n\r", BOARD_NAME);
    TRACE_INFO("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0);

    /* Initialize all USB power (off) */
    _ConfigureUotghs();
    /* Initialize PIO pins */
    _ConfigurePIOs();

    /* Initialize drivers */
    _ConfigureDrivers();

    _MemoriesInitialize(pSd);
    
    /* BOT driver initialization */
    MSDDriver_Initialize(&msdDriverDescriptors, luns, MAX_LUNS);

    /* connect if needed */
    USBD_Connect();
    while (1) {

        /* Mass storage state machine */
        if (USBD_GetState() < USBD_STATE_CONFIGURED){}
        else 
        {
          MSDDriver_StateMachine();
          
          if (msdRefresh)
          {
              msdRefresh = 0;

              if (msdWriteTotal < 50 * 1000)
              {
                  /* Flush Disk Media */
              }
              msdWriteTotal = 0;
          }
        }
    }
}
/** \endcond */
