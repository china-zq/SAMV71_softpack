#ifndef _LIB_STORAGEMEDIA_ 
#define _LIB_STORAGEMEDIA_ 


/* Define attribute */
#if defined   ( __CC_ARM   ) /* Keil uvision 4 */
    #define WEAK __attribute__ ((weak))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
    #define WEAK __weak
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68 */
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

/*
 * drivers
 */

#include "board.h"
 
#include "Media.h"
#include "MEDNandFlash.h"
#include "MEDRamDisk.h"
#include "MEDSdcard.h"
#include "MEDSdram.h"
#include "sdio.h"
#include "sdmmc.h"
#include "sdmmc_cmd.h"
#include "sdmmc_hal.h"
#include "sdmmc_trace.h"
#endif /* _LIB_STORAGEMEDIA_ */
