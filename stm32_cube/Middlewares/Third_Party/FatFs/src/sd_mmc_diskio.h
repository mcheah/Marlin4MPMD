/**
  ******************************************************************************
  * @file    sd_mmc_diskio.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-sept-2016
  * @brief   Header for sd_mmc_diskio.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SD_MMC_DISKIO_H
#define __SD_MMC_DISKIO_H

/* Includes ------------------------------------------------------------------*/
#include "diskio.h"
#include "ffconf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

DSTATUS MMC_disk_initialize(void);

DSTATUS MMC_disk_status(void);

DRESULT MMC_disk_read(BYTE *buff, DWORD sector, UINT count);

#if _USE_WRITE == 1
DRESULT MMC_disk_write(const BYTE *buff, DWORD sector, UINT count);
#endif

#if _USE_IOCTL == 1
DRESULT MMC_disk_ioctl(BYTE cmd, void *buff);
#endif


#endif /* __SD_MMC_DISKIO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

