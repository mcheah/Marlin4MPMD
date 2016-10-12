/*-----------------------------------------------------------------------*/
/* Disk I/O module for SD/MMC drive     (C)ST Microelectronics, 2016     */
/*-----------------------------------------------------------------------*/
/* Based on low level disk I/O module skeleton for FatFs - (C)ChaN, 2016 */
/* This module has been designed only for SD/MMC device. In case of      */
/* other device type (RAM, USB, ...), the physical drive number shall    */
/* be used to select the correct action.                                 */
/*-----------------------------------------------------------------------*/


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "diskio.h"		/* FatFs lower layer API */
#include "sd_mmc_diskio.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Block Size */
#define BLOCK_SIZE                512

#define TRUE  1
#define FALSE 0

/* Private variables ---------------------------------------------------------*/
static volatile BYTE mmc_initialized = FALSE;

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive number to identify the drive */
)
{
	DSTATUS stat = STA_OK;

	if(BSP_SD_Init() != MSD_OK)
	{
		stat |= STA_NOINIT;
	}
	mmc_initialized = TRUE;

	return stat;
}

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive number to identify the drive */
)
{
	DSTATUS stat = STA_OK;

	if(BSP_SD_GetStatus() != MSD_OK)
	{
		stat |= STA_NOINIT;
	}

	return stat;
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	if(BSP_SD_ReadBlocks_DMA(
			(uint32_t*)buff,
	        (uint64_t) (sector * BLOCK_SIZE),
	        BLOCK_SIZE,
	        count) != MSD_OK)
	{
		return RES_ERROR;
	}

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
#if _USE_WRITE == 1
	if(BSP_SD_WriteBlocks_DMA(
			(uint32_t*)buff,
	        (uint64_t)(sector * BLOCK_SIZE),
	        BLOCK_SIZE, count) != MSD_OK)
	{
		return RES_ERROR;
	}

	return RES_OK;
#else
	return RES_PARERR;
#endif
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
#if _USE_IOCTL == 1

	DRESULT res = RES_ERROR;
	SD_CardInfo CardInfo;

	if (!mmc_initialized)
		return RES_NOTRDY;

	switch (cmd)
	{
		/* Make sure that no pending write process */
		case CTRL_SYNC :
			res = RES_OK;
			break;

		/* Get number of sectors on the disk (DWORD) */
		case GET_SECTOR_COUNT :
			BSP_SD_GetCardInfo(&CardInfo);
			*(DWORD*)buff = CardInfo.CardCapacity / BLOCK_SIZE;
			res = RES_OK;
			break;

		/* Get R/W sector size (WORD) */
		case GET_SECTOR_SIZE :
			*(WORD*)buff = BLOCK_SIZE;
			res = RES_OK;
			break;

		/* Get erase block size in unit of sector (DWORD) */
		case GET_BLOCK_SIZE :
			*(DWORD*)buff = BLOCK_SIZE;
			break;

		default:
			res = RES_PARERR;
	}

	return res;

#else
	return RES_PARERR;
#endif
}



