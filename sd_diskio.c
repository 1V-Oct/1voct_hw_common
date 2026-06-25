/* USER CODE BEGIN Header */

/**
 ******************************************************************************
 * @file    sd_diskio.c
 * @brief   SD Disk I/O driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Note: code generation based on sd_diskio_dma_template_bspv1.c v2.1.4
   as "Use dma template" is enabled. */

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection*/

/* Includes ------------------------------------------------------------------*/
#include "sd_diskio.h"
#include "bsp_driver_sd.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "vo_logger.h"
#include "vo_string.h"

#include <string.h>

static FATFS SDFatFS; /* File system object for SD logical drive */

// DBG_MAKE_TAG("sd-diskio")
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*
 * the following Timeout is useful to give the control back to the applications
 * in case of errors in either BSP_SD_ReadCpltCallback() or BSP_SD_WriteCpltCallback()
 * the value by default is as defined in the BSP platform driver otherwise 30 secs
 */
#define SD_TIMEOUT            1 * 1000

#define SD_DEFAULT_BLOCK_SIZE 512

/*
 * Depending on the use case, the SD card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize() and add a call to
 * BSP_SD_Init() elsewhere in the application.
 */
/* USER CODE BEGIN disableSDInit */
/* #define DISABLE_SD_INIT */
/* USER CODE END disableSDInit */

/* Private variables ---------------------------------------------------------*/

/* Disk status */
static volatile DSTATUS Stat     = STA_NOINIT;

static volatile UINT WriteStatus = 0, ReadStatus = 0;
/* Private function prototypes -----------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun);
DSTATUS        SD_initialize(BYTE);
DSTATUS        SD_status(BYTE);
DRESULT        SD_read(BYTE, BYTE *, DWORD, UINT);
#if _USE_WRITE == 1
DRESULT SD_write(BYTE, const BYTE *, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE, BYTE, void *);
#endif /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef SD_Driver =
  {
    SD_initialize,
    SD_status,
    SD_read,
#if _USE_WRITE == 1
    SD_write,
#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1
    SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/

static int SD_CheckStatusWithTimeout(uint32_t timeout) {
  uint32_t timer = HAL_GetTick();
  /* block until SDIO IP is ready again or a timeout occur */
  while (HAL_GetTick() - timer < timeout) {
    if (BSP_SD_GetCardState() == SD_TRANSFER_OK) {
      return 0;
    }
  }

  return -1;
}

static DSTATUS SD_CheckStatus(BYTE lun) {
  UNUSED(lun);
  Stat = STA_NOINIT;

  if (BSP_SD_GetCardState() == MSD_OK) {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
 * @brief  Initializes a Drive
 * @param  lun : not used
 * @retval DSTATUS: Operation status
 */
DSTATUS SD_initialize(BYTE lun) {

#if !defined(DISABLE_SD_INIT)

  if (BSP_SD_Init() == MSD_OK) {
    Stat = SD_CheckStatus(lun);
  }

#else
  Stat = SD_CheckStatus(lun);
#endif

  return Stat;
}

/**
 * @brief  Gets Disk Status
 * @param  lun : not used
 * @retval DSTATUS: Operation status
 */
DSTATUS SD_status(BYTE lun) {
  return SD_CheckStatus(lun);
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */
/**
 * @brief  Reads Sector(s)
 * @param  lun : not used
 * @param  *buff: Data buffer to store read data
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors to read (1..128)
 * @retval DRESULT: Operation result
 */

uint32_t *sd_dma_buf = (uint32_t *)0x24000000;

DRESULT SD_read_dma(uint32_t *b, uint32_t sec, uint32_t cnt) {
  DRESULT  res = RES_ERROR;
  uint32_t timeout;

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0) {
    return res;
  }

  if (BSP_SD_ReadBlocks_DMA(b, sec, cnt) == MSD_OK) {
    ReadStatus = 0;
    timeout    = HAL_GetTick();
    while ((ReadStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT)) {
      // empty loop
    }
    if (ReadStatus == 0) {
      res = RES_ERROR;
    } else {
      ReadStatus = 0;
      timeout    = HAL_GetTick();

      while ((HAL_GetTick() - timeout) < SD_TIMEOUT) {
        if (BSP_SD_GetCardState() == SD_TRANSFER_OK) {
          res = RES_OK;
          break;
        }
      }
    }
  }
  return res;
}

DRESULT SD_write_dma(uint32_t *b, uint32_t sec, uint32_t cnt) {
  DRESULT  res = RES_ERROR;
  uint32_t timeout;

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0) {
    return res;
  }

  if (BSP_SD_WriteBlocks_DMA(b, sec, cnt) == MSD_OK) {
    WriteStatus = 0;
    timeout     = HAL_GetTick();
    while ((WriteStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT)) {
      // empty loop
    }
    /* incase of a timeout return error */
    if (WriteStatus == 0) {
      res = RES_ERROR;
    } else {
      WriteStatus = 0;
      timeout     = HAL_GetTick();

      while ((HAL_GetTick() - timeout) < SD_TIMEOUT) {
        if (BSP_SD_GetCardState() == SD_TRANSFER_OK) {
          res = RES_OK;
          break;
        }
      }
    }
  }
  return res;
}

DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count) {
  UNUSED(lun);
  DRESULT res = RES_ERROR;
  // uint8_t ret;

  // uart_printf("SD_Read %08x %d %d\n", buff, sector, count);
  //  if(BSP_SD_ReadBlocks((uint32_t*)buff, (uint32_t) (sector), count, SD_TIMEOUT) == MSD_OK) {
  //    res = RES_OK;
  //  }

  uint32_t addr;
  addr = (uint32_t)buff;
  addr = (addr >> 24);

  // if (addr == 0x24) {
  //   res = SD_read_dma((uint32_t *)buff, (uint32_t) sector, count);
  // SCB_CleanDCache();
  //     SCB_InvalidateDCache_by_Addr((uint32_t *)buff, 512 * count);
  // } else
  {
    // uart_printf("single buf\n");
    uint8_t *d;
    d = buff;
    while (count--) {
      res = SD_read_dma(sd_dma_buf, (uint32_t)sector++, 1);
      if (res != RES_OK) {
        LOGE("Error?\n");
      }
      SCB_InvalidateDCache_by_Addr(sd_dma_buf, 512);
      vo_memcpy(d, sd_dma_buf, 512);
      d += 512;
    }
  }
  return res;
#if 0
          int i;
          for(i = 0 ; i < 512 ; i++) {
            // uart_printf("%02x %02x \n", buff[i], ((uint8_t *)sd_dma_buf)[i]);
            uart_printf("%02x ", buff[i]);
          }
          uart_printf("\n\n");
          for(i = 0 ; i < 512 ; i++) {
            // uart_printf("%02x %02x \n", buff[i], ((uint8_t *)sd_dma_buf)[i]);
            uart_printf("%02x ", ((uint8_t *)sd_dma_buf)[i]);
          }
          uart_printf("\n\n");
          uart_printf("\n\n");
#endif
}

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */
/**
 * @brief  Writes Sector(s)
 * @param  lun : not used
 * @param  *buff: Data to be written
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors to write (1..128)
 * @retval DRESULT: Operation result
 */
#if _USE_WRITE == 1

DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count) {
  UNUSED(lun);
  DRESULT res = RES_ERROR;
  // uint32_t timeout;
#if defined(ENABLE_SCRATCH_BUFFER)
  uint8_t ret;
  int     i;
#endif

  const uint8_t *d;
  d = buff;
  while (count--) {
    vo_memcpy(sd_dma_buf, d, BLOCKSIZE);
    SCB_CleanDCache_by_Addr(sd_dma_buf, BLOCKSIZE);
    res = SD_write_dma(sd_dma_buf, (uint32_t)sector++, 1);
    if (res != RES_OK) {
      LOGE("Error?\n");
    }
    d += BLOCKSIZE;
  }

  return res;
}

#endif /* _USE_WRITE == 1 */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */
/**
 * @brief  I/O control operation
 * @param  lun : not used
 * @param  cmd: Control code
 * @param  *buff: Buffer to send/receive control data
 * @retval DRESULT: Operation result
 */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff) {
  UNUSED(lun);
  DRESULT         res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT)
    return RES_NOTRDY;

  switch (cmd) {
  /* Make sure that no pending write process */
  case CTRL_SYNC:
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT:
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD *)buff = CardInfo.LogBlockNbr;
    res            = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE:
    BSP_SD_GetCardInfo(&CardInfo);
    *(WORD *)buff = CardInfo.LogBlockSize;
    res           = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE:
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD *)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    res            = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

// extern "C" {
/* USER CODE BEGIN callbackSection */
/* can be used to modify / following code or add new code */
/* USER CODE END callbackSection */
/**
 * @brief Tx Transfer completed callbacks
 * @param hsd: SD handle
 * @retval None
 */
void BSP_SD_WriteCpltCallback(void) {

  WriteStatus = 1;
}

/**
 * @brief Rx Transfer completed callbacks
 * @param hsd: SD handle
 * @retval None
 */
void BSP_SD_ReadCpltCallback(void) {
  ReadStatus = 1;
}

uint8_t sd_check_card_state(void) {
  HAL_SD_CardStateTypeDef state;
  state = HAL_SD_GetCardState(&hsd1);
  if (state != 4)
    LOGW("Disk Status = %d", state);
  if (state == 0) {
    LOGE("Disk Error - Ejected?");
  }
  return state;
}

uint8_t sd_process(uint8_t old_status, TCHAR const *SDPath, uint8_t wide_op) {
  uint8_t new_status = old_status;
  uint8_t sds        = 1; 

  if (new_status == STO_CARD_MOUNTED) {
    if (sd_check_card_state() == 0) {
      sds = SD_NOT_PRESENT;
    }
  }

  // LOGW("SDS = %d", sds);
  if (sds == SD_NOT_PRESENT) {
    if (new_status != STO_CARD_NOT_INSERTED) {
      new_status = STO_CARD_NOT_INSERTED;
      if (f_mount(NULL, (TCHAR const *)SDPath, 0) != FR_OK) {
        LOGE("Failed to umount SD Card");
      }
      HAL_SD_DeInit(&hsd1);
      LOGI("SD Card removed");
    }
    return new_status;
  }

  if (!new_status && (sds == SD_PRESENT)) {
    // STO_TX_BEGIN
    if (BSP_SD_Init() == HAL_OK) {

      LOGI("SD Card Inserted and Initialised OK")
      new_status = STO_CARD_INSERTED;
      if (wide_op) {
        BSP_SD_UpgradeWide();
      }
      int n = 10;
      while (n-- && new_status != STO_CARD_MOUNTED) {
        if (f_mount(&SDFatFS, (TCHAR const *)SDPath, 1) != FR_OK) {
          LOGW("Failed to mount SD Card. Retrying...");
        } else {
          LOGI("Filesystem mounted OK");
          new_status = STO_CARD_MOUNTED;
          return new_status;
        }
      }
    }
    // really bad coding, fall through statement
    // but too lazy to move it to function :)
    // LOGE("Failed to initialise SD Card");
    HAL_SD_DeInit(&hsd1);
    new_status = STO_CARD_NOT_INSERTED;

    // STO_TX_END
    return new_status;
  }
  return new_status;
}


  // };
  /* USER CODE BEGIN ErrorAbortCallbacks */
  /*
  ==============================================================================================
    depending on the SD_HAL_Driver version, either the HAL_SD_ErrorCallback() or HAL_SD_AbortCallback()
    or both could be defined, activate the callbacks below when suitable and needed
  ==============================================================================================
  void BSP_SD_AbortCallback(void)
  {
  }

  void BSP_SD_ErrorCallback(void)
  {
  }
  */
  /* USER CODE END ErrorAbortCallbacks */

  /* USER CODE BEGIN lastSection */
  /* can be used to modify / undefine previous code or add new code */
  /* USER CODE END lastSection */

  /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
