#include "usb_otg_fs.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USB_OTG_FS init function */

void usb_otg_init_hw(void) {
  GPIO_InitTypeDef         GPIO_InitStruct     = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection     = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection        = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Enable USB Voltage detector
   */
  HAL_PWREx_EnableUSBVoltageDetector();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USB_OTG_FS GPIO Configuration
  PA11     ------> USB_OTG_FS_DM
  PA12     ------> USB_OTG_FS_DP
  */
  GPIO_InitStruct.Pin       = USB_DM_Pin | USB_DP_Pin;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USB_OTG_FS clock enable */
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
}
#if 0
void MX_USB_OTG_FS_PCD_Init(void) {

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /*USB

  SER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance                     = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints           = 9;
  hpcd_USB_OTG_FS.Init.speed                   = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable              = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface              = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable              = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable        = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable              = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable     = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1       = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcdHandle) {

  GPIO_InitTypeDef         GPIO_InitStruct     = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if (pcdHandle->Instance == USB_OTG_FS) {
    /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

    /* USER CODE END USB_OTG_FS_MspInit 0 */

    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
      Error_Handler();
    }

    /** Enable USB Voltage detector
     */
    HAL_PWREx_EnableUSBVoltageDetector();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USB_OTG_FS GPIO Configuration
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    GPIO_InitStruct.Pin       = USB_DM_Pin | USB_DP_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USB_OTG_FS clock enable */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

    /* USER CODE END USB_OTG_FS_MspInit 1 */
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcdHandle) {

  if (pcdHandle->Instance == USB_OTG_FS) {
    /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

    /* USER CODE END USB_OTG_FS_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

    /**USB_OTG_FS GPIO Configuration
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    HAL_GPIO_DeInit(GPIOA, USB_DM_Pin | USB_DP_Pin);

    /* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

    /* USER CODE END USB_OTG_FS_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#endif