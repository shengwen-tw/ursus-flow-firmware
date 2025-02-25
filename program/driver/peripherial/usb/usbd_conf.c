#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"

#include "interrupt.h"

PCD_HandleTypeDef hpcd_USB_OTG_FS;
void Error_Handler(void);

/* External functions --------------------------------------------------------*/
void SystemClock_Config(void);

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/

/* MSP Init */
void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(pcdHandle->Instance==USB_OTG_FS) {
		/**USB_OTG_FS GPIO Configuration
		PA9     ------> USB_OTG_FS_VBUS
		PA11     ------> USB_OTG_FS_DM
		PA12     ------> USB_OTG_FS_DP
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_USB_OTG_FS_CLK_ENABLE();

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(OTG_FS_IRQn, USB_FS_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
	}
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* pcdHandle)
{
	if(pcdHandle->Instance==USB_OTG_FS) {
		/* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

		/* USER CODE END USB_OTG_FS_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USB_OTG_FS_CLK_DISABLE();

		/**USB_OTG_FS GPIO Configuration
		PA9     ------> USB_OTG_FS_VBUS
		PA11     ------> USB_OTG_FS_DM
		PA12     ------> USB_OTG_FS_DP
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12);

		/* Peripheral interrupt Deinit*/
		HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

		/* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

		/* USER CODE END USB_OTG_FS_MspDeInit 1 */
	}
}

/**
  * @brief  Setup stage callback
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
	USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
  * @brief  Data Out stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

/**
  * @brief  Data In stage callback..
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/**
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
	USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
	USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

	/*Set USB Current Speed*/
	switch (hpcd->Init.speed) {
	case PCD_SPEED_HIGH:
		speed = USBD_SPEED_HIGH;
		break;
	case PCD_SPEED_FULL:
		speed = USBD_SPEED_FULL;
		break;

	default:
		speed = USBD_SPEED_FULL;
		break;
	}
	USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);

	/*Reset Device*/
	USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  Suspend callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
	/* Inform USB library that core enters in suspend Mode */
	USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
	__HAL_PCD_GATE_PHYCLOCK(hpcd);
	/*Enter in STOP mode */
	/* USER CODE BEGIN 2 */
	if (hpcd->Init.low_power_enable) {
		/* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register */
		SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
	}
	/* USER CODE END 2 */
}

/**
  * @brief  Resume callback.
    When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
	USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);

}

/**
  * @brief  ISOOUTIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

/**
  * @brief  ISOINIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

/**
  * @brief  ConnectCallback callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
	USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
	USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData);
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/
/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Init (USBD_HandleTypeDef *pdev)
{
	/* Init USB_IP */
	if (pdev->id == DEVICE_FS) {
		/* Link The driver to the stack */
		hpcd_USB_OTG_FS.pData = pdev;
		pdev->pData = &hpcd_USB_OTG_FS;

		hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
		hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
		hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
		hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
		hpcd_USB_OTG_FS.Init.ep0_mps = DEP0CTL_MPS_64;
		hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
		hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
		hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
		hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
		hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
		hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
		if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
			//Error_Handler();
		}

		HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
		HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
		HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80);
	}
	return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_DeInit (USBD_HandleTypeDef *pdev)
{
	HAL_PCD_DeInit((PCD_HandleTypeDef*)pdev->pData);
	return USBD_OK;
}

/**
  * @brief  Starts the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
	HAL_PCD_Start((PCD_HandleTypeDef*)pdev->pData);
	return USBD_OK;
}

/**
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Stop (USBD_HandleTypeDef *pdev)
{
	HAL_PCD_Stop((PCD_HandleTypeDef*) pdev->pData);
	return USBD_OK;
}

/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_OpenEP  (USBD_HandleTypeDef *pdev,
                                     uint8_t  ep_addr,
                                     uint8_t  ep_type,
                                     uint16_t ep_mps)
{
	HAL_PCD_EP_Open((PCD_HandleTypeDef*) pdev->pData,
	                ep_addr,
	                ep_mps,
	                ep_type);

	return USBD_OK;
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_CloseEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
	HAL_PCD_EP_Close((PCD_HandleTypeDef*) pdev->pData, ep_addr);
	return USBD_OK;
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_FlushEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
	HAL_PCD_EP_Flush((PCD_HandleTypeDef*) pdev->pData, ep_addr);
	return USBD_OK;
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_StallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
	HAL_PCD_EP_SetStall((PCD_HandleTypeDef*) pdev->pData, ep_addr);
	return USBD_OK;
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_ClearStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
	HAL_PCD_EP_ClrStall((PCD_HandleTypeDef*) pdev->pData, ep_addr);
	return USBD_OK;
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
	PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;

	if((ep_addr & 0x80) == 0x80) {
		return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
	} else {
		return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
	}
}
/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_SetUSBAddress (USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
	HAL_PCD_SetAddress((PCD_HandleTypeDef*) pdev->pData, dev_addr);
	return USBD_OK;
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Transmit (USBD_HandleTypeDef *pdev,
                                      uint8_t  ep_addr,
                                      uint8_t  *pbuf,
                                      uint16_t  size)
{
	HAL_PCD_EP_Transmit((PCD_HandleTypeDef*) pdev->pData, ep_addr, pbuf, size);
	return USBD_OK;
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,
                uint8_t  ep_addr,
                uint8_t  *pbuf,
                uint16_t  size)
{
	HAL_PCD_EP_Receive((PCD_HandleTypeDef*) pdev->pData, ep_addr, pbuf, size);
	return USBD_OK;
}

/**
  * @brief  Returns the last transfered packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Recived Data Size
  */
uint32_t USBD_LL_GetRxDataSize  (USBD_HandleTypeDef *pdev, uint8_t  ep_addr)
{
	return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, ep_addr);
}
#if (USBD_LPM_ENABLED == 1)
/**
  * @brief  HAL_PCDEx_LPM_Callback : Send LPM message to user layer
  * @param  hpcd: PCD handle
  * @param  msg: LPM message
  * @retval HAL status
  */
void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg)
{
	switch ( msg) {
	case PCD_LPM_L0_ACTIVE:
		if (hpcd->Init.low_power_enable) {
			//XXX:SystemClock_Config();

			/* Reset SLEEPDEEP bit of Cortex System Control Register */
			SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
		}
		__HAL_PCD_UNGATE_PHYCLOCK(hpcd);
		USBD_LL_Resume(hpcd->pData);
		break;

	case PCD_LPM_L1_ACTIVE:
		__HAL_PCD_GATE_PHYCLOCK(hpcd);
		USBD_LL_Suspend(hpcd->pData);

		/*Enter in STOP mode */
		if (hpcd->Init.low_power_enable) {
			/* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register */
			SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
		}
		break;
	}
}
#endif
/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void  USBD_LL_Delay (uint32_t Delay)
{
	HAL_Delay(Delay);
}
