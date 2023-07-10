/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * @verbatim
  *
  *          ===================================================================
  *                                AUDIO Class  Description
  *          ===================================================================
  *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48KHz.
  *             - Bit resolution: 16
  *             - Number of channels: 2
  *             - No volume control
  *             - Mute/Unmute capability
  *             - Asynchronous Endpoints
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}_audio.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */

/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev);

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_OutTokenWhileDisabled(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetCtl16(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req, uint16_t value);
static void AUDIO_REQ_SetCmd(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void *USBD_AUDIO_GetAudioHeaderDesc(uint8_t *pConfDesc);

static uint8_t zero_data[AUDIO_OUT_PACKET];
static uint8_t dummy_buffer[AUDIO_OUT_PACKET];
/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_AUDIO =
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,
  USBD_AUDIO_OutTokenWhileDisabled,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  NULL,
  NULL,
  NULL,
  NULL,
#endif /* USE_USBD_COMPOSITE  */
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  NULL,
#endif /* USBD_SUPPORT_USER_STRING_DESC  */
};

/**
  * @}
  */
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

#ifdef USB_AUDIO_ENABLE_HISTORY
void USBD_AUDIO_trace(USBD_AUDIO_LoopbackDataTypeDef* data, const char* operation) {
  if(data < loopbackData || data >= &loopbackData[AUDIO_LOOPBACKS_NUMBER])
    return;
  data->history_index++;
  data->history[data->history_index].cycles = DWT->CYCCNT;
  data->history[data->history_index].operation = operation;

  USB_OTG_GlobalTypeDef *USBx = hpcd_USB_OTG_HS.Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  data->history[data->history_index].DOEPCTL = USBx_OUTEP(5)->DOEPCTL;
  data->history[data->history_index].DOEPINT = USBx_OUTEP(5)->DOEPINT;
  //data->history[data->history_index].frame = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos;
}
#endif

USBD_AUDIO_LoopbackDataTypeDef* USBD_AUDIO_getDataFromEndpoint(USBD_AUDIO_HandleTypeDef* haudio,
                                                             uint8_t epnum, uint8_t is_in)
{
	if(epnum < 3 || haudio == NULL)
		return NULL;

	uint32_t index = (epnum & 0x0F) - 3;

	if(is_in) {
		if(index >= AUDIO_IN_NUMBER)
			return NULL;

		return &usb_audio_endpoint_in_data[index];
	} else {
		if(index >= AUDIO_OUT_NUMBER)
			return NULL;

		return &usb_audio_endpoint_out_data[index];
	}
}

USBD_AUDIO_LoopbackDataTypeDef* USBD_AUDIO_getDataFromInterface(USBD_AUDIO_HandleTypeDef* haudio,
                                                              uint8_t interface)
{
	if(interface < 3 || haudio == NULL)
		return NULL;

	uint32_t index = (interface - 3) / 2;
	uint32_t is_in = ((interface - 3) % 2) == 1;

	if(is_in) {
		if(index < AUDIO_IN_NUMBER) {
			return &usb_audio_endpoint_in_data[index];
		}
	} else {
		if(index < AUDIO_OUT_NUMBER) {
			return &usb_audio_endpoint_out_data[index];
		}
	}

	return NULL;
}

USBD_AUDIO_LoopbackDataTypeDef* USBD_AUDIO_getDataFromUnitId(USBD_AUDIO_HandleTypeDef* haudio,
                                                           uint8_t unitId)
{
	if(unitId < 1 || haudio == NULL)
		return NULL;

	uint32_t index = (unitId - 1) / 6;
	uint32_t is_in = (((unitId - 1) / 3) % 2) == 1;

	if(is_in) {
		if(index < AUDIO_IN_NUMBER) {
			return &usb_audio_endpoint_in_data[index];
		}
	} else {
		if(index < AUDIO_OUT_NUMBER) {
			return &usb_audio_endpoint_out_data[index];
		}
	}
	return NULL;
}

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
USBD_AUDIO_LoopbackDataTypeDef usb_audio_endpoint_out_data[AUDIO_OUT_NUMBER];
USBD_AUDIO_LoopbackDataTypeDef usb_audio_endpoint_in_data[AUDIO_IN_NUMBER];

volatile uint8_t usb_new_frame_flag;
static volatile uint32_t *SCB_DEMCR = (volatile uint32_t *)0xE000EDFC; //address of the register

static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_AUDIO_HandleTypeDef *haudio;

  /* Allocate Audio structure */
  haudio = (USBD_AUDIO_HandleTypeDef *)USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));

  if (haudio == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassDataCmsit[pdev->classId] = (void *)haudio;

  *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
  DWT->CYCCNT = 0;
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;


#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  for(size_t i = 0; i < AUDIO_OUT_NUMBER; i++) {
	  unsigned int ep = AUDIO_OUT_EP + i;
	  USBD_AUDIO_LoopbackDataTypeDef* data = &usb_audio_endpoint_out_data[i];
	  data->is_in = 0;
	  data->endpoint = ep;
	  data->max_packet_size = AUDIO_OUT_PACKET;
	  data->nominal_packet_size = AUDIO_OUT_PACKET;
	  data->current_alternate = 0U;
	  data->next_target_frame = -1;
	  data->transfer_in_progress = 0;
	  memset(&data->buffer, 0, sizeof(data->buffer));
	  data->buffer[0].state = BS_AvailableForUSB;
	  data->buffer[1].state = BS_AvailableForUSB;

	  if (pdev->dev_speed == USBD_SPEED_HIGH)
	  {
		pdev->ep_out[ep & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
	  }
	  else   /* LOW and FULL-speed endpoints */
	  {
		pdev->ep_out[ep & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
	  }

	  /* Open EP OUT */
	  (void)USBD_LL_OpenEP(pdev, ep, USBD_EP_TYPE_ISOC, data->max_packet_size);
	  pdev->ep_out[ep & 0xFU].is_used = 1U;
  }


  for(size_t i = 0; i < AUDIO_IN_NUMBER; i++) {
	  unsigned int ep = AUDIO_IN_EP + i;
	  USBD_AUDIO_LoopbackDataTypeDef* data = &usb_audio_endpoint_in_data[i];
	  data->is_in = 1;
	  data->endpoint = ep;
	  data->max_packet_size = AUDIO_OUT_PACKET;
	  data->nominal_packet_size = AUDIO_OUT_PACKET;
	  data->current_alternate = 0U;
	  data->next_target_frame = -1;
	  data->transfer_in_progress = 0;
	  memset(&data->buffer, 0, sizeof(data->buffer));
	  data->buffer[0].state = BS_AvailableForApp;
	  data->buffer[1].state = BS_AvailableForApp;

	  /* Open EP IN */
	  if (pdev->dev_speed == USBD_SPEED_HIGH)
	  {
		pdev->ep_in[ep & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
	  }
	  else   /* LOW and FULL-speed endpoints */
	  {
		pdev->ep_in[ep & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
	  }

	  /* Open EP OUT */
	  (void)USBD_LL_OpenEP(pdev, ep, USBD_EP_TYPE_ISOC, data->max_packet_size);
	  pdev->ep_in[ep & 0xFU].is_used = 1U;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */


  /* DeInit  physical Interface components */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
	  for(size_t i = 0; i < AUDIO_IN_NUMBER; i++) {
		  /* Close EP IN */
		  USBD_AUDIO_LoopbackDataTypeDef* data = &usb_audio_endpoint_in_data[i];
		  (void)USBD_LL_CloseEP(pdev, data->endpoint);
		  pdev->ep_out[data->endpoint & 0xFU].is_used = 0U;
		  pdev->ep_out[data->endpoint & 0xFU].bInterval = 0U;
	  }

	  for(size_t i = 0; i < AUDIO_OUT_NUMBER; i++) {
		  /* Close EP OUT */
		  USBD_AUDIO_LoopbackDataTypeDef* data = &usb_audio_endpoint_out_data[i];
		  (void)USBD_LL_CloseEP(pdev, data->endpoint);
		  pdev->ep_out[data->endpoint & 0xFU].is_used = 0U;
		  pdev->ep_out[data->endpoint & 0xFU].bInterval = 0U;
	  }

    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case AUDIO_REQ_GET_CUR:
          AUDIO_REQ_GetCurrent(pdev, req);
          break;

        case AUDIO_REQ_GET_MIN:
        	AUDIO_REQ_GetCtl16(pdev, req, 0x8001);
          break;

        case AUDIO_REQ_GET_MAX:
        	AUDIO_REQ_GetCtl16(pdev, req, 0);
          break;

        case AUDIO_REQ_GET_RES:
        	AUDIO_REQ_GetCtl16(pdev, req, 256);
          break;

        case AUDIO_REQ_SET_CUR:
          AUDIO_REQ_SetCmd(pdev, req);
          break;


        case AUDIO_REQ_SET_MIN:
        case AUDIO_REQ_SET_MAX:
        case AUDIO_REQ_SET_RES:
          AUDIO_REQ_SetCmd(pdev, req);
		  break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
          {
            pbuf = (uint8_t *)USBD_AUDIO_GetAudioHeaderDesc(pdev->pConfDesc);
            if (pbuf != NULL)
            {
              len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);
              (void)USBD_CtlSendData(pdev, pbuf, len);
            }
            else
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_AUDIO_LoopbackDataTypeDef* data = USBD_AUDIO_getDataFromInterface(haudio, req->wIndex);
            if(data == NULL) {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            } else {
              (void) USBD_CtlSendData(pdev, (uint8_t*) &data->current_alternate, 1U);
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_AUDIO_LoopbackDataTypeDef* data = USBD_AUDIO_getDataFromInterface(haudio, req->wIndex);
            if(data == NULL) {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            } else {
              if(data->is_in && req->wValue == 1 && data->current_alternate == 0) {
                PCD_HandleTypeDef* pcd = (PCD_HandleTypeDef*)pdev->pData;
                data->next_target_frame = (pcd->FrameNumber + 1) & 0x3FFF;
              }
              data->current_alternate = req->wValue;
              if(data->is_in) {
                  if(req->wValue)
                      USBD_AUDIO_trace(data, "Set Alternate IN 1");
                  else
                      USBD_AUDIO_trace(data, "Set Alternate IN 0");
              } else {
                  if(req->wValue)
                      USBD_AUDIO_trace(data, "Set Alternate OUT 1");
                  else
                      USBD_AUDIO_trace(data, "Set Alternate OUT 0");
              }
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}


/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
  {
    /* In this driver, to simplify code, only SET_CUR request is managed */

    if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL)
    {
      haudio->control.cmd = 0U;
      haudio->control.len = 0U;
    }
  }

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);

  /* Only OUT control data are processed */
  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
uint32_t missed_sofs;
uint32_t last_frame_number_gap;
uint32_t duplicate_sofs;
uint32_t long_sofs;
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev)
{
  PCD_HandleTypeDef* pcd = (PCD_HandleTypeDef*)pdev->pData;
  USB_OTG_GlobalTypeDef *USBx = pcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t frameNumber = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;

  static uint32_t previous_framenumber;

  if(previous_framenumber == frameNumber) {
	  duplicate_sofs++;
	  return USBD_OK;
  }
  if(((previous_framenumber + 1) & 0x3fff) != frameNumber) {
	  missed_sofs++;
	  last_frame_number_gap = (frameNumber + 0x4000 - previous_framenumber) & 0x3fff;
  }

  previous_framenumber = frameNumber;

  if((frameNumber % 8) == 0) {
	  usb_new_frame_flag = 1;
  }

  for(size_t i = 0; i < AUDIO_IN_NUMBER; i++) {
	  USBD_AUDIO_LoopbackDataTypeDef* data = &usb_audio_endpoint_in_data[i];

	  // Switch buffers on frame 7 as prepare/transmit will be effective only on the next frame
	  data->usb_index_for_processing = data->usb_index_for_prepare;
	  if((frameNumber % 8) == 7) {
		  data->usb_index_for_prepare = !data->usb_index_for_prepare;
	  }

	  if(data->current_alternate && data->next_target_frame == frameNumber) {
		  while(data->transfer_in_progress > 0);
		  data->transfer_in_progress = 1;
		  USBD_AUDIO_Buffer* buffer = &data->buffer[data->usb_index_for_prepare];
		  if(buffer->size > 0 && (1 || buffer->state == BS_AvailableForUSB)) {
			  buffer->state = BS_USBBusy;
			  USBD_AUDIO_trace(data, "USBD_LL_Transmit");
			  USBD_LL_Transmit(pdev, data->endpoint, buffer->buffer, buffer->size);
			  buffer->size = 0U;
		  } else {
			  USBD_AUDIO_trace(data, "USBD_LL_Transmit dummy");
			  USBD_LL_Transmit(pdev, data->endpoint, zero_data, data->nominal_packet_size);
		  }
	  }
  }

  for(size_t i = 0; i < AUDIO_OUT_NUMBER; i++) {
	  USBD_AUDIO_LoopbackDataTypeDef* data = &usb_audio_endpoint_out_data[i];

	  // Switch buffers on frame 7 as prepare/transmit will be effective only on the next frame
	  data->usb_index_for_processing = data->usb_index_for_prepare;
	  if((frameNumber % 8) == 7) {
		  data->usb_index_for_prepare = !data->usb_index_for_prepare;
	  }

	  if(data->current_alternate && data->transfer_in_progress == 0 && data->next_target_frame == frameNumber) {
		  while(data->transfer_in_progress > 0);
		  data->transfer_in_progress = 1;
		  USBD_AUDIO_Buffer* buffer = &data->buffer[data->usb_index_for_prepare];
		  if(1 || buffer->state == BS_AvailableForUSB) {
			  buffer->state = BS_USBBusy;
			  USBD_AUDIO_trace(data, "USBD_LL_PrepareReceive");
			  USBD_LL_PrepareReceive(pdev, data->endpoint, buffer->buffer, data->max_packet_size);
		  } else {
			  USBD_AUDIO_trace(data, "USBD_LL_PrepareReceive dummy");
			  USBD_LL_PrepareReceive(pdev, data->endpoint, dummy_buffer, data->max_packet_size);
		  }
	  }
  }

  uint32_t frameNumber2 = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;
  if(frameNumber2 != frameNumber) {
	  long_sofs++;
	  last_frame_number_gap = (frameNumber + 0x4000 - frameNumber2) & 0x3fff;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_AUDIO_LoopbackDataTypeDef* data = USBD_AUDIO_getDataFromEndpoint(haudio, epnum, 1);

  if(data == NULL) {
	  return USBD_OK;
  }

  USBD_AUDIO_trace(data, "IsoINIncomplete (usbd_audio.c)");
  data->transfer_in_progress = 0;
  data->incomplete_iso++;

  if(data->current_alternate) {
	  PCD_HandleTypeDef* pcd = (PCD_HandleTypeDef*)pdev->pData;
	  data->next_target_frame = (pcd->FrameNumber + 6) & 0x3FFF;
  }

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
uint32_t iso_incomplete_gintsts;
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_AUDIO_LoopbackDataTypeDef* data = USBD_AUDIO_getDataFromEndpoint(haudio, epnum, 0);

  if(data == NULL) {
	  return USBD_OK;
  }

  if(epnum == 5 && iso_incomplete_gintsts == 0) {
	  USB_OTG_GlobalTypeDef *USBx = hpcd_USB_OTG_HS.Instance;
	  iso_incomplete_gintsts = USBx->GINTSTS;
  }

  USBD_AUDIO_trace(data, "IsoOutIncomplete (usbd_audio.c)");
  data->transfer_in_progress = 0;
  data->incomplete_iso++;


  if(data->current_alternate) {
//	  PCD_HandleTypeDef* pcd = (PCD_HandleTypeDef*)pdev->pData;
//	  data->next_target_frame = (pcd->FrameNumber + 6) & 0x3FFF;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_OutTokenWhileDisabled
  *         handle OUT token while endpoint disabled event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_OutTokenWhileDisabled(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_AUDIO_LoopbackDataTypeDef* data = USBD_AUDIO_getDataFromEndpoint(haudio, epnum, 0);

  if(data == NULL) {
	  return USBD_OK;
  }

  USBD_AUDIO_trace(data, "OutTokenWhileDisabled");

#ifdef USB_AUDIO_ENABLE_HISTORY
  if(data->history_save_disable == 1) {
	  uint8_t history_index = data->history_index + 1;
	  memcpy(data->history_on_isoincomplete, &data->history[history_index], (256 - history_index)*sizeof(struct history_data));
	  memcpy(&data->history_on_isoincomplete[256 - history_index], &data->history[0], history_index*sizeof(struct history_data));
  }
  if(data->history_save_disable > 0) {
	  data->history_save_disable--;
  }
#endif

  if(data->current_alternate && data->transfer_in_progress == 0) {
	  PCD_HandleTypeDef* pcd = (PCD_HandleTypeDef*)pdev->pData;
	  data->next_target_frame = (pcd->FrameNumber + 7) & 0x3FFF;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */

// https://elixir.bootlin.com/linux/v6.3.1/source/drivers/usb/dwc2/gadget.c#L2953

static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if(epnum >= 1)
  {
	  USBD_AUDIO_HandleTypeDef *haudio;
	  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	  USBD_AUDIO_LoopbackDataTypeDef* data = USBD_AUDIO_getDataFromEndpoint(haudio, epnum, 1);

	  if(data == NULL) {
	    return USBD_OK;
	  }

	  data->complete_iso++;
	  data->transfer_in_progress = 0;
	  USBD_AUDIO_trace(data, "DataIn");

	  if(data->current_alternate == 0) {
	    return USBD_OK;
	  }

	  USBD_AUDIO_Buffer* buffer = &data->buffer[data->usb_index_for_processing];
	  if(buffer->state == BS_USBBusy) {
		  buffer->state = BS_AvailableForApp;
		  USBD_AUDIO_trace(data, "TS_TX_Empty");
	  }

	  data->next_target_frame = (data->next_target_frame + 8) & 0x3FFF;
  }

  /* Only OUT data are processed */
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef *haudio;

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_AUDIO_LoopbackDataTypeDef* data = USBD_AUDIO_getDataFromEndpoint(haudio, epnum, 0);

  if (haudio == NULL || data == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  USBD_AUDIO_trace(data, "DataOut");
  data->transfer_in_progress = 0;
  data->complete_iso++;

  if (epnum >= AUDIO_OUT_EP)
  {
	  USBD_AUDIO_Buffer* buffer = &data->buffer[data->usb_index_for_processing];

    /* Get received data packet length */
	  buffer->size = (uint16_t)USBD_LL_GetRxDataSize(pdev, epnum);

    /* Prepare Out endpoint to receive next audio packet */
	  if(buffer->state == BS_USBBusy) {
		  buffer->state = BS_AvailableForApp;
		  USBD_AUDIO_trace(data, "TS_RX_ReadyToProcess");
	  }

	data->next_target_frame = (data->next_target_frame + 8) & 0x3FFF;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  (void)USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);

  /* Send the current mute state */
  (void)USBD_CtlSendData(pdev, haudio->control.data,
                         MIN(req->wLength, USB_MAX_EP0_SIZE));
}
/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetCtl16(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req, uint16_t value)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  (void)USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);

  memcpy(haudio->control.data, &value, sizeof(value));

  /* Send the current mute state */
  (void)USBD_CtlSendData(pdev, haudio->control.data,
                         MIN(req->wLength, USB_MAX_EP0_SIZE));
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCmd(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  if (req->wLength != 0U)
  {
    haudio->control.cmd = req->bRequest;     /* Set the request value */
    haudio->control.len = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);  /* Set the request data length */
    haudio->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */

    /* Prepare the reception of the buffer over EP0 */
    (void)USBD_CtlPrepareRx(pdev, haudio->control.data, haudio->control.len);
  }
}


#ifdef USE_USBD_COMPOSITE
/**
  * @brief  USBD_AUDIO_GetEpPcktSze
  * @param  pdev: device instance (reserved for future use)
  * @param  If: Interface number (reserved for future use)
  * @param  Ep: Endpoint number (reserved for future use)
  * @retval status
  */
uint32_t USBD_AUDIO_GetEpPcktSze(USBD_HandleTypeDef *pdev, uint8_t If, uint8_t Ep)
{
  uint32_t mps;

  UNUSED(pdev);
  UNUSED(If);
  UNUSED(Ep);

  mps = AUDIO_PACKET_SZE_WORD;

  /* Return the wMaxPacketSize value in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
  return mps;
}
#endif /* USE_USBD_COMPOSITE */

/**
  * @brief  USBD_AUDIO_GetAudioHeaderDesc
  *         This function return the Audio descriptor
  * @param  pdev: device instance
  * @param  pConfDesc:  pointer to Bos descriptor
  * @retval pointer to the Audio AC Header descriptor
  */
static void *USBD_AUDIO_GetAudioHeaderDesc(uint8_t *pConfDesc)
{
  USBD_ConfigDescTypeDef *desc = (USBD_ConfigDescTypeDef *)(void *)pConfDesc;
  USBD_DescHeaderTypeDef *pdesc = (USBD_DescHeaderTypeDef *)(void *)pConfDesc;
  uint8_t *pAudioDesc =  NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_GetNextDesc((uint8_t *)pdesc, &ptr);
      if ((pdesc->bDescriptorType == AUDIO_INTERFACE_DESCRIPTOR_TYPE) &&
          (pdesc->bDescriptorSubType == AUDIO_CONTROL_HEADER))
      {
        pAudioDesc = (uint8_t *)pdesc;
        break;
      }
    }
  }
  return pAudioDesc;
}

uint8_t* USBD_AUDIO_GetBufferFromApp(USBD_AUDIO_LoopbackDataTypeDef *data)
{
	USBD_AUDIO_Buffer* buffer = &data->buffer[!data->usb_index_for_processing];
	if(data->is_in || buffer->size > 0) {
		return buffer->buffer;
	} else {
		return zero_data;
	}
}

void USBD_AUDIO_ReleaseBufferFromApp(USBD_AUDIO_LoopbackDataTypeDef *data)
{
	USBD_AUDIO_Buffer* buffer = &data->buffer[!data->usb_index_for_processing];

	if(data->is_in) {
		buffer->size = data->nominal_packet_size;
	} else {
		buffer->size = 0;
	}
	buffer->state = BS_AvailableForUSB;
}
/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */
