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
#include PLATFORM_HEADER
#include "usbd_conf.h"
#include "usbd_ctlreq.h"
#include "AudioCApi.h"
#include "GlitchDetection.h"

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

static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev);

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_OutTokenWhileDisabled(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_InTokenWhileTXEmptyCallback(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void AUDIO_REQ_GetControl(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_SetCmd(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void *USBD_AUDIO_GetAudioHeaderDesc(uint8_t *pConfDesc);

static uint8_t zero_data[AUDIO_OUT_PACKET];
/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_AUDIO = {
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
  USBD_AUDIO_InTokenWhileTXEmptyCallback,
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
void USBD_AUDIO_trace(USBD_AUDIO_LoopbackDataTypeDef *data, const char *operation)
{
  if (data < loopbackData || data >= &loopbackData[AUDIO_LOOPBACKS_NUMBER])
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

uint8_t USBD_AUDIO_isFeedbackEndpoint(uint8_t epnum)
{
  return (((epnum & 0x0F) - (AUDIO_OUT_FEEDBACK_EP & 0x0F)) % 2) == 0;
}

USBD_AUDIO_LoopbackDataTypeDef *USBD_AUDIO_getDataFromEndpoint(uint8_t epnum, uint8_t is_in)
{
  if (is_in)
  {
    int32_t index = ((epnum & 0x0F) - (AUDIO_IN_EP & 0x0F)) / 2;
    if (index < 0 || index >= AUDIO_IN_NUMBER)
      return NULL;

    return &usb_audio_endpoint_in_data[index];
  }
  else
  {
    int32_t index = ((epnum & 0x0F) - (AUDIO_OUT_EP & 0x0F)) / 2;
    if (index < 0 || index >= AUDIO_OUT_NUMBER)
      return NULL;

    return &usb_audio_endpoint_out_data[index];
  }
}

USBD_AUDIO_LoopbackDataTypeDef *USBD_AUDIO_getDataFromInterface(USBD_AUDIO_HandleTypeDef *haudio, uint8_t interface)
{
  if (interface < 3 || haudio == NULL)
    return NULL;

  uint32_t index = (interface - 3) / 2;
  uint32_t is_in = ((interface - 3) % 2) == 1;

  if (is_in)
  {
    if (index < AUDIO_IN_NUMBER)
    {
      return &usb_audio_endpoint_in_data[index];
    }
  }
  else
  {
    if (index < AUDIO_OUT_NUMBER)
    {
      return &usb_audio_endpoint_out_data[index];
    }
  }

  return NULL;
}

USBD_AUDIO_LoopbackDataTypeDef *USBD_AUDIO_getDataFromUnitId(USBD_AUDIO_HandleTypeDef *haudio, uint8_t unitId)
{
  if (unitId < 1 || haudio == NULL)
    return NULL;

  uint32_t index = (unitId - 1) / AUDIO_UNIT_ID_PER_ENDPOINT / 2;
  uint32_t is_in = (((unitId - 1) / AUDIO_UNIT_ID_PER_ENDPOINT) % 2) == 1;

  if (is_in)
  {
    if (index < AUDIO_IN_NUMBER)
    {
      return &usb_audio_endpoint_in_data[index];
    }
  }
  else
  {
    if (index < AUDIO_OUT_NUMBER)
    {
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
static uint16_t usb_audio_notify_unit_id_change;
static uint16_t usb_audio_notify_in_progress_data;

static volatile uint32_t *SCB_DEMCR = (volatile uint32_t *)0xE000EDFC;  //address of the register

static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  static USBD_AUDIO_HandleTypeDef haudioData;
  USBD_AUDIO_HandleTypeDef *haudio;

  /* Allocate Audio structure */
  haudio = &haudioData;

  if (haudio == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassDataCmsit[pdev->classId] = (void *)haudio;

  *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  for (size_t i = 0; i < AUDIO_OUT_NUMBER; i++)
  {
    unsigned int ep = AUDIO_OUT_EP + i * 2;
    unsigned int ep_feedback = AUDIO_OUT_FEEDBACK_EP + i * 2;
    USBD_AUDIO_LoopbackDataTypeDef *data = &usb_audio_endpoint_out_data[i];
    data->is_in = 0;
    data->endpoint = ep;
    data->endpoint_feedback = ep_feedback;
    data->max_packet_size = AUDIO_OUT_MAX_PACKET;
    data->nominal_packet_size = AUDIO_OUT_PACKET;
    data->index = i;
    data->current_alternate = 0U;
    data->next_target_frame = -1;
    data->next_target_frame_feedback = -1;
    data->transfer_in_progress = 0;
    memset(&data->buffer, 0, sizeof(data->buffer));

    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
      pdev->ep_out[ep & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
    }
    else /* LOW and FULL-speed endpoints */
    {
      pdev->ep_out[ep & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
    }

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, ep, USBD_EP_TYPE_ISOC, data->max_packet_size);
    pdev->ep_out[ep & 0xFU].is_used = 1U;

    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
      pdev->ep_in[ep_feedback & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
    }
    else /* LOW and FULL-speed endpoints */
    {
      pdev->ep_in[ep_feedback & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
    }

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, ep_feedback, USBD_EP_TYPE_ISOC, AUDIO_OUT_FEEDBACK_MAX_PACKET);
    pdev->ep_in[ep_feedback & 0xFU].is_used = 1U;
  }


  for (size_t i = 0; i < AUDIO_IN_NUMBER; i++)
  {
    unsigned int ep = AUDIO_IN_EP + i * 2;
    USBD_AUDIO_LoopbackDataTypeDef *data = &usb_audio_endpoint_in_data[i];
    data->is_in = 1;
    data->endpoint = ep;
    data->max_packet_size = AUDIO_OUT_MAX_PACKET;
    data->nominal_packet_size = AUDIO_OUT_PACKET;
    data->index = i;
    data->current_alternate = 0U;
    data->next_target_frame = -1;
    data->transfer_in_progress = 0;
    memset(&data->buffer, 0, sizeof(data->buffer));

    /* Open EP IN */
    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
      pdev->ep_in[ep & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
    }
    else /* LOW and FULL-speed endpoints */
    {
      pdev->ep_in[ep & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
    }

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, ep, USBD_EP_TYPE_ISOC, data->max_packet_size);
    pdev->ep_in[ep & 0xFU].is_used = 1U;
  }

  {
    uint8_t ep = AUDIO_INTERRUPT_EP;
    /* Open EP IN for control status interrupt */
    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
      pdev->ep_in[ep & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
    }
    else /* LOW and FULL-speed endpoints */
    {
      pdev->ep_in[ep & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
    }

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, ep, USBD_EP_TYPE_INTR, 2);
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

  /* DeInit  physical Interface components */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    for (size_t i = 0; i < AUDIO_IN_NUMBER; i++)
    {
      /* Close EP IN */
      USBD_AUDIO_LoopbackDataTypeDef *data = &usb_audio_endpoint_in_data[i];
      (void)USBD_LL_CloseEP(pdev, data->endpoint);
      pdev->ep_out[data->endpoint & 0xFU].is_used = 0U;
      pdev->ep_out[data->endpoint & 0xFU].bInterval = 0U;
    }

    for (size_t i = 0; i < AUDIO_OUT_NUMBER; i++)
    {
      /* Close EP OUT */
      USBD_AUDIO_LoopbackDataTypeDef *data = &usb_audio_endpoint_out_data[i];
      (void)USBD_LL_CloseEP(pdev, data->endpoint);
      pdev->ep_out[data->endpoint & 0xFU].is_used = 0U;
      pdev->ep_out[data->endpoint & 0xFU].bInterval = 0U;
      /* Close EP OUT feedback */
      (void)USBD_LL_CloseEP(pdev, data->endpoint_feedback);
      pdev->ep_out[data->endpoint_feedback & 0xFU].is_used = 0U;
      pdev->ep_out[data->endpoint_feedback & 0xFU].bInterval = 0U;
    }

    (void)USBD_LL_CloseEP(pdev, AUDIO_INTERRUPT_EP);
    pdev->ep_out[AUDIO_INTERRUPT_EP & 0xFU].is_used = 0U;
    pdev->ep_out[AUDIO_INTERRUPT_EP & 0xFU].bInterval = 0U;

    //(void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
  }

  return (uint8_t)USBD_OK;
}

static void USBD_EnableOutTokenWhileDisabled(USBD_HandleTypeDef *pdev)
{
  PCD_HandleTypeDef *pcd = (PCD_HandleTypeDef *)pdev->pData;
  USB_OTG_GlobalTypeDef *USBx = pcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  USBx_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_OTEPDM;
}

static void USBD_DisableOutTokenWhileDisabled(USBD_HandleTypeDef *pdev)
{
  // Disable interrupt when not needed as it will cause many CDC interrupts
  bool is_interrupt_needed = false;

  for (size_t i = 0; i < AUDIO_OUT_NUMBER; i++)
  {
    if (usb_audio_endpoint_out_data[i].waiting_start)
    {
      is_interrupt_needed = true;
      break;
    }
  }

  // Interrupt is still needed
  if (is_interrupt_needed)
    return;

  PCD_HandleTypeDef *pcd = (PCD_HandleTypeDef *)pdev->pData;
  USB_OTG_GlobalTypeDef *USBx = pcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  USBx_DEVICE->DOEPMSK &= ~USB_OTG_DOEPMSK_OTEPDM;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
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
        case AUDIO_REQ_GET_MIN:
        case AUDIO_REQ_GET_MAX:
        case AUDIO_REQ_GET_RES:
          AUDIO_REQ_GetControl(pdev, req);
          break;

        case AUDIO_REQ_SET_CUR:
          AUDIO_REQ_SetCmd(pdev, req);
          break;


        case AUDIO_REQ_SET_MIN:
        case AUDIO_REQ_SET_MAX:
        case AUDIO_REQ_SET_RES:
          AUDIO_REQ_SetCmd(pdev, req);
          break;

        case AUDIO_REQ_GET_STAT:
          USBD_CtlSendData(pdev, haudio->control_tx_data, 0);
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
            USBD_AUDIO_LoopbackDataTypeDef *data = USBD_AUDIO_getDataFromInterface(haudio, req->wIndex);
            if (data == NULL)
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
            else
            {
              (void)USBD_CtlSendData(pdev, (uint8_t *)&data->current_alternate, 1U);
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
            USBD_AUDIO_LoopbackDataTypeDef *data = USBD_AUDIO_getDataFromInterface(haudio, req->wIndex);
            if (data == NULL)
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
            else
            {
              if (req->wValue == 1 && data->current_alternate == 0)
              {
                if (data->is_in)
                {
                  data->next_target_frame = -1;
                }
                else
                {
                  // ISO Out endpoint activated, we need to know which frame number the USB Host will use
                  // for these ISO Out transfers.
                  // For this, enable OutTokenWhileDisabled interrupt and wait for it
                  // before scheduling ISO Out transfers.
                  // For the feedback ISO In endpoint, we can't do that and instead will try until we get it
                  // using successive ISO In Incomplete interrupt.
                  data->next_target_frame = -1;
                  data->next_target_frame_feedback = -1;
                  GLITCH_DETECTION_set_USB_out_feedback_state(data->index, false);
                  USBD_EnableOutTokenWhileDisabled(pdev);
                }
                // We will start many ISO IN transfert with Incomplete IRQ at each micro frame
                // leading to higher CPU usage, reset CPU frequency to max
                DAMC_resetFrequencyToMaxPerformance();
              }
              data->current_alternate = req->wValue;
              if (data->current_alternate)
              {
                // Start of audio stream, we wait actual ISO transfer
                // before enabling glitch detection.
                data->waiting_start = 1;
                data->complete_iso = 0;
              }
              else
              {
                // End of audio stream, if we were waiting for stream start,
                // we need to cancel it and disable OutTokenWhileDisabled if not needed.
                data->waiting_start = 0;
                USBD_DisableOutTokenWhileDisabled(pdev);
                // Mark the stream as out of sync
                GLITCH_DETECTION_set_USB_out_feedback_state(data->index, false);
              }
              data->waiting_stop = 0;

              if (data->is_in)
              {
                if (req->wValue)
                  USBD_AUDIO_trace(data, "Set Alternate IN 1");
                else
                  USBD_AUDIO_trace(data, "Set Alternate IN 0");
              }
              else
              {
                if (req->wValue)
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
  USBD_AUDIO_HandleTypeDef *haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  uint8_t unit_id = HIBYTE(haudio->control_req.wIndex);

  if (haudio->control_req.bRequest == AUDIO_REQ_SET_CUR)
  {
    /* In this driver, to simplify code, only SET_CUR request is managed */
    uint16_t value = 0;

    memcpy(&value, haudio->control_tx_data, MIN(haudio->control_req.wLength, sizeof(value)));

    DAMC_setControlFromUSB(unit_id, HIBYTE(haudio->control_req.wValue), LOBYTE(haudio->control_req.wValue), haudio->control_req.bRequest, value);

    haudio->control_req = (USBD_SetupReqTypedef){0};
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
__attribute__((used)) uint32_t missed_sofs;
__attribute__((used)) uint32_t last_frame_number_gap;
__attribute__((used)) uint32_t duplicate_sofs;
__attribute__((used)) uint32_t long_sofs;
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev)
{
  PCD_HandleTypeDef *pcd = (PCD_HandleTypeDef *)pdev->pData;
  USB_OTG_GlobalTypeDef *USBx = pcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t frameNumber = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;

  static uint32_t previous_framenumber;

  if (previous_framenumber == frameNumber)
  {
    duplicate_sofs++;
    return USBD_OK;
  }
  if (((previous_framenumber + 1) & 0x3fff) != frameNumber)
  {
    missed_sofs++;
    last_frame_number_gap = (frameNumber + 0x4000 - previous_framenumber) & 0x3fff;
  }

  previous_framenumber = frameNumber;

  // Start ISO transfers when needed according to the frame number.
  // All transfers are started here.

  for (size_t i = 0; i < AUDIO_IN_NUMBER; i++)
  {
    USBD_AUDIO_LoopbackDataTypeDef *data = &usb_audio_endpoint_in_data[i];

    if (data->current_alternate)
    {
      if (data->next_target_frame == -1 && data->transfer_in_progress == 0)
      {
        // Transfer ZLP at each microframe to detect the first frame number of the ISO period
        data->transfer_in_progress = 1;
        USBD_LL_Transmit(pdev, data->endpoint, zero_data, 0);
      }
      else if (data->next_target_frame == frameNumber)
      {
        while (data->transfer_in_progress > 0)
          ;
        data->transfer_in_progress = 1;
        USBD_AUDIO_Buffer *buffer = &data->buffer;
        USBD_AUDIO_trace(data, "USBD_LL_Transmit");

        uint32_t size_to_read;

        if (data->waiting_start && data->complete_iso == 1)
        {
          // We must not set waiting_start to 0 on the same frame as the reset as we might
          // have preempted an audio processing period inside DAMC_writeAudioSample with a full buffer
          // which will cause a buffer overflow detection after returning from this interrupt.

          // Reset audio buffer when the stream is starting, we are doing the first non-ZLP transfer
          DAMC_resetAudioBuffer(DUB_In + i);

          data->accumulated_transmit_error = 0;

          // Don't compute feedback when we reset audio buffers as the feedback will always be the perfect one (no drift).
          size_to_read = 48 << 16;
        }
        else
        {
          // Compute number of sample to send back
          size_to_read = DAMC_getUSBInSizeValue(DUB_In + i);
        }

        // Stream started, next_target_frame is not -1 so we got the first ISO transfer done.
        // This will enable glitch detection.
        if (data->waiting_start && data->complete_iso > 1)
        {
          data->waiting_start = 0;
        }

        // Accumulate rounding error
        data->accumulated_transmit_error += size_to_read;
        size_to_read = data->accumulated_transmit_error / 65536;
        if (size_to_read > sizeof(buffer->buffer) / 4)
        {
          size_to_read = sizeof(buffer->buffer) / 4;
        }
        data->accumulated_transmit_error -= size_to_read * 65536;

        buffer->size = DAMC_readAudioSampleFromUSB(DUB_In + i, buffer->buffer, size_to_read) * 4;

        if (buffer->size > 0)
        {
          USBD_LL_Transmit(pdev, data->endpoint, buffer->buffer, buffer->size);
          buffer->size = 0U;
        }
        else
        {
          USBD_LL_Transmit(pdev, data->endpoint, zero_data, data->nominal_packet_size);
        }
      }
    }
  }

  for (size_t i = 0; i < AUDIO_OUT_NUMBER; i++)
  {
    USBD_AUDIO_LoopbackDataTypeDef *data = &usb_audio_endpoint_out_data[i];

    if (data->current_alternate)
    {
      if (data->transfer_in_progress == 0 && data->next_target_frame == frameNumber)
      {
        while (data->transfer_in_progress > 0)
          ;
        data->transfer_in_progress = 1;
        USBD_AUDIO_Buffer *buffer = &data->buffer;
        USBD_AUDIO_trace(data, "USBD_LL_PrepareReceive");
        USBD_LL_PrepareReceive(pdev, data->endpoint, buffer->buffer, data->max_packet_size);
      }
      if ((data->next_target_frame_feedback == -1 && !data->feedback_transfer_in_progress) || data->next_target_frame_feedback == frameNumber)
      {
        data->buffer_feedback[0] = data->feedback & 0xFF;
        data->buffer_feedback[1] = (data->feedback >> 8) & 0xFF;
        data->buffer_feedback[2] = (data->feedback >> 16) & 0xFF;
        data->buffer_feedback[3] = (data->feedback >> 24) & 0xFF;
        data->feedback_transfer_in_progress = 1;
        USBD_LL_Transmit(pdev, data->endpoint_feedback, data->buffer_feedback, sizeof(data->buffer_feedback));
      }
    }
  }

  // If an USB Interrupt is not in progress and a USB control value changed, send a USB interrupt to the USB Host
  uint16_t unit_id_change_bitmask = usb_audio_notify_unit_id_change;
  if (!usb_audio_notify_in_progress_data && unit_id_change_bitmask)
  {
    uint8_t unit_id = 31 - __builtin_clz(unit_id_change_bitmask);
    usb_audio_notify_unit_id_change &= ~(1 << unit_id);
    // bStatusType (lower byte) = bit7 (interrupt pending) and bit0-3 == 0 (audio control interface)
    usb_audio_notify_in_progress_data = (unit_id << 8) | (1 << 7) | (0 << 0);
    USBD_LL_Transmit(pdev, AUDIO_INTERRUPT_EP, (uint8_t *)&usb_audio_notify_in_progress_data, sizeof(usb_audio_notify_in_progress_data));
  }

  uint32_t frameNumber2 = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;
  if (frameNumber2 != frameNumber)
  {
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
  uint8_t is_feedback = USBD_AUDIO_isFeedbackEndpoint(epnum);
  USBD_AUDIO_LoopbackDataTypeDef *data = USBD_AUDIO_getDataFromEndpoint(epnum, !is_feedback);

  if (data == NULL)
  {
    return USBD_OK;
  }

  data->incomplete_iso++;

  // The USB Host did not emitted a ISO In token for the frame we scheduled it,
  // either the USB Host is not reading the ISO In endpoint or
  // it is reading on a different frame number.
  //
  // This event happen at the end of a micro-frame, before the next SOF.
  // So we still have time to prepare a ZLP transfer
  // in case the Host emit ISO In token on the next micro-frame.
  //
  // Schedule a transfer at a different frame number to try to sync with the USB Host.

  if (is_feedback)
  {
    data->feedback_transfer_in_progress = 0;
    if (data->current_alternate)
    {
      if (data->next_target_frame_feedback != -1)
      {
        // First time we get incomplete, reset CPU frequency to max in case we get many Incomplete IRQ
        DAMC_resetFrequencyToMaxPerformance();
      }

      // Prepare for next SOF with ZLP to check for IN token
      data->feedback_transfer_in_progress = 1;
      USBD_LL_Transmit(pdev, data->endpoint_feedback, data->buffer_feedback, 0);

      // Reset next_target_frame_feedback to -1.
      // In case a ZLP transfer is not prepared, SOF handler will take care of it.
      // This happens for the initial transfer to be done after the host switch USB alternate to 1.
      data->next_target_frame_feedback = -1;

      // The feedback transfer was lost and we are not stopping, the host is not reading the feedback clock anymore
      GLITCH_DETECTION_set_USB_out_feedback_state(data->index, false);
    }
  }
  else
  {
    USBD_AUDIO_trace(data, "IsoINIncomplete (usbd_audio.c)");
    data->transfer_in_progress = 0;

    if (data->current_alternate)
    {
      if (data->next_target_frame != -1)
      {
        // First time we get incomplete, reset CPU frequency to max in case we get many Incomplete IRQ
        DAMC_resetFrequencyToMaxPerformance();
      }

      // SOF will try a ZLP transfer at each micro-frame to find on which FrameNumber the ISO IN transfer is.
      data->next_target_frame = -1;

      // Prepare for next SOF with ZLP to check for IN token
      data->transfer_in_progress = 1;
      USBD_LL_Transmit(pdev, data->endpoint, zero_data, 0);

      // Reset complete_iso to prepare for the next start
      data->complete_iso = 0;

      if (!data->waiting_start)
      {
        // We are not waiting for a start, so we got ISO transfer previously but not anymore.
        // Assume the stream is stopping.
        // If it was a glitch (lost ISO transfer) and we will get ISO transfer later, DataIn will handle that case
        // and notify the glitch.
        data->waiting_stop = 1;

        // We are going back to searching the first IN token.
        // So we are also waiting for a possible start too (along with a possible stop)
        // If it was just a glitch, we will act as if a new stream is started.
        data->waiting_start = 1;
      }
    }
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
  USBD_AUDIO_LoopbackDataTypeDef *data = USBD_AUDIO_getDataFromEndpoint(epnum, 0);

  if (data == NULL)
  {
    return USBD_OK;
  }

  if (epnum == 5 && iso_incomplete_gintsts == 0)
  {
    USB_OTG_GlobalTypeDef *USBx = hpcd_USB_OTG_HS.Instance;
    iso_incomplete_gintsts = USBx->GINTSTS;
  }

  USBD_AUDIO_trace(data, "IsoOutIncomplete (usbd_audio.c)");
  data->transfer_in_progress = 0;
  data->incomplete_iso++;

  // Reset complete_iso to prepare for the next start
  data->complete_iso = 0;

  if (data->current_alternate && !data->waiting_start)
  {
    // We are not waiting for a start, so we got ISO transfer previously but not anymore.
    // Assume the stream is stopping.
    // If it was a glitch (lost ISO transfer) and we will get ISO transfer later, DataOut will handle that case
    // and notify the glitch.
    data->waiting_stop = 1;

    // Don't update data->next_target_frame to keep the current running state
    // but reenable OutTokenWhileDisabled in case we got desync.
    // So we are also waiting for a possible start too (along with a possible stop)
    // If it was just a glitch, OutTokenWhileDisabled will be disabled again in DataOut.
    data->waiting_start = 1;
    USBD_EnableOutTokenWhileDisabled(pdev);
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
  USBD_AUDIO_LoopbackDataTypeDef *data = USBD_AUDIO_getDataFromEndpoint(epnum, 0);

  if (data == NULL)
  {
    return USBD_OK;
  }

  USBD_AUDIO_trace(data, "OutTokenWhileDisabled");

#ifdef USB_AUDIO_ENABLE_HISTORY
  if (data->history_save_disable == 1)
  {
    uint8_t history_index = data->history_index + 1;
    memcpy(data->history_on_isoincomplete, &data->history[history_index], (256 - history_index) * sizeof(struct history_data));
    memcpy(&data->history_on_isoincomplete[256 - history_index], &data->history[0], history_index * sizeof(struct history_data));
  }
  if (data->history_save_disable > 0)
  {
    data->history_save_disable--;
  }
#endif

  // We got a OutTokenWhileDisabled interrupt, this means the USB Host
  // tried to send data using a ISO Out token.
  // Schedule a ISO Out receive transfer for the next period.
  // If that transfer succeed, OutTokenWhileDisabled interrupt will be disabled as not needed anymore.
  if (data->current_alternate && data->transfer_in_progress == 0)
  {
    PCD_HandleTypeDef *pcd = (PCD_HandleTypeDef *)pdev->pData;
    data->next_target_frame = (pcd->FrameNumber + 7) & 0x3FFF;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_InTokenWhileTXEmptyCallback
  *         handle OUT token while endpoint disabled event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_InTokenWhileTXEmptyCallback(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_trace(data, "USBD_AUDIO_InTokenWhileTXEmptyCallback");
  if (epnum == 3)
    return USBD_OK;

  uint8_t is_feedback = USBD_AUDIO_isFeedbackEndpoint(epnum);
  if (!is_feedback)
  {
    return USBD_OK;
  }

  USBD_AUDIO_LoopbackDataTypeDef *data = USBD_AUDIO_getDataFromEndpoint(epnum, 0);
  if (data == NULL)
  {
    return USBD_OK;
  }

  // We got a USBD_AUDIO_InTokenWhileTXEmptyCallback interrupt, this means the USB Host
  // tried to receive data using a In token.
  // Schedule a ISO In transmit transfer for the next period.
  // If that transfer succeed, USBD_AUDIO_InTokenWhileTXEmptyCallback interrupt will be disabled as not needed anymore.
  if (data->current_alternate)
  {
    PCD_HandleTypeDef *pcd = (PCD_HandleTypeDef *)pdev->pData;
    data->next_target_frame_feedback = (pcd->FrameNumber + 127) & 0x3FFF;
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
  if (epnum == (AUDIO_INTERRUPT_EP & 0x7F))
  {
    // USB Interrupt completed, reset to 0 so we are
    // ready for a new interrupt transfer.
    usb_audio_notify_in_progress_data = 0;
  }
  else
  {
    uint8_t is_feedback = USBD_AUDIO_isFeedbackEndpoint(epnum);
    USBD_AUDIO_LoopbackDataTypeDef *data = USBD_AUDIO_getDataFromEndpoint(epnum, !is_feedback);

    if (data == NULL)
    {
      return USBD_OK;
    }

    if (is_feedback)
    {
      data->feedback_transfer_in_progress = 0;
      if (data->current_alternate == 0)
      {
        return USBD_OK;
      }

      // Schedule the next feedback transfer
      // Note: next_target_frame_feedback is -1 when we get the first ISO In token
      PCD_HandleTypeDef *pcd = (PCD_HandleTypeDef *)pdev->pData;
      data->next_target_frame_feedback = (pcd->FrameNumber + 127) & 0x3FFF;

      // Mark audio out sync as working as the host read our feedback clock data
      GLITCH_DETECTION_set_USB_out_feedback_state(data->index, true);
    }
    else
    {
      // Used to ensure we never start a transfer while one is already scheduled.
      data->transfer_in_progress = 0;

      // May never happen ? ISO transfer should always be stopped
      // before the USB bandwidth is unallocated by changing to alternate 0,
      // but in case we have a somewhat cancelled transfer, ignore it
      if (data->current_alternate == 0)
      {
        return USBD_OK;
      }

      // Count number of completed ISO transfer for statistics
      data->complete_iso++;

      // Schedule next ISO transfer frame number
      PCD_HandleTypeDef *pcd = (PCD_HandleTypeDef *)pdev->pData;
      data->next_target_frame = (pcd->FrameNumber + 7) & 0x3FFF;

      // If we lost a ISO transfer previously and got DataIn now,
      // a ISO transfer got lost and the stream is continuing instead of stopping.
      if (data->waiting_stop)
      {
        data->waiting_stop = 0;
        GLITCH_DETECTION_increment_counter(GT_UsbIsochronousTransferLost);
      }

      USBD_AUDIO_trace(data, "DataIn");
    }
  }

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

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_AUDIO_LoopbackDataTypeDef *data = USBD_AUDIO_getDataFromEndpoint(epnum, 0);

  if (haudio == NULL || data == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (epnum >= AUDIO_OUT_EP)
  {
    USBD_AUDIO_Buffer *buffer = &data->buffer;

    USBD_AUDIO_trace(data, "DataOut");
    data->transfer_in_progress = 0;
    data->complete_iso++;

    // Reset USB audio buffer when starting to have the perfect buffering level
    // This will avoid early underrun or overrun due to the host audio clock not yet synchronized
    if (data->complete_iso == 1 && data->waiting_start)
    {
      DAMC_resetAudioBuffer(data->index);

      // Don't compute feedback when we reset audio buffers as the feedback will always be the perfect one (no drift).
      // If we compute feedback, we might get wrong values in this case:
      //  - USB ISR preempt the audio processing ISR while reading the usb buffer. The audio pro
      //    - The audio processing ISR will only read available data before usb buffer reset, which might be 0 if there was underflow.
      //    - DAMC_resetAudioBuffer take into account that the read in the current processing period won't read 48 samples but maybe 0
      //  - Then computing feedback would assume a full read of 48 samples by the audio processing ISR:
      //    - DAMC_getUSBFeedbackValue would compute the available space, but after the reset, which will be 48 samples instead of
      //      the previous state that would be 0 sample in case of underflow.
      // Thus, the feedback might get non-exact value.
      // No calling it reduce edge cases.
      // Use the default feedback of 6 << 16 which means no clock drift.
      data->feedback = 6 << 16;
    }
    else
    {
      // Compute feedback to target usbBuffers with 0.25ms of available
      // samples when DMA starts a new buffer (+1ms that we will add just after by DAMC_writeAudioSample).
      // Do this just before adding the received USB OUT samples so the 0.25
      // margin is really a margin and not systematically reduced by waiting the next
      // USB OUT transfer later.
      data->feedback = DAMC_getUSBFeedbackValue(data->index);
    }

    // Assume we are fully started at correct speed after 2 feedbacks,
    // which is 16*2 = 32 data ISO xfer.
    // We are not counting feedback transfer as they might not be done by the USB Host (Windows).
    // This additional wait before assuming the transfer is started is needed
    // to avoid the case where a USB stream just started, but the USB Host is not
    // sending enough data as it didn't received enough feedback yet.
    // In this case, there might be USB buffer underflow which are expected and should not
    // be notified to the user.
    //
    // We also get waiting_start == 1 in case of a USB ISO Out desync where
    // OutTokenWhileDisabled interrupt got enabled again in ISOOutIncomplete.
    if (data->complete_iso >= 32 && data->waiting_start)
    {
      data->waiting_start = 0;
      // OTEPDM Interrupt is not needed anymore
      USBD_DisableOutTokenWhileDisabled(pdev);
    }

    /* Get received data packet length */
    buffer->size = (uint16_t)USBD_LL_GetRxDataSize(pdev, epnum);

    // If we lost a ISO transfer previous and got DataOut now,
    // a ISO transfer got lost and the stream is continuing instead of stopping.
    if (data->waiting_stop)
    {
      data->waiting_stop = 0;
      GLITCH_DETECTION_increment_counter(GT_UsbIsochronousTransferLost);
    }

    uint32_t nframes = buffer->size / 4;
    size_t writtenSize = DAMC_writeAudioSampleFromUSB(data->index, buffer->buffer, nframes);
    if (writtenSize != nframes)
    {
      GLITCH_DETECTION_increment_counter(GT_UsbOutOverrun);
    }

    // Schedule the next ISO transfer
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
static void AUDIO_REQ_GetControl(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  (void)USBD_memset(haudio->control_tx_data, 0, USB_MAX_EP0_SIZE);

  uint16_t value = DAMC_getControlFromUSB(HIBYTE(req->wIndex), HIBYTE(req->wValue), LOBYTE(req->wValue), req->bRequest);

  memcpy(haudio->control_tx_data, &value, sizeof(value));

  /* Send the current mute state */
  (void)USBD_CtlSendData(pdev, haudio->control_tx_data, MIN(req->wLength, USB_MAX_EP0_SIZE));
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
    haudio->ep0_data_endpoint = USBD_AUDIO_getDataFromUnitId(haudio, HIBYTE(req->wIndex));
    haudio->control_req = *req;

    haudio->control_req.wLength = MIN(req->wLength, USB_MAX_EP0_SIZE);
    /* Prepare the reception of the buffer over EP0 */
    (void)USBD_CtlPrepareRx(pdev, haudio->control_tx_data, haudio->control_req.wLength);
  }
}

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
  uint8_t *pAudioDesc = NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_GetNextDesc((uint8_t *)pdesc, &ptr);
      if ((pdesc->bDescriptorType == AUDIO_INTERFACE_DESCRIPTOR_TYPE) && (pdesc->bDescriptorSubType == AUDIO_CONTROL_HEADER))
      {
        pAudioDesc = (uint8_t *)pdesc;
        break;
      }
    }
  }
  return pAudioDesc;
}

void USBD_AUDIO_NotifyUnitIdChanged(uint8_t unit_id)
{
  usb_audio_notify_unit_id_change |= 1 << unit_id;
}

// Return true when the audio stream is started and stable and we are
// ready to detect glitches.
uint32_t USBD_AUDIO_IsEndpointEnabled(int is_in, int index)
{
  USBD_AUDIO_LoopbackDataTypeDef *data;
  if (is_in)
  {
    data = &usb_audio_endpoint_in_data[index];
  }
  else
  {
    data = &usb_audio_endpoint_out_data[index];
  }

  // Read current_alternate last to cover the case where
  // between reading the flags, the stream is stopped (which means waiting_* and current_alternate are set to 0).
  //
  // With "data->current_alternate && !data->waiting_start && !data->waiting_stop",
  // we can have a case like this:
  //  - Initial state: waiting_start = 1, waiting_stop = 1, current_alternate = 1
  //  - Audio processing thread call USBD_AUDIO_IsEndpointEnabled
  //  - USBD_AUDIO_IsEndpointEnabled check for current_alternate => == 1
  //  - Audio processing thread is preempted by USB thread
  //  - USB thread process a SETUP packet that stop the stream and set waiting_start = 0, waiting_stop = 0, current_alternate = 0
  //  - Audio processing thread resume
  //  - USBD_AUDIO_IsEndpointEnabled check for !waiting_start and !waiting_stop => ok they are now 0
  // This leads to false positive glitch detection when the USB stream is stopped.

  bool state1 = !data->waiting_start && !data->waiting_stop && data->current_alternate;
  __DMB();
  bool state2 = !data->waiting_start && !data->waiting_stop && data->current_alternate;

  return state1 && state2;
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
