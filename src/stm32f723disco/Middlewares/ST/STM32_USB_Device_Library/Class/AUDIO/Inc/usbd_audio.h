/**
  ******************************************************************************
  * @file    usbd_audio.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_audio.c file.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_AUDIO_H
#define __USB_AUDIO_H

//#define USB_AUDIO_ENABLE_HISTORY

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_AUDIO
  * @brief This file is the Header file for usbd_audio.c
  * @{
  */


/** @defgroup USBD_AUDIO_Exported_Defines
  * @{
  */

#ifndef USBD_AUDIO_FREQ
/* AUDIO Class Config */
#define USBD_AUDIO_FREQ 48000U
#endif /* USBD_AUDIO_FREQ */

#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES 1U
#endif /* USBD_AUDIO_FREQ */

#ifndef AUDIO_HS_BINTERVAL
#define AUDIO_HS_BINTERVAL 0x04U
#endif /* AUDIO_HS_BINTERVAL */

#ifndef AUDIO_FS_BINTERVAL
#define AUDIO_FS_BINTERVAL 0x01U
#endif /* AUDIO_FS_BINTERVAL */

#define AUDIO_INTERRUPT_EP 0x83U
#ifndef AUDIO_OUT_EP
#define AUDIO_OUT_EP 0x04U
#endif /* AUDIO_OUT_EP */
#define AUDIO_OUT_FEEDBACK_EP 0x84U
#define AUDIO_IN_EP           0x85U

#define AUDIO_UNIT_ID_PER_ENDPOINT        3
#define AUDIO_UNIT_ID_OFFSET_FEATURE_UNIT 2

#define AUDIO_INTERFACE_DESC_SIZE          0x09U
#define USB_AUDIO_DESC_SIZ                 0x09U
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE  0x09U
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE 0x07U

#define AUDIO_DESCRIPTOR_TYPE         0x21U
#define USB_DEVICE_CLASS_AUDIO        0x01U
#define AUDIO_SUBCLASS_AUDIOCONTROL   0x01U
#define AUDIO_SUBCLASS_AUDIOSTREAMING 0x02U
#define AUDIO_PROTOCOL_UNDEFINED      0x00U
#define AUDIO_STREAMING_GENERAL       0x01U
#define AUDIO_STREAMING_FORMAT_TYPE   0x02U

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE 0x24U
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE  0x25U

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER          0x01U
#define AUDIO_CONTROL_INPUT_TERMINAL  0x02U
#define AUDIO_CONTROL_OUTPUT_TERMINAL 0x03U
#define AUDIO_CONTROL_FEATURE_UNIT    0x06U

#define AUDIO_INPUT_TERMINAL_DESC_SIZE      0x0CU
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE     0x09U
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE 0x07U

#define AUDIO_CONTROL_MUTE   1
#define AUDIO_CONTROL_VOLUME 2

#define AUDIO_FORMAT_TYPE_I   0x01U
#define AUDIO_FORMAT_TYPE_III 0x03U

#define AUDIO_ENDPOINT_GENERAL 0x01U

#define AUDIO_REQ_GET_CUR  0x81U
#define AUDIO_REQ_GET_MIN  0x82U
#define AUDIO_REQ_GET_MAX  0x83U
#define AUDIO_REQ_GET_RES  0x84U
#define AUDIO_REQ_SET_CUR  0x01U
#define AUDIO_REQ_SET_MIN  0x02U
#define AUDIO_REQ_SET_MAX  0x03U
#define AUDIO_REQ_SET_RES  0x04U
#define AUDIO_REQ_GET_STAT 0xFFU


#define AUDIO_OUT_PACKET              (uint16_t)(((USBD_AUDIO_FREQ * USBD_AUDIO_CHANNELS * USBD_AUDIO_BYTES_PER_SAMPLE) / 1000U))
#define AUDIO_OUT_MAX_PACKET          (uint16_t)((2 * USBD_AUDIO_CHANNELS * USBD_AUDIO_BYTES_PER_SAMPLE) + AUDIO_OUT_PACKET)
#define AUDIO_OUT_FEEDBACK_MAX_PACKET ((uint16_t)4)

#define AUDIO_SAMPLE_FREQ(frq) (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE (uint8_t)(AUDIO_OUT_MAX_PACKET & 0xFFU), (uint8_t)((AUDIO_OUT_MAX_PACKET >> 8) & 0xFFU)

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef struct
{
  uint8_t cmd;
  uint8_t data[USB_MAX_EP0_SIZE];
  uint8_t len;
  uint8_t unit;
} USBD_AUDIO_ControlTypeDef;

#ifdef USB_AUDIO_ENABLE_HISTORY
struct history_data
{
  uint32_t cycles;
  const char *operation;
  uint32_t DOEPCTL;
  uint32_t DOEPINT;
};
#endif

typedef struct
{
  uint32_t size;
  uint8_t buffer[AUDIO_OUT_MAX_PACKET];
} USBD_AUDIO_Buffer;

typedef struct
{
  uint32_t is_in;
  uint32_t endpoint;
  uint32_t endpoint_feedback;
  uint32_t max_packet_size;
  uint32_t nominal_packet_size;

  uint32_t index;
  uint32_t current_alternate;
  int32_t next_target_frame;
  int32_t next_target_frame_feedback;
  uint32_t feedback;
  uint32_t transfer_in_progress;
  uint32_t feedback_transfer_in_progress;
  uint32_t incomplete_iso;
  uint32_t complete_iso;
  uint32_t waiting_start;
  uint32_t waiting_stop;
#ifdef USB_AUDIO_ENABLE_HISTORY
  struct history_data history[256];
  struct history_data history_on_isoincomplete[256];
#endif
#ifdef USB_AUDIO_ENABLE_HISTORY
  uint8_t history_index;
  uint8_t history_save_disable;
#endif
  int32_t accumulated_transmit_error;
  USBD_AUDIO_Buffer buffer __attribute__((aligned(4)));
  USBD_AUDIO_ControlTypeDef control;
  uint8_t buffer_feedback[AUDIO_OUT_FEEDBACK_MAX_PACKET] __attribute__((aligned(4)));
} USBD_AUDIO_LoopbackDataTypeDef;

#ifdef USB_AUDIO_ENABLE_HISTORY
void USBD_AUDIO_trace(USBD_AUDIO_LoopbackDataTypeDef *data, const char *operation);
#else
#define USBD_AUDIO_trace(...) ((void)0)
#endif

extern USBD_AUDIO_LoopbackDataTypeDef usb_audio_endpoint_out_data[AUDIO_OUT_NUMBER];
extern USBD_AUDIO_LoopbackDataTypeDef usb_audio_endpoint_in_data[AUDIO_IN_NUMBER];
void USBD_AUDIO_NotifyUnitIdChanged(uint8_t unit_id);
uint32_t USBD_AUDIO_IsEndpointEnabled(int is_in, int index);

typedef struct
{
  USBD_AUDIO_LoopbackDataTypeDef *ep0_data_endpoint;
  USBD_SetupReqTypedef control_req;
  uint8_t control_tx_data[USB_MAX_EP0_SIZE];
} USBD_AUDIO_HandleTypeDef;

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef USBD_AUDIO;
#define USBD_AUDIO_CLASS &USBD_AUDIO
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_AUDIO_H */
/**
  * @}
  */

/**
  * @}
  */
