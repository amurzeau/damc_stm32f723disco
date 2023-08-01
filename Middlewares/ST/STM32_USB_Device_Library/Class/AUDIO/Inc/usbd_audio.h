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
#include  "usbd_ioreq.h"

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
#define USBD_AUDIO_FREQ                               48000U
#endif /* USBD_AUDIO_FREQ */

#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES                       1U
#endif /* USBD_AUDIO_FREQ */

#ifndef AUDIO_HS_BINTERVAL
#define AUDIO_HS_BINTERVAL                            0x04U
#endif /* AUDIO_HS_BINTERVAL */

#ifndef AUDIO_FS_BINTERVAL
#define AUDIO_FS_BINTERVAL                            0x01U
#endif /* AUDIO_FS_BINTERVAL */

#define AUDIO_INTERRUPT_EP                            0x83U
#ifndef AUDIO_OUT_EP
#define AUDIO_OUT_EP                                  0x04U
#endif /* AUDIO_OUT_EP */
#define AUDIO_OUT_FEEDBACK_EP                         0x84U
#define AUDIO_IN_EP                                   0x85U

#define AUDIO_UNIT_ID_PER_ENDPOINT 3
#define AUDIO_UNIT_ID_OFFSET_FEATURE_UNIT 2

#define AUDIO_INTERFACE_DESC_SIZE                     0x09U
#define USB_AUDIO_DESC_SIZ                            0x09U
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE             0x09U
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE            0x07U

#define AUDIO_DESCRIPTOR_TYPE                         0x21U
#define USB_DEVICE_CLASS_AUDIO                        0x01U
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01U
#define AUDIO_SUBCLASS_AUDIOSTREAMING                 0x02U
#define AUDIO_PROTOCOL_UNDEFINED                      0x00U
#define AUDIO_STREAMING_GENERAL                       0x01U
#define AUDIO_STREAMING_FORMAT_TYPE                   0x02U

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE               0x24U
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE                0x25U

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                          0x01U
#define AUDIO_CONTROL_INPUT_TERMINAL                  0x02U
#define AUDIO_CONTROL_OUTPUT_TERMINAL                 0x03U
#define AUDIO_CONTROL_FEATURE_UNIT                    0x06U

#define AUDIO_INPUT_TERMINAL_DESC_SIZE                0x0CU
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE               0x09U
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE           0x07U

#define AUDIO_CONTROL_MUTE                            1
#define AUDIO_CONTROL_VOLUME                          2

#define AUDIO_FORMAT_TYPE_I                           0x01U
#define AUDIO_FORMAT_TYPE_III                         0x03U

#define AUDIO_ENDPOINT_GENERAL                        0x01U

#define AUDIO_REQ_GET_CUR                             0x81U
#define AUDIO_REQ_GET_MIN                             0x82U
#define AUDIO_REQ_GET_MAX                             0x83U
#define AUDIO_REQ_GET_RES                             0x84U
#define AUDIO_REQ_SET_CUR                             0x01U
#define AUDIO_REQ_SET_MIN                             0x02U
#define AUDIO_REQ_SET_MAX                             0x03U
#define AUDIO_REQ_SET_RES                             0x04U
#define AUDIO_REQ_GET_STAT                            0xFFU

#define AUDIO_OUT_STREAMING_CTRL                      0x02U

#define AUDIO_OUT_TC                                  0x01U
#define AUDIO_IN_TC                                   0x02U


#define AUDIO_OUT_PACKET                              (uint16_t)(((USBD_AUDIO_FREQ * USBD_AUDIO_CHANNELS * USBD_AUDIO_BYTES_PER_SAMPLE) / 1000U))
#define AUDIO_OUT_MAX_PACKET                          (uint16_t)((2 * USBD_AUDIO_CHANNELS * USBD_AUDIO_BYTES_PER_SAMPLE) + AUDIO_OUT_PACKET)
#define AUDIO_OUT_FEEDBACK_MAX_PACKET                 ((uint16_t)4)

#define AUDIO_DEFAULT_VOLUME                          70U

#define AUDIO_SAMPLE_FREQ(frq) \
  (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE \
	(uint8_t)(AUDIO_OUT_MAX_PACKET & 0xFFU), (uint8_t)((AUDIO_OUT_MAX_PACKET >> 8) & 0xFFU)

/* Number of sub-packets in the audio transfer buffer. You can modify this value but always make sure
  that it is an even number and higher than 3 */
//#define AUDIO_OUT_PACKET_NUM                          80U
/* Total size of the audio transfer buffer */
//#define AUDIO_TOTAL_BUF_SIZE                          ((uint16_t)(AUDIO_OUT_PACKET * AUDIO_OUT_PACKET_NUM))

/* Audio Commands enumeration */
typedef enum
{
  AUDIO_CMD_START = 1,
  AUDIO_CMD_PLAY,
  AUDIO_CMD_STOP,
} AUDIO_CMD_TypeDef;


typedef enum
{
  AUDIO_OFFSET_NONE = 0,
  AUDIO_OFFSET_HALF,
  AUDIO_OFFSET_FULL,
  AUDIO_OFFSET_UNKNOWN,
} AUDIO_OffsetTypeDef;
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

struct history_data {
	uint32_t cycles;
	const char* operation;
	uint32_t DOEPCTL;
	uint32_t DOEPINT;
};

enum USBD_AUDIO_BufferState {
	BS_AvailableForUSB,
	BS_USBBusy,
	BS_AvailableForApp,
};

typedef struct {
	volatile enum USBD_AUDIO_BufferState state;
	uint32_t size;
	uint8_t buffer[AUDIO_OUT_MAX_PACKET];
} USBD_AUDIO_Buffer;

typedef struct {
	uint32_t size;
	uint8_t* buffer;
} USBD_AUDIO_BufferRef;

typedef struct {
	uint32_t is_in;
	uint32_t endpoint;
	uint32_t endpoint_feedback;
	uint32_t max_packet_size;
	uint32_t nominal_packet_size;

	uint32_t current_alternate;
	int32_t next_target_frame;
	int32_t next_target_frame_feedback;
	uint32_t feedback;
	uint32_t transfer_in_progress;
	uint32_t incomplete_iso;
	uint32_t complete_iso;
#ifdef USB_AUDIO_ENABLE_HISTORY
	struct history_data history[256];
	struct history_data history_on_isoincomplete[256];
#endif
#ifdef USB_AUDIO_ENABLE_HISTORY
	uint8_t history_index;
	uint8_t history_save_disable;
#endif
	int32_t accumulated_transmit_error;
	uint32_t usb_index_for_prepare;
	uint32_t usb_index_for_processing;
	USBD_AUDIO_Buffer buffer[2] __attribute__((aligned(4)));
	USBD_AUDIO_ControlTypeDef control;
	uint8_t buffer_feedback[AUDIO_OUT_FEEDBACK_MAX_PACKET] __attribute__((aligned(4)));
} USBD_AUDIO_LoopbackDataTypeDef;

#ifdef USB_AUDIO_ENABLE_HISTORY
void USBD_AUDIO_trace(USBD_AUDIO_LoopbackDataTypeDef* data, const char* operation);
#else
#define USBD_AUDIO_trace(...) ((void)0)
#endif

extern volatile uint8_t usb_new_frame_flag;
extern USBD_AUDIO_LoopbackDataTypeDef usb_audio_endpoint_out_data[AUDIO_OUT_NUMBER];
extern USBD_AUDIO_LoopbackDataTypeDef usb_audio_endpoint_in_data[AUDIO_IN_NUMBER];
uint8_t* USBD_AUDIO_GetBufferFromApp(USBD_AUDIO_LoopbackDataTypeDef *data);
void USBD_AUDIO_ReleaseBufferFromApp(USBD_AUDIO_LoopbackDataTypeDef *data);
void USBD_AUDIO_NotifyUnitIdChanged(uint8_t unit_id);

typedef struct
{
	USBD_AUDIO_LoopbackDataTypeDef* ep0_data_endpoint;
	USBD_SetupReqTypedef control_req;
	uint8_t control_tx_data[USB_MAX_EP0_SIZE];
} USBD_AUDIO_HandleTypeDef;


typedef struct
{
  int8_t (*Init)(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
  int8_t (*DeInit)(uint32_t options);
  int8_t (*AudioCmd)(uint8_t *pbuf, uint32_t size, uint8_t cmd);
  int8_t (*VolumeCtl)(uint8_t vol);
  int8_t (*MuteCtl)(uint8_t cmd);
  int8_t (*PeriodicTC)(uint8_t *pbuf, uint32_t size, uint8_t cmd);
  int8_t (*GetState)(void);
} USBD_AUDIO_ItfTypeDef;

/*
 * Audio Class specification release 1.0
 */

/* Table 4-2: Class-Specific AC Interface Header Descriptor */
typedef struct
{
  uint8_t           bLength;
  uint8_t           bDescriptorType;
  uint8_t           bDescriptorSubtype;
  uint16_t          bcdADC;
  uint16_t          wTotalLength;
  uint8_t           bInCollection;
  uint8_t           baInterfaceNr;
} __PACKED USBD_SpeakerIfDescTypeDef;

/* Table 4-3: Input Terminal Descriptor */
typedef struct
{
  uint8_t           bLength;
  uint8_t           bDescriptorType;
  uint8_t           bDescriptorSubtype;
  uint8_t           bTerminalID;
  uint16_t          wTerminalType;
  uint8_t           bAssocTerminal;
  uint8_t           bNrChannels;
  uint16_t          wChannelConfig;
  uint8_t           iChannelNames;
  uint8_t           iTerminal;
} __PACKED USBD_SpeakerInDescTypeDef;

/* USB Speaker Audio Feature Unit Descriptor */
typedef struct
{
  uint8_t           bLength;
  uint8_t           bDescriptorType;
  uint8_t           bDescriptorSubtype;
  uint8_t           bUnitID;
  uint8_t           bSourceID;
  uint8_t           bControlSize;
  uint16_t          bmaControls;
  uint8_t           iTerminal;
} __PACKED USBD_SpeakerFeatureDescTypeDef;

/* Table 4-4: Output Terminal Descriptor */
typedef struct
{
  uint8_t           bLength;
  uint8_t           bDescriptorType;
  uint8_t           bDescriptorSubtype;
  uint8_t           bTerminalID;
  uint16_t          wTerminalType;
  uint8_t           bAssocTerminal;
  uint8_t           bSourceID;
  uint8_t           iTerminal;
} __PACKED USBD_SpeakerOutDescTypeDef;

/* Table 4-19: Class-Specific AS Interface Descriptor */
typedef struct
{
  uint8_t           bLength;
  uint8_t           bDescriptorType;
  uint8_t           bDescriptorSubtype;
  uint8_t           bTerminalLink;
  uint8_t           bDelay;
  uint16_t          wFormatTag;
} __PACKED USBD_SpeakerStreamIfDescTypeDef;

/* USB Speaker Audio Type III Format Interface Descriptor */
typedef struct
{
  uint8_t           bLength;
  uint8_t           bDescriptorType;
  uint8_t           bDescriptorSubtype;
  uint8_t           bFormatType;
  uint8_t           bNrChannels;
  uint8_t           bSubFrameSize;
  uint8_t           bBitResolution;
  uint8_t           bSamFreqType;
  uint8_t           tSamFreq2;
  uint8_t           tSamFreq1;
  uint8_t           tSamFreq0;
} USBD_SpeakerIIIFormatIfDescTypeDef;

/* Table 4-17: Standard AC Interrupt Endpoint Descriptor */
typedef struct
{
  uint8_t           bLength;
  uint8_t           bDescriptorType;
  uint8_t           bEndpointAddress;
  uint8_t           bmAttributes;
  uint16_t          wMaxPacketSize;
  uint8_t           bInterval;
  uint8_t           bRefresh;
  uint8_t           bSynchAddress;
} __PACKED USBD_SpeakerEndDescTypeDef;

/* Table 4-21: Class-Specific AS Isochronous Audio Data Endpoint Descriptor        */
typedef struct
{
  uint8_t           bLength;
  uint8_t           bDescriptorType;
  uint8_t           bDescriptor;
  uint8_t           bmAttributes;
  uint8_t           bLockDelayUnits;
  uint16_t          wLockDelay;
} __PACKED USBD_SpeakerEndStDescTypeDef;

/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

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

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev,
                                     USBD_AUDIO_ItfTypeDef *fops);

void USBD_AUDIO_Sync(USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset);

#ifdef USE_USBD_COMPOSITE
uint32_t USBD_AUDIO_GetEpPcktSze(USBD_HandleTypeDef *pdev, uint8_t If, uint8_t Ep);
#endif /* USE_USBD_COMPOSITE */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_AUDIO_H */
/**
  * @}
  */

/**
  * @}
  */
