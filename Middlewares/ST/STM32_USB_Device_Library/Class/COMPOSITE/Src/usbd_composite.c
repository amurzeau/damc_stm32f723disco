
/* Includes ------------------------------------------------------------------*/
#include "usbd_composite.h"
#include "usbd_ctlreq.h"
#include "usbd_audio.h"
#include "usbd_cdc.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_COMPOSITE
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_COMPOSITE_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_COMPOSITE_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_COMPOSITE_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_COMPOSITE_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_COMPOSITE_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_COMPOSITE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_COMPOSITE_SOF(USBD_HandleTypeDef *pdev);
static uint8_t USBD_COMPOSITE_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_OutTokenWhileDisabled(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *USBD_COMPOSITE_GetCfgDesc(uint16_t *length);
uint8_t *USBD_COMPOSITE_GetDeviceQualifierDescriptor(uint16_t *length);
static uint8_t* USBD_COMPOSITE_GetUsrStrDescriptor(struct _USBD_HandleTypeDef *pdev, uint8_t index,  uint16_t *length);


/** @defgroup USBD_COMPOSITE_Private_Variables
  * @{
  */


/* CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_COMPOSITE =
{
  USBD_COMPOSITE_Init,
  USBD_COMPOSITE_DeInit,
  USBD_COMPOSITE_Setup,
  NULL,                 /* EP0_TxSent */
  USBD_COMPOSITE_EP0_RxReady,
  USBD_COMPOSITE_DataIn,
  USBD_COMPOSITE_DataOut,
  USBD_COMPOSITE_SOF,
  USBD_COMPOSITE_IsoINIncomplete,
  USBD_COMPOSITE_IsoOutIncomplete,
  USBD_COMPOSITE_OutTokenWhileDisabled,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_COMPOSITE_GetCfgDesc,
  USBD_COMPOSITE_GetCfgDesc,
  USBD_COMPOSITE_GetCfgDesc,
  USBD_COMPOSITE_GetDeviceQualifierDescriptor,
#endif /* USE_USBD_COMPOSITE  */
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  USBD_COMPOSITE_GetUsrStrDescriptor,
#endif /* USBD_SUPPORT_USER_STRING_DESC  */
};

#define DECLARE_UNITS_OUT(iTerminal, bBaseTerminalID) \
	/* USB Speaker Input Terminal Descriptor */ \
	AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */ \
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */ \
	AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */ \
	bBaseTerminalID * 3 + 0x01,           /* bTerminalID */ \
	0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */ \
	0x01, \
	0x00,                                 /* bAssocTerminal */ \
	0x02,                                 /* bNrChannels */ \
	0x03,                                 /* wChannelConfig 0x0003  Left & Right Front */ \
	0x00, \
	0x00,                                 /* iChannelNames */ \
	iTerminal,                            /* iTerminal */ \
	/* 12 byte*/ \
\
	/* USB Speaker Audio Feature Unit Descriptor */ \
	10,                                   /* bLength */ \
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */ \
	AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */ \
	bBaseTerminalID * 3 + 0x02,           /* bUnitID */ \
	bBaseTerminalID * 3 + 0x01,           /* bSourceID */ \
	0x01,                                 /* bControlSize */ \
	AUDIO_CONTROL_MUTE,                   /* bmaControls(0) */ \
	AUDIO_CONTROL_VOLUME,                 /* bmaControls(1) */ \
	AUDIO_CONTROL_VOLUME,                 /* bmaControls(2) */ \
	0x00,                                 /* iTerminal */ \
	/* 10 byte*/ \
\
	/* USB Speaker Output Terminal Descriptor */ \
	0x09,                                 /* bLength */ \
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */ \
	AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */ \
	bBaseTerminalID * 3 + 0x03,           /* bTerminalID */ \
	0x03,                                 /* wTerminalType  0x0603  Line connector */ \
	0x06, \
	0x00,                                 /* bAssocTerminal */ \
	bBaseTerminalID * 3 + 0x02,           /* bSourceID */ \
	iTerminal,                            /* iTerminal */ \
	/* 09 byte*/

#define DECLARE_UNITS_IN(iTerminal, bBaseTerminalID) \
	/* USB Mic Input Terminal Descriptor */ \
	AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */ \
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */ \
	AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */ \
	bBaseTerminalID * 3 + 0x01,           /* bTerminalID */ \
	0x03,                                 /* wTerminalType  0x0603  Line connector*/ \
	0x06, \
	0x00,                                 /* bAssocTerminal */ \
	0x02,                                 /* bNrChannels */ \
	0x03,                                 /* wChannelConfig 0x0003  Left & Right Front */ \
	0x00, \
	0x00,                                 /* iChannelNames */ \
	iTerminal,                            /* iTerminal */ \
	/* 12 byte*/ \
\
	/* USB Mic Audio Feature Unit Descriptor */ \
	10,                                   /* bLength */ \
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */ \
	AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */ \
	bBaseTerminalID * 3 + 0x02,           /* bUnitID */ \
	bBaseTerminalID * 3 + 0x01,           /* bSourceID */ \
	0x01,                                 /* bControlSize */ \
	AUDIO_CONTROL_MUTE,                   /* bmaControls(0) */ \
	AUDIO_CONTROL_VOLUME,                 /* bmaControls(1) */ \
	AUDIO_CONTROL_VOLUME,                 /* bmaControls(1) */ \
	0x00,                                 /* iTerminal */ \
	/* 10 byte*/ \
\
	/*USB Mic Output Terminal Descriptor */ \
	0x09,                                 /* bLength */ \
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */ \
	AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */ \
	bBaseTerminalID * 3 + 0x03,           /* bTerminalID */ \
	0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */ \
	0x01, \
	0x00,                                 /* bAssocTerminal */ \
	bBaseTerminalID * 3 + 0x02,           /* bSourceID */ \
	iTerminal,                            /* iTerminal */ \
	/* 09 byte*/

#define DECLARE_ENDPOINT_OUT(bInterfaceNumber, bTerminalLink, bNrChannels, wMaxPacketSize) \
	/* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwidth */ \
	/* Interface 1, Alternate Setting 0                                              */ \
	AUDIO_INTERFACE_DESC_SIZE,            /* bLength */ \
	USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */ \
	0x01 + bInterfaceNumber,              /* bInterfaceNumber */ \
	0x00,                                 /* bAlternateSetting */ \
	0x00,                                 /* bNumEndpoints */ \
	USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */ \
	AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */ \
	AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */ \
	0x00,                                 /* iInterface */ \
	/* 09 byte*/ \
\
	/* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */ \
	/* Interface 1, Alternate Setting 1                                           */ \
	AUDIO_INTERFACE_DESC_SIZE,            /* bLength */ \
	USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */ \
	0x01 + bInterfaceNumber,              /* bInterfaceNumber */ \
	0x01,                                 /* bAlternateSetting */ \
	0x01,                                 /* bNumEndpoints */ \
	USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */ \
	AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */ \
	AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */ \
	0x00,                                 /* iInterface */ \
	/* 09 byte*/ \
\
	/* USB Speaker Audio Streaming Interface Descriptor */ \
	AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */ \
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */ \
	AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */ \
	bTerminalLink * 3 + 0x01,             /* bTerminalLink */ \
	0x01,                                 /* bDelay */ \
	0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001 */ \
	0x00, \
	/* 07 byte*/ \
\
	/* USB Speaker Audio Type III Format Interface Descriptor */ \
	0x0B,                                 /* bLength */ \
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */ \
	AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */ \
	AUDIO_FORMAT_TYPE_I,                  /* bFormatType */ \
	bNrChannels,                          /* bNrChannels */ \
	USBD_AUDIO_BYTES_PER_SAMPLE,          /* bSubFrameSize :  2 Bytes per frame (16bits) */ \
	8 * USBD_AUDIO_BYTES_PER_SAMPLE,      /* bBitResolution (16-bits per sample) */ \
	0x01,                                 /* bSamFreqType only one frequency supported */ \
	AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),   /* Audio sampling frequency coded on 3 bytes */ \
	/* 11 byte*/ \
\
	/* Endpoint 1 - Standard Descriptor */ \
	AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */ \
	USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */ \
	AUDIO_OUT_EP + (bInterfaceNumber/2),      /* bEndpointAddress 1 out endpoint */ \
	USBD_EP_TYPE_ISOC,                    /* bmAttributes */ \
	wMaxPacketSize,                       /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */ \
	AUDIO_HS_BINTERVAL,                   /* bInterval */ \
	0x00,                                 /* bRefresh */ \
	0x00,                                 /* bSynchAddress */ \
	/* 09 byte*/ \
\
	/* Endpoint - Audio Streaming Descriptor */ \
	AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */ \
	AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */ \
	AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */ \
	0x00,                                 /* bmAttributes */ \
	0x00,                                 /* bLockDelayUnits */ \
	0x00,                                 /* wLockDelay */ \
	0x00, \
	/* 07 byte*/

#define DECLARE_ENDPOINT_IN(bInterfaceNumber, bTerminalLink, bNrChannels, wMaxPacketSize) \
	/* USB Mic Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */ \
	/* Interface 2, Alternate Setting 0                                         */ \
	AUDIO_INTERFACE_DESC_SIZE,            /* bLength */ \
	USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */ \
	0x01 + bInterfaceNumber,              /* bInterfaceNumber */ \
	0x00,                                 /* bAlternateSetting */ \
	0x00,                                 /* bNumEndpoints */ \
	USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */ \
	AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */ \
	AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */ \
	0x00, /* iInterface */                /* 09 byte*/ \
\
	/* USB Mic Standard AS Interface Descriptor - Audio Streaming Operational */ \
	/* Interface 2, Alternate Setting 1                                       */ \
	AUDIO_INTERFACE_DESC_SIZE,            /* bLength */ \
	USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */ \
	0x01 + bInterfaceNumber,              /* bInterfaceNumber */ \
	0x01,                                 /* bAlternateSetting */ \
	0x01,                                 /* bNumEndpoints */ \
	USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */ \
	AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */ \
	AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */ \
	0x00,                                 /* iInterface */ \
	/* 09 byte*/ \
\
	/* USB Speaker Audio Streaming Interface Descriptor */ \
	AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */ \
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */ \
	AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */ \
	bTerminalLink * 3 + 0x03,             /* bTerminalLink */ \
	0x01,                                 /* bDelay */ \
	0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001 */ \
	0x00, \
	/* 07 byte*/ \
\
	/* USB Speaker Audio Type III Format Interface Descriptor */ \
	0x0B,                                 /* bLength */ \
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */ \
	AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */ \
	AUDIO_FORMAT_TYPE_I,                  /* bFormatType */ \
	bNrChannels,                          /* bNrChannels */ \
	USBD_AUDIO_BYTES_PER_SAMPLE,          /* bSubFrameSize :  2 Bytes per frame (16bits) */ \
	8 * USBD_AUDIO_BYTES_PER_SAMPLE,      /* bBitResolution (16-bits per sample) */ \
	0x01,                                 /* bSamFreqType only one frequency supported */ \
	AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),   /* Audio sampling frequency coded on 3 bytes */ \
	/* 11 byte*/ \
\
	/* Endpoint 1 - Standard Descriptor */ \
	AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */ \
	USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */ \
	AUDIO_IN_EP + (bInterfaceNumber/2),   /* bEndpointAddress 2 in endpoint */ \
	USBD_EP_TYPE_ISOC,                    /* bmAttributes */ \
	wMaxPacketSize,                       /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */ \
	AUDIO_HS_BINTERVAL,                   /* bInterval */ \
	0x00,                                 /* bRefresh */ \
	0x00,                                 /* bSynchAddress */ \
	/* 09 byte*/ \
\
	/* Endpoint - Audio Streaming Descriptor*/ \
	AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */ \
	AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */ \
	AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */ \
	0x00,                                 /* bmAttributes */ \
	0x00,                                 /* bLockDelayUnits */ \
	0x00,                                 /* wLockDelay */ \
	0x00, \
	/* 07 byte*/

/* USB AUDIO device Configuration Descriptor */
#define USB_AUDIO_CONTROL_DESC_SIZ (8 + 2 * AUDIO_LOOPBACKS_NUMBER + (31 * 2) * AUDIO_LOOPBACKS_NUMBER)
#define USB_AUDIO_CONFIG_DESC_SIZ (9 + 8 + 9 + USB_AUDIO_CONTROL_DESC_SIZ + (52 * 2) * AUDIO_LOOPBACKS_NUMBER + 8 + 58)
#define USBD_AUDIO_STR_FIRST_INDEX 10

enum USBD_AUDIO_StringEnum {
	USBD_AUDIO_STR_SPEAKER1 = USBD_AUDIO_STR_FIRST_INDEX,
	USBD_AUDIO_STR_SPEAKER2,
	USBD_AUDIO_STR_SPEAKER3,
	USBD_AUDIO_STR_SPEAKER4,
	USBD_AUDIO_STR_MIC1,
	USBD_AUDIO_STR_MIC2,
	USBD_AUDIO_STR_MIC3,
	USBD_AUDIO_STR_MIC4,
};

__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration 1 */
  0x09,                                 /* bLength */
  USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
  LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ),    /* wTotalLength */
  HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
  0x01 + 2 * AUDIO_LOOPBACKS_NUMBER + 2,   /* bNumInterfaces */
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,                                 /* bmAttributes: Bus Powered according to user configuration */
#else
  0x80,                                 /* bmAttributes: Bus Powered according to user configuration */
#endif /* USBD_SELF_POWERED */
  USBD_MAX_POWER,                       /* MaxPower (mA) */
  /* 09 byte*/

  /* Interface Association Descriptor */
  0x08,                                 /* bLength */
  0x0B,                                 /* bDescriptorType */
  0x00,                                 /* bFirstInterface */
  0x09,                                 /* bInterfaceCount */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iFunction */

  /* USB Speaker Standard interface descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  0x00,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Class-specific AC Interface Descriptor */
  8 + 2 * AUDIO_LOOPBACKS_NUMBER,            /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
  0x00,          /* 1.00 */             /* bcdADC */
  0x01,
  LOBYTE(USB_AUDIO_CONTROL_DESC_SIZ), /* wTotalLength */
  HIBYTE(USB_AUDIO_CONTROL_DESC_SIZ),
  AUDIO_LOOPBACKS_NUMBER * 2,          /* bInCollection */
  0x01,                                 /* baInterfaceNr(0) */
  0x02,                                 /* baInterfaceNr(1) */
  0x03,                                 /* baInterfaceNr(2) */
  0x04,                                 /* baInterfaceNr(3) */
  0x05,                                 /* baInterfaceNr(4) */
  0x06,                                 /* baInterfaceNr(5) */
  0x07,                                 /* baInterfaceNr(6) */
  0x08,                                 /* baInterfaceNr(7) */

  DECLARE_UNITS_OUT(USBD_AUDIO_STR_SPEAKER1, 0) /* 31 bytes */
  DECLARE_UNITS_IN(USBD_AUDIO_STR_MIC1, 1) /* 31 bytes */
  DECLARE_UNITS_OUT(USBD_AUDIO_STR_SPEAKER2, 2) /* 31 bytes */
  DECLARE_UNITS_IN(USBD_AUDIO_STR_MIC2, 3) /* 31 bytes */
  DECLARE_UNITS_OUT(USBD_AUDIO_STR_SPEAKER3, 4) /* 31 bytes */
  DECLARE_UNITS_IN(USBD_AUDIO_STR_MIC3, 5) /* 31 bytes */
  DECLARE_UNITS_OUT(USBD_AUDIO_STR_SPEAKER4, 6) /* 31 bytes */
  DECLARE_UNITS_IN(USBD_AUDIO_STR_MIC4, 7) /* 31 bytes */

  DECLARE_ENDPOINT_OUT(0, 0, USBD_AUDIO_CHANNELS, AUDIO_PACKET_SZE) /* 52 bytes */
  DECLARE_ENDPOINT_IN(1, 1, USBD_AUDIO_CHANNELS, AUDIO_PACKET_SZE) /* 52 bytes */
  DECLARE_ENDPOINT_OUT(2, 2, USBD_AUDIO_CHANNELS, AUDIO_PACKET_SZE) /* 52 bytes */
  DECLARE_ENDPOINT_IN(3, 3, USBD_AUDIO_CHANNELS, AUDIO_PACKET_SZE) /* 52 bytes */
  DECLARE_ENDPOINT_OUT(4, 4, USBD_AUDIO_CHANNELS, AUDIO_PACKET_SZE) /* 52 bytes */
  DECLARE_ENDPOINT_IN(5, 5, USBD_AUDIO_CHANNELS, AUDIO_PACKET_SZE) /* 52 bytes */
  DECLARE_ENDPOINT_OUT(6, 6, USBD_AUDIO_CHANNELS, AUDIO_PACKET_SZE) /* 52 bytes */
  DECLARE_ENDPOINT_IN(7, 7, USBD_AUDIO_CHANNELS, AUDIO_PACKET_SZE) /* 52 bytes */



  /*---------------------------------------------------------------------------*/

  /* Interface Association Descriptor */
  0x08,                                 /* bLength */
  0x0B,                                 /* bDescriptorType */
  9,                                    /* bFirstInterface */
  0x02,                                 /* bInterfaceCount */
  0x02,                                 /* bInterfaceClass: Communication Interface Class */
  0x02,                                 /* bInterfaceSubClass: Abstract Control Model */
  0x01,                                 /* bInterfaceProtocol: Common AT commands */
  0x00,                                 /* iFunction */

  /* Interface Descriptor */
  0x09,                                       /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
  /* Interface descriptor type */
  9,                                          /* bInterfaceNumber: Number of Interface */
  0x00,                                       /* bAlternateSetting: Alternate setting */
  0x01,                                       /* bNumEndpoints: One endpoint used */
  0x02,                                       /* bInterfaceClass: Communication Interface Class */
  0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
  0x00,                                       /* bInterfaceProtocol: No class specific protocol */
  0x00,                                       /* iInterface */

  /* Header Functional Descriptor */
  0x05,                                       /* bLength: Endpoint Descriptor size */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x00,                                       /* bDescriptorSubtype: Header Func Desc */
  0x10,                                       /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,                                       /* bFunctionLength */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
  0x00,                                       /* bmCapabilities: D0+D1 */
  10,                                         /* bDataInterface */

  /* ACM Functional Descriptor */
  0x04,                                       /* bFunctionLength */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,                                       /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,                                       /* bFunctionLength */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x06,                                       /* bDescriptorSubtype: Union func desc */
  9,                                          /* bMasterInterface: Communication class interface */
  10,                                         /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                                 /* bEndpointAddress */
  0x03,                                       /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  CDC_HS_BINTERVAL,                           /* bInterval */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
  10,                                         /* bInterfaceNumber: Number of Interface */
  0x00,                                       /* bAlternateSetting: Alternate setting */
  0x02,                                       /* bNumEndpoints: Two endpoints used */
  0x0A,                                       /* bInterfaceClass: CDC */
  0x00,                                       /* bInterfaceSubClass */
  0x00,                                       /* bInterfaceProtocol */
  0x00,                                       /* iInterface */

  /* Endpoint OUT Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                                 /* bEndpointAddress */
  0x02,                                       /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),        /* wMaxPacketSize */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
  0x00,                                       /* bInterval */

  /* Endpoint IN Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  CDC_IN_EP,                                  /* bEndpointAddress */
  0x02,                                       /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),        /* wMaxPacketSize */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
  0x00                                        /* bInterval */
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00, /* bcdUSB */
  0x02, /* bcdUSB */
  0x00, /* bDeviceClass */
  0x00, /* bDeviceSubClass */
  0x00, /* bDeviceProtocol */
  0x40, /* bMaxPacketSize0 */
  0x01, /* idVendor */
  0x00, /* bDeviceSubClass */
};

static enum USBD_COMPOSITE_ClassId interface_mapping[] = {
	[0] = CI_AudioClass,
	[1] = CI_AudioClass,
	[2] = CI_AudioClass,
	[3] = CI_AudioClass,
	[4] = CI_AudioClass,
	[5] = CI_AudioClass,
	[6] = CI_AudioClass,
	[7] = CI_AudioClass,
	[8] = CI_AudioClass,
	[9] = CI_CDCClass,
	[10] = CI_CDCClass,
};

static enum USBD_COMPOSITE_ClassId endpoint_mapping[] = {
	[1] = CI_AudioClass,
	[2] = CI_AudioClass,
	[3] = CI_AudioClass,
	[4] = CI_AudioClass,
	[5] = CI_CDCClass,
	[6] = CI_CDCClass,
};

/**
  * @}
  */

/** @defgroup USBD_COMPOSITE_Private_Functions
  * @{
  */

#define CALL_CLASS_FUNCTION(ret_variable_, pdev_, classId_, func_, ...) \
	do { \
		pdev_->classId = classId_; \
		USBD_ClassTypeDef* classTypeDef = ((USBD_COMPOSITE_HandleTypeDef *)pdev_->pClassDataCmsit[CI_Composite])->classes[classId_]; \
		if(classTypeDef && classTypeDef->func_) {\
			ret_variable_ = classTypeDef->func_(__VA_ARGS__); \
		} \
		pdev_->classId = CI_Composite; \
	} while(0)

#define CALL_CLASS_PROCEDURE(pdev_, classId_, func_, ...) \
	do { \
		pdev_->classId = classId_; \
		USBD_ClassTypeDef* classTypeDef = ((USBD_COMPOSITE_HandleTypeDef *)pdev_->pClassDataCmsit[CI_Composite])->classes[classId_]; \
		if(classTypeDef && classTypeDef->func_) {\
			classTypeDef->func_(__VA_ARGS__); \
		} \
		pdev_->classId = CI_Composite; \
	} while(0)

/**
  * @brief  USBD_COMPOSITE_Init
  *         Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_COMPOSITE_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_COMPOSITE_HandleTypeDef *handle;

  handle = (USBD_COMPOSITE_HandleTypeDef *)USBD_malloc(sizeof(USBD_COMPOSITE_HandleTypeDef));

  if (handle == NULL)
  {
    pdev->pClassDataCmsit[CI_Composite] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  (void)USBD_memset(handle, 0, sizeof(USBD_COMPOSITE_HandleTypeDef));

  pdev->pClassDataCmsit[CI_Composite] = handle;

  handle->classes[CI_AudioClass] = &USBD_AUDIO;
  handle->classes[CI_CDCClass] = &USBD_CDC;

  for(int i = 0; i < CI_NUMBER; i++) {
	if(handle->classes[i]) {
      CALL_CLASS_PROCEDURE(pdev, i, Init, pdev, cfgidx);
	}
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_COMPOSITE_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_COMPOSITE_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_COMPOSITE_HandleTypeDef *handle = (USBD_COMPOSITE_HandleTypeDef *)pdev->pClassDataCmsit[CI_Composite];
  if(handle == NULL)
	  return USBD_OK;

  for(int i = 0; i < CI_NUMBER; i++) {
	if(handle->classes[i]) {
	  CALL_CLASS_PROCEDURE(pdev, i, DeInit, pdev, cfgidx);
	}
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_COMPOSITE_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_COMPOSITE_Setup(USBD_HandleTypeDef *pdev,
                              USBD_SetupReqTypedef *req)
{
  USBD_COMPOSITE_HandleTypeDef *handle = (USBD_COMPOSITE_HandleTypeDef *)pdev->pClassDataCmsit[CI_Composite];
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;
  enum USBD_COMPOSITE_ClassId classId;

  if (handle == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch(req->bmRequest & USB_REQ_RECIPIENT_MASK) {
      case USB_REQ_RECIPIENT_INTERFACE:
          classId = interface_mapping[req->wIndex & 0xFF];
          CALL_CLASS_FUNCTION(ret, pdev, classId, Setup, pdev, req);
          break;
      case USB_REQ_RECIPIENT_ENDPOINT:
          classId = endpoint_mapping[req->wIndex & 0x0F];
          CALL_CLASS_FUNCTION(ret, pdev, classId, Setup, pdev, req);
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
            CALL_CLASS_FUNCTION(ret, pdev, CI_AudioClass, Setup, pdev, req);
          }
          break;

        case USB_REQ_GET_INTERFACE:
          classId = interface_mapping[req->wIndex];
          CALL_CLASS_FUNCTION(ret, pdev, classId, Setup, pdev, req);
          break;

        case USB_REQ_SET_INTERFACE:
          classId = interface_mapping[req->wIndex];
          CALL_CLASS_FUNCTION(ret, pdev, classId, Setup, pdev, req);
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
  * @brief  USBD_COMPOSITE_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_COMPOSITE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint32_t classId = endpoint_mapping[epnum & 0x0F];
  uint8_t ret = USBD_OK;
  CALL_CLASS_FUNCTION(ret, pdev, classId, DataIn, pdev, epnum);
  return ret;
}

/**
  * @brief  USBD_COMPOSITE_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_COMPOSITE_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint32_t classId = endpoint_mapping[epnum & 0x0F];
  uint8_t ret = USBD_OK;
  CALL_CLASS_FUNCTION(ret, pdev, classId, DataOut, pdev, epnum);
  return ret;
}

/**
  * @brief  USBD_AUDIO_OutTokenWhileDisabled
  *         handle OUT token while endpoint disabled event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_COMPOSITE_OutTokenWhileDisabled(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint32_t classId = endpoint_mapping[epnum & 0x0F];
  uint8_t ret = USBD_OK;
  CALL_CLASS_FUNCTION(ret, pdev, classId, OutTokenWhileDisabled, pdev, epnum);
  return ret;
}

/**
  * @brief  USBD_COMPOSITE_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  uint32_t classId = pdev->ep0RxClassId;
  uint8_t ret = USBD_OK;
  CALL_CLASS_FUNCTION(ret, pdev, classId, EP0_RxReady, pdev);
  return ret;
}

static uint8_t USBD_COMPOSITE_SOF(USBD_HandleTypeDef *pdev) {
  USBD_COMPOSITE_HandleTypeDef *handle = (USBD_COMPOSITE_HandleTypeDef *)pdev->pClassDataCmsit[CI_Composite];
  for(int i = 0; i < CI_NUMBER; i++) {
	  if(handle->classes[i]) {
		 CALL_CLASS_PROCEDURE(pdev, i, SOF, pdev);
	  }
  }

  return USBD_OK;
}

static uint8_t USBD_COMPOSITE_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum) {
  uint32_t classId = endpoint_mapping[epnum & 0x0F];
  uint8_t ret = USBD_OK;
  CALL_CLASS_FUNCTION(ret, pdev, classId, IsoINIncomplete, pdev, epnum);
  return ret;
}
static uint8_t USBD_COMPOSITE_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum) {
  uint32_t classId = endpoint_mapping[epnum & 0x0F];
  uint8_t ret = USBD_OK;
  CALL_CLASS_FUNCTION(ret, pdev, classId, IsoOUTIncomplete, pdev, epnum);
  return ret;
}

static uint8_t *USBD_COMPOSITE_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_COMPOSITE_CfgDesc);

  return USBD_COMPOSITE_CfgDesc;
}

/**
  * @brief  USBD_COMPOSITE_GetDeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_COMPOSITE_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_COMPOSITE_DeviceQualifierDesc);

  return USBD_COMPOSITE_DeviceQualifierDesc;
}




static uint8_t USBD_COMPOSITE_UsrStrDescriptor[0x200];

static uint8_t* USBD_COMPOSITE_GetString(const char* str, uint16_t* length) {
	USBD_GetString((uint8_t*) str, USBD_COMPOSITE_UsrStrDescriptor, length);
	return USBD_COMPOSITE_UsrStrDescriptor;
}

/**
 * @brief  Return the interface string descriptor
 * @param  speed : Current device speed
 * @param  length : Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
static uint8_t* USBD_COMPOSITE_GetUsrStrDescriptor(struct _USBD_HandleTypeDef *pdev, uint8_t index,  uint16_t *length) {
	int user_index = index - USBD_AUDIO_STR_FIRST_INDEX;

	static const char* USER_STRINGS[] = {
	    "Line Out 1",
	    "Line Out 2",
	    "Line Out 3",
	    "Line Out 4",
	    "Line In 1",
	    "Line In 2",
	    "Line In 3",
	    "Line In 4",
	};

	if(user_index < 0 || user_index >= sizeof(USER_STRINGS) / sizeof(USER_STRINGS[0])) {
		return USBD_COMPOSITE_GetString("", length);
	}

	return USBD_COMPOSITE_GetString(USER_STRINGS[user_index], length);
}

