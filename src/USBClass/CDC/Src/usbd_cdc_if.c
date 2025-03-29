/**
  ******************************************************************************
  * @file    usbd_cdc_if_template.c
  * @author  MCD Application Team
  * @brief   Generic media access Layer.
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

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include <assert.h>

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */

static int8_t USB_CDC_IF_Init(USBD_HandleTypeDef *pdev);
static int8_t USB_CDC_IF_DeInit(void);
static int8_t USB_CDC_IF_Control(USBD_SetupReqTypedef *req, uint8_t *pbuf, uint16_t length);
static int8_t USB_CDC_IF_Receive(uint8_t *pbuf, uint32_t *Len);
static int8_t USB_CDC_IF_TransmitCplt(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_CDC_IF_fops = {
  USB_CDC_IF_Init,
  USB_CDC_IF_DeInit,
  USB_CDC_IF_Control,
  USB_CDC_IF_Receive,
  USB_CDC_IF_TransmitCplt,
};

static USBD_CDC_LineCodingTypeDef linecoding = {
  115200, /* baud rate*/
  0x00,   /* stop bits-1*/
  0x00,   /* parity - none*/
  0x08    /* nb. of bits 8*/
};

struct USBD_CDC_CircularBuffer
{
  uint32_t sentinel;
  uint32_t max_write_size;
  volatile uint32_t current_theorical_size;  // Size without counting discarded data
  uint16_t write_index;
  uint16_t read_index;
  uint8_t buffer[CDC_DATA_HS_MAX_PACKET_SIZE * 2];
  uint8_t usb_buffer[CDC_DATA_HS_MAX_PACKET_SIZE];
  volatile uint8_t usb_busy;
  uint8_t usb_idle;  // USB didn't send data for more than 10ms
};

static struct USBD_CDC_CircularBuffer rxBuffer;
static struct USBD_CDC_CircularBuffer txBuffer;
static USBD_HandleTypeDef *usb_pdev;

static USB_CDC_IF_notifyDataReady userDataReadyCallback = NULL;
static void *userDataReadyCallbackArg = NULL;

/* Private functions ---------------------------------------------------------*/

static void USB_CDC_IF_receiveIfReady()
{
  if (rxBuffer.usb_busy || !usb_pdev)
  {
    return;
  }

  uint16_t max_size = (rxBuffer.read_index - rxBuffer.write_index - 1 + sizeof(rxBuffer.buffer)) % sizeof(rxBuffer.buffer);
  if (max_size >= CDC_DATA_HS_OUT_PACKET_SIZE)
  {
    // Enough room to receive a packet, start reception
    rxBuffer.usb_busy = 1;
    USBD_CDC_ReceivePacket(usb_pdev);
  }
  else
  {
    // Application didn't read enough data, execute data ready callback again so it reads more data later
    if (userDataReadyCallback)
      userDataReadyCallback(userDataReadyCallbackArg);
  }
}

static void USB_CDC_IF_sendPending()
{
  if (txBuffer.usb_busy || !usb_pdev)
  {
    return;
  }

  uint16_t start = txBuffer.read_index;
  uint16_t end = txBuffer.write_index;
  uint16_t size = (end - start + sizeof(txBuffer.buffer)) % sizeof(txBuffer.buffer);

  assert(start < sizeof(txBuffer.buffer));
  assert(end < sizeof(txBuffer.buffer));

  if (size > sizeof(txBuffer.usb_buffer))
  {
    size = sizeof(txBuffer.usb_buffer);
    end = (txBuffer.read_index + size) % sizeof(txBuffer.buffer);
  }

  assert(end < sizeof(txBuffer.buffer));

  uint16_t total_size = 0;

  if (end < start)
  {
    // Copy between start and end of buffer
    uint16_t first_chunk_size = sizeof(txBuffer.buffer) - start;
    memcpy(txBuffer.usb_buffer, &txBuffer.buffer[start], first_chunk_size);

    // then between begin of buffer and end
    memcpy(&txBuffer.usb_buffer[first_chunk_size], &txBuffer.buffer[0], end);

    total_size = first_chunk_size + end;
    assert(total_size <= sizeof(txBuffer.usb_buffer));
  }
  else
  {
    // Copy from start to end
    memcpy(txBuffer.usb_buffer, &txBuffer.buffer[start], end - start);
    total_size = end - start;
    assert(total_size <= sizeof(txBuffer.usb_buffer));
  }

  __DSB();
  txBuffer.read_index = end;


  if (total_size > 0)
  {
    txBuffer.usb_busy = 1;
    txBuffer.current_theorical_size -= total_size;
    USBD_CDC_SetTxBuffer(usb_pdev, txBuffer.usb_buffer, total_size);
    USBD_CDC_TransmitPacket(usb_pdev);
  }
  else
  {
    txBuffer.current_theorical_size = 0;
  }
}

uint32_t usb_cdc_timeout;
static void USB_CDC_IF_BUFFER_write_char(struct USBD_CDC_CircularBuffer *buffer, uint8_t c)
{
  assert(buffer->write_index < sizeof(buffer->buffer));
  assert(buffer->read_index < sizeof(buffer->buffer));

  uint16_t next_write_index = (buffer->write_index + 1) % sizeof(buffer->buffer);

  // Block until available space
  if (next_write_index == buffer->read_index)
  {
    // If already idle since a previous write and still idle, don't wait at all
    if (buffer->usb_idle)
    {
      return;
    }

    buffer->usb_idle = 1;
    USB_CDC_IF_sendPending();

    // 10ms timeout, if USB didn't sent anything, don't wait anymore to avoid blocking everything
    uint32_t timeout = HAL_GetTick() + 1000;
    while (next_write_index == buffer->read_index && HAL_GetTick() < timeout)
    {
      __DSB();
    }

    if (next_write_index == buffer->read_index)
    {
      usb_cdc_timeout = 1;
      return;
    }

    buffer->usb_idle = 0;
  }

  // Buffer not full
  buffer->buffer[buffer->write_index] = c;
  assert(next_write_index < sizeof(buffer->buffer));
  __DSB();
  buffer->write_index = next_write_index;
}

static uint8_t USB_CDC_IF_BUFFER_read_char(struct USBD_CDC_CircularBuffer *buffer, uint8_t *c)
{
  uint32_t read_index;

retry:
  read_index = buffer->read_index;
  if (read_index == buffer->write_index)
  {
    // Buffer empty
    return 0;
  }

  assert(read_index < sizeof(buffer->buffer));

  *c = buffer->buffer[read_index];
  __DSB();
  uint32_t next_index = (read_index + 1) % sizeof(buffer->buffer);

  if (!__sync_bool_compare_and_swap(&buffer->read_index, read_index, next_index))
    goto retry;

  return 1;
}

/**
  * @brief  USB_CDC_IF_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USB_CDC_IF_Init(USBD_HandleTypeDef *pdev)
{
  /*
     Add your initialization code here
  */
  usb_pdev = pdev;

  txBuffer.usb_busy = 0;
  rxBuffer.usb_busy = 1;

  USBD_CDC_SetRxBuffer(pdev, rxBuffer.usb_buffer);
  USB_CDC_IF_sendPending();
  // No need to start receiving, this is already done in usbd_cdc.c

  return (0);
}

/**
  * @brief  USB_CDC_IF_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USB_CDC_IF_DeInit(void)
{
  /*
     Add your deinitialization code here
  */
  usb_pdev = NULL;
  return (0);
}


/**
  * @brief  USB_CDC_IF_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USB_CDC_IF_Control(USBD_SetupReqTypedef *req, uint8_t *pbuf, uint16_t length)
{
  UNUSED(length);

  switch (req->bRequest)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
      /* Add your code here */
      break;

    case CDC_GET_ENCAPSULATED_RESPONSE:
      /* Add your code here */
      break;

    case CDC_SET_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_GET_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_CLEAR_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_SET_LINE_CODING:
      linecoding.bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
      linecoding.format = pbuf[4];
      linecoding.paritytype = pbuf[5];
      linecoding.datatype = pbuf[6];

      /* Add your code here */
      break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(linecoding.bitrate);
      pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
      pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
      pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
      pbuf[4] = linecoding.format;
      pbuf[5] = linecoding.paritytype;
      pbuf[6] = linecoding.datatype;

      /* Add your code here */
      break;

    case CDC_SET_CONTROL_LINE_STATE:
      if ((req->wValue & 0x03) != 0)
      {
        // Start transmission, reset fifos here
        txBuffer.write_index = txBuffer.read_index = 0;
        rxBuffer.write_index = rxBuffer.read_index = 0;
      }
      /* Add your code here */
      break;

    case CDC_SEND_BREAK:
      /* Add your code here */
      break;

    default:
      break;
  }

  return (0);
}

/**
  * @brief  USB_CDC_IF_Receive
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USB_CDC_IF_Receive(uint8_t *Buf, uint32_t *Len)
{
  uint16_t start = rxBuffer.write_index;
  uint16_t size = *Len;
  uint16_t max_size = (rxBuffer.read_index - rxBuffer.write_index - 1 + sizeof(rxBuffer.buffer)) % sizeof(rxBuffer.buffer);

  rxBuffer.usb_busy = 0;

  rxBuffer.current_theorical_size += size;
  if (rxBuffer.max_write_size < rxBuffer.current_theorical_size)
    rxBuffer.max_write_size = rxBuffer.current_theorical_size;

  if (size > max_size)
    size = max_size;

  uint16_t end = (start + size) % sizeof(rxBuffer.buffer);

  assert(start < sizeof(rxBuffer.buffer));
  assert(end < sizeof(rxBuffer.buffer));


  if (end < start)
  {
    // Copy between start and end of buffer
    uint16_t first_chunk_size = sizeof(rxBuffer.buffer) - start;
    memcpy(&rxBuffer.buffer[start], rxBuffer.usb_buffer, first_chunk_size);

    // then between begin of buffer and end
    memcpy(&rxBuffer.buffer[0], &rxBuffer.usb_buffer[first_chunk_size], end);
  }
  else
  {
    // Copy from start to end
    memcpy(&rxBuffer.buffer[start], rxBuffer.usb_buffer, end - start);
  }

  assert(end < sizeof(rxBuffer.buffer));
  __DSB();
  rxBuffer.write_index = end;

  // Don't read again from here to avoid starving the audio processing with USB CDC reads
  //USB_CDC_IF_receiveIfReady();

  if (userDataReadyCallback)
    userDataReadyCallback(userDataReadyCallbackArg);

  return (0);
}

/**
  * @brief  USB_CDC_IF_TransmitCplt
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USB_CDC_IF_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  txBuffer.usb_busy = 0;
  txBuffer.usb_idle = 0;
  usb_cdc_timeout = 0;

  USB_CDC_IF_sendPending();

  return (0);
}


void USB_CDC_IF_TX_write(const uint8_t *Buf, uint32_t Len)
{
  if (!usb_pdev)
    return;

  size_t i;
  for (i = 0; i < Len; i++)
  {
    USB_CDC_IF_BUFFER_write_char(&txBuffer, Buf[i]);
  }

  txBuffer.current_theorical_size += Len;
  if (txBuffer.max_write_size < txBuffer.current_theorical_size)
    txBuffer.max_write_size = txBuffer.current_theorical_size;

  USB_CDC_IF_sendPending();
}

uint32_t USB_CDC_IF_RX_read(uint8_t *Buf, uint32_t max_len)
{
  size_t i = 0;

  while (i < max_len && USB_CDC_IF_BUFFER_read_char(&rxBuffer, &Buf[i]))
  {
    i++;
  }

  // Trigger receive again after leaving more room in the buffer
  USB_CDC_IF_receiveIfReady();

  if (rxBuffer.read_index == rxBuffer.write_index)
  {
    rxBuffer.current_theorical_size = 0;
  }
  else
  {
    rxBuffer.current_theorical_size -= i;
  }

  return i;
}

void USB_CDC_IF_set_rx_data_ready_callback(USB_CDC_IF_notifyDataReady callback, void *arg)
{
  userDataReadyCallback = callback;
  userDataReadyCallbackArg = arg;
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
