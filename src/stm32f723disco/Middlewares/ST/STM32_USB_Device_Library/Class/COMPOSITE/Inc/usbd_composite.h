/**
  ******************************************************************************
  * @file    usbd_cdc.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_cdc.c file.
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
#ifndef __USB_COMPOSITE_H
#define __USB_COMPOSITE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"

enum USBD_COMPOSITE_ClassId
{
  CI_Composite,
  CI_AudioClass,
  CI_CDCClass,
  CI_NUMBER,
};

typedef struct
{
  USBD_ClassTypeDef *classes[CI_NUMBER];
} USBD_COMPOSITE_HandleTypeDef;


extern USBD_ClassTypeDef USBD_COMPOSITE;
#define USBD_COMPOSITE_CLASS &USBD_COMPOSITE


/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_COMPOSITE_H */
/**
  * @}
  */

/**
  * @}
  */
