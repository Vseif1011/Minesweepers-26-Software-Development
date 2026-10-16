#ifndef USBD_CONF_H
#define USBD_CONF_H
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#define USBD_MAX_NUM_INTERFACES 1
#define USBD_MAX_NUM_CONFIGURATION 1
#define USBD_MAX_STR_DESC_SIZ 128
#define USBD_DEBUG_LEVEL 0
#define USBD_LPM_ENABLED 0
#define USBD_SELF_POWERED 0
#define DEVICE_FS 0
#define USBD_SUPPORT_USER_STRING_DESC 0
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *pointer);
#define USBD_malloc USBD_static_malloc
#define USBD_free USBD_static_free
#define USBD_memset memset
#define USBD_memcpy memcpy
#define USBD_Delay HAL_Delay
#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)
#endif
