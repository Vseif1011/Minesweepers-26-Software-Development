#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H
#include "usbd_cdc.h"
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
uint8_t CDC_Transmit_FS(uint8_t *buf,uint16_t len);
void transport_rx(const uint8_t *buf,uint32_t len);
#endif
