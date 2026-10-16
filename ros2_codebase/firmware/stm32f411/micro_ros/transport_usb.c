/* Custom HAL USB CDC transport following micro_ros_stm32cubemx_utils.
 * Differences: detect RX overflow, never overwrite live DMA/USB TX storage,
 * honor timeouts, no unbounded waits. Sole reader/writer: microROS task. */
#include "rmw_microros/rmw_microros.h"
#include "app.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
#define RX_CAPACITY 2048u
#define TX_CAPACITY 512u
static uint8_t rx[RX_CAPACITY],tx[TX_CAPACITY];
static volatile uint32_t head,tail;
static volatile bool overflow;
void transport_rx(const uint8_t *buf,uint32_t len) {
  uint32_t h=head,t=tail;
  if (len>RX_CAPACITY-(t-h)) { overflow=true;return; }
  for (uint32_t i=0;i<len;++i) rx[(t+i)%RX_CAPACITY]=buf[i];
  __DMB();tail=t+len;
}
bool cubemx_transport_open(struct uxrCustomTransport *transport) {
  (void)transport;
  uint32_t mask=__get_PRIMASK();__disable_irq();head=tail=0;overflow=false;__set_PRIMASK(mask);
  return true;
}
bool cubemx_transport_close(struct uxrCustomTransport *transport) { (void)transport;return true; }
static bool tx_busy(void) {
  return hUsbDeviceFS.pClassData && ((USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData)->TxState;
}
size_t cubemx_transport_write(struct uxrCustomTransport *transport,const uint8_t *buf,size_t len,uint8_t *err) {
  (void)transport;*err=0;
  if (len>TX_CAPACITY || hUsbDeviceFS.dev_state!=USBD_STATE_CONFIGURED) { *err=1;return 0; }
  uint32_t start=HAL_GetTick();
  while (tx_busy()) {
    if ((uint32_t)(HAL_GetTick()-start)>=20) { *err=1;return 0; }
    vTaskDelay(1);
  }
  memcpy(tx,buf,len);
  if (CDC_Transmit_FS(tx,(uint16_t)len)!=USBD_OK) { *err=1;return 0; }
  while (tx_busy()) {
    if ((uint32_t)(HAL_GetTick()-start)>=20) { *err=1;return 0; }
    vTaskDelay(1);
  }
  return len;
}
size_t cubemx_transport_read(struct uxrCustomTransport *transport,uint8_t *buf,size_t len,int timeout,uint8_t *err) {
  (void)transport;*err=0;uint32_t start=HAL_GetTick();
  if (!len) return 0;
  do {
    if (overflow) { *err=1;return 0; }
    uint32_t h=head,t=tail;__DMB();
    if (h!=t) {
      size_t count=t-h;if (count>len) count=len;
      for (size_t i=0;i<count;++i) buf[i]=rx[(h+i)%RX_CAPACITY];
      __DMB();head=h+count;return count;
    }
    if (timeout<=0) break;
    vTaskDelay(1);
  } while ((uint32_t)(HAL_GetTick()-start)<(uint32_t)timeout);
  return 0;
}
