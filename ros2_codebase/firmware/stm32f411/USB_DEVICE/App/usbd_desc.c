#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "board.h"
/* ST example VID/PID for local commissioning; allocate identifiers before distribution. */
static uint8_t device[]={18,USB_DESC_TYPE_DEVICE,0x00,0x02,0x02,0x02,0x00,64,
  0x83,0x04,0x40,0x57,0x00,0x01,1,2,3,1};
static uint8_t lang[]={4,USB_DESC_TYPE_STRING,0x09,0x04};
static uint8_t string_desc[USBD_MAX_STR_DESC_SIZ];
static uint8_t *dev(USBD_SpeedTypeDef speed,uint16_t *len) { (void)speed;*len=sizeof(device);return device; }
static uint8_t *language(USBD_SpeedTypeDef speed,uint16_t *len) { (void)speed;*len=sizeof(lang);return lang; }
static uint8_t *str(const char *s,uint16_t *len) { USBD_GetString((uint8_t*)s,string_desc,len);return string_desc; }
static uint8_t *manufacturer(USBD_SpeedTypeDef speed,uint16_t *len) { (void)speed;return str("Minesweepers Team A",len); }
static uint8_t *product(USBD_SpeedTypeDef speed,uint16_t *len) { (void)speed;return str("STM32 direct serial",len); }
static uint8_t *serial(USBD_SpeedTypeDef speed,uint16_t *len) {
  (void)speed;static char uid[25]; const char hex[]="0123456789ABCDEF";
  uint32_t words[]={HAL_GetUIDw0(),HAL_GetUIDw1(),HAL_GetUIDw2()};
  for (unsigned w=0;w<3;++w) for (unsigned n=0;n<8;++n) uid[w*8+n]=hex[(words[w]>>(28-4*n))&15];
  uid[24]=0;return str(uid,len);
}
static uint8_t *configuration(USBD_SpeedTypeDef s,uint16_t *len) { (void)s;return str("CDC",len); }
static uint8_t *interface(USBD_SpeedTypeDef s,uint16_t *len) { (void)s;return str("Minesweeper serial v1",len); }
USBD_DescriptorsTypeDef FS_Desc={dev,language,manufacturer,product,serial,configuration,interface};
