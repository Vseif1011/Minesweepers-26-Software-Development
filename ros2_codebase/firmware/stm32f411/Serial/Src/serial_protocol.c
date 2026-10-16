#include "serial_protocol.h"
#include <string.h>
uint32_t serial_crc32(const uint8_t *data,size_t len) {
  uint32_t crc=UINT32_MAX;
  for (size_t i=0;i<len;++i) {
    crc^=data[i];
    for (unsigned b=0;b<8;++b) crc=(crc>>1)^((crc&1)?UINT32_C(0xedb88320):0);
  }
  return ~crc;
}
size_t serial_encode(uint8_t type,const void *payload,size_t len,uint8_t *frame) {
  if (len>SERIAL_MAX_PAYLOAD) return 0;
  frame[0]=0xa5;frame[1]=0x5a;frame[2]=1;frame[3]=type;
  frame[4]=(uint8_t)len;frame[5]=(uint8_t)(len>>8);
  if (len) memcpy(frame+6,payload,len);
  uint32_t crc=serial_crc32(frame+2,len+4);
  for (unsigned i=0;i<4;++i) frame[len+6+i]=(uint8_t)(crc>>(8*i));
  return len+10;
}
static void discard(serial_parser_t *p) {
  memmove(p->data,p->data+1,--p->used);
}
void serial_feed(serial_parser_t *p,const uint8_t *data,size_t len,
                 serial_receive_fn receive,void *context) {
  for (size_t i=0;i<len;++i) {
    if (p->used==SERIAL_MAX_FRAME) discard(p);
    p->data[p->used++]=data[i];
    while (p->used>=2) {
      if (p->data[0]!=0xa5 || p->data[1]!=0x5a) { discard(p);continue; }
      if (p->used<6) break;
      size_t count=(size_t)p->data[4]|((size_t)p->data[5]<<8);
      if (p->data[2]!=1 || count>SERIAL_MAX_PAYLOAD) { discard(p);continue; }
      if (p->used<count+10) break;
      uint32_t crc=0;
      for (unsigned b=0;b<4;++b) crc|=(uint32_t)p->data[count+6+b]<<(8*b);
      if (crc!=serial_crc32(p->data+2,count+4)) { discard(p);continue; }
      receive(p->data[3],p->data+6,count,context);
      p->used-=count+10;
      memmove(p->data,p->data+count+10,p->used);
    }
  }
}
