#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define SERIAL_MAX_PAYLOAD 128u
#define SERIAL_MAX_FRAME (SERIAL_MAX_PAYLOAD+10u)
enum { SERIAL_SYNC_REQUEST=1, SERIAL_SYNC_RESPONSE=2, SERIAL_COMMAND=3,
       SERIAL_STATUS=4, SERIAL_ENCODERS=5, SERIAL_IMU=6, SERIAL_METAL=7 };
typedef struct { uint8_t data[SERIAL_MAX_FRAME]; size_t used; } serial_parser_t;
typedef void (*serial_receive_fn)(uint8_t type,const uint8_t *payload,size_t len,void *context);
uint32_t serial_crc32(const uint8_t *data,size_t len);
size_t serial_encode(uint8_t type,const void *payload,size_t len,uint8_t *frame);
void serial_feed(serial_parser_t *parser,const uint8_t *data,size_t len,
                 serial_receive_fn receive,void *context);
#endif
