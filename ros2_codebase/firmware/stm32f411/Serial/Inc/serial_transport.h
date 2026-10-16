#ifndef SERIAL_TRANSPORT_H
#define SERIAL_TRANSPORT_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void serial_transport_reset(void);
bool serial_transport_read(uint8_t *buf,size_t capacity,size_t *length);
bool serial_transport_write(const uint8_t *buf,size_t length);
bool serial_transport_configured(void);
#endif
