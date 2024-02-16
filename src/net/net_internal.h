#ifndef B173C_NET_INTERNAL_H
#define B173C_NET_INTERNAL_H

#include "common.h"
#include "net.h"

#define PROTOCOL_VERSION 14

// ni - net internal
// wi - window items
struct ni_wi_payload {
    int16_t item_id, metadata;
    int8_t count;
};

void net_read_buf(void *dest, size_t n);
byte net_read_byte(void);
short net_read_short(void);
int net_read_int(void);
long net_read_long(void);
float net_read_float(void);
double net_read_double(void);
bool net_read_bool(void);
string8 net_read_string8(void);
string16 net_read_string16(void);

void net_write_buf(const void *buf, size_t n);
void net_write_byte(ubyte v);
void net_write_short(short v);
void net_write_int(int v);
void net_write_long(long v);
void net_write_float(float v);
void net_write_double(double v);
void net_write_bool(bool v);
void net_write_string8(string8 v);
void net_write_string16(string16 v);

void net_free_string8(string8 v);
void net_free_string16(string16 v);

string8 net_make_string8(const char *text);
string16 net_make_string16(const char *text);

#endif
