#ifndef B173C_NET_H
#define B173C_NET_H

#include <stdint.h>
#include <uchar.h>
#include "common.h"

typedef struct {
    int16_t length;
    char16_t *data;
} string16;

typedef struct {
    int16_t length;
    char *data;
} string8;

struct net_entity_metadata {
    int idk;
};

errcode net_init(void);
void net_process(void);
void net_write_packets(void);
void net_shutdown(void);

#endif
