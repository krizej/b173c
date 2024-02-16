#ifndef B173C_PACKETS_H
#define B173C_PACKETS_H

#include <stdint.h>
#include <stdbool.h>
#include "net_internal.h"
#include "net.h"

/// data type macros
#define UBYTE(name)                uint8_t name;
#define BYTE(name)                 int8_t name;
#define SHORT(name)                int16_t name;
#define INT(name)                  int32_t name;
#define LONG(name)                 int64_t name;
#define FLOAT(name)                float name;
#define DOUBLE(name)               double name;
#define STRING8(name)              string8 name;
#define STRING16(name)             string16 name;
#define BOOL(name)                 bool name;
#define METADATA(name)             struct net_entity_metadata name;
#define WINDOW_ITEMS_PAYLOAD(name) struct ni_wi_payload name;
/// advanced functionality macros
#define OPT(cond, stuff)           stuff
#define BUF(type, name, size)      type(*name)
/// struct and handler/writer function prototypes
#define PACKET(id, name, stuff)             \
    typedef struct {                        \
        stuff                               \
    } pkt_##name;                           \
    void net_handle_pkt_##name(pkt_##name); \
    void net_write_pkt_##name(pkt_##name);

#include "packets_def.h"

#undef UBYTE
#undef BYTE
#undef SHORT
#undef INT
#undef LONG
#undef FLOAT
#undef DOUBLE
#undef STRING8
#undef STRING16
#undef BOOL
#undef METADATA
#undef WINDOW_ITEMS_PAYLOAD
#undef OPT
#undef BUF
#undef PACKET

#endif
