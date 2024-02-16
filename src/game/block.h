#ifndef B173C_BLOCK_H
#define B173C_BLOCK_H

#include "common.h"
#include "block_ids.h"
#include "mathlib.h"

typedef enum {
    BLOCK_FACE_Y_NEG,
    BLOCK_FACE_Y_POS,
    BLOCK_FACE_Z_NEG,
    BLOCK_FACE_Z_POS,
    BLOCK_FACE_X_NEG,
    BLOCK_FACE_X_POS
} block_face;

#define IS_SIDE_FACE(f)   ((f) != BLOCK_FACE_Y_NEG && (f) != BLOCK_FACE_Y_POS)
#define IS_TOPBOT_FACE(f) ((f) == BLOCK_FACE_Y_NEG || (f) == BLOCK_FACE_Y_POS)

typedef enum {
    RENDER_CUBE,
    RENDER_CROSS,
    RENDER_TORCH,
    RENDER_FIRE,
    RENDER_FLUID,
    RENDER_WIRE,
    RENDER_CROPS,
    RENDER_DOOR,
    RENDER_LADDER,
    RENDER_RAIL,
    RENDER_STAIRS,
    RENDER_FENCE,
    RENDER_LEVER,
    RENDER_CACTUS,
    RENDER_BED,
    RENDER_REPEATER,
    RENDER_PISTON_BASE,
    RENDER_PISTON_EXTENSION,
    RENDER_CUBE_SPECIAL,
    RENDER_NONE,
    RENDER_TYPE_COUNT
} block_render_type;

typedef ubyte block_id;

typedef struct {
    block_id id;
    ubyte metadata   : 4;
    ubyte skylight   : 4;
    ubyte blocklight : 4;
} block_data;

typedef struct {
    char *name;
    ubyte texture_indices[6];
    bool opaque;
    block_render_type render_type;
} block_properties;

// :P fixme !
#define block_is_solid(b)                                            \
    ((b).id != 0 && (b).id != 9 && (b).id != 10 && (b).id != 11 &&   \
     (b).id != 27 && (b).id != 28 && (b).id != 31 && (b).id != 32 && \
     (b).id != 37 && (b).id != 38 && (b).id != 39 && (b).id != 40 && \
     (b).id != 50 && (b).id != 51 && (b).id != 55 && (b).id != 59 && \
     (b).id != 65 && (b).id != 66 && (b).id != 69 && (b).id != 75 && \
     (b).id != 76 && (b).id != 77 && (b).id != 78 && (b).id != 83 && \
     (b).id != 90 && (b).id != 93 && (b).id != 94)
#define block_is_empty(b) ((b).id == BLOCK_AIR)
#define block_is_liquid(b)                                          \
    ((b).id == BLOCK_WATER_STILL || (b).id == BLOCK_WATER_MOVING || \
     (b).id == BLOCK_LAVA_STILL || (b).id == BLOCK_LAVA_MOVING)
#define block_is_collidable(b)  ((b).id != BLOCK_AIR && !block_is_liquid(b))
#define block_is_transparent(b) (block_get_properties(b.id).opaque == 0)
#define block_is_semi_transparent(b)                                \
    ((b).id == BLOCK_WATER_STILL || (b).id == BLOCK_WATER_MOVING || \
     (b).id == BLOCK_ICE)
block_properties block_get_properties(block_id id);
ubyte block_get_texture_index(
    block_id id, block_face face, ubyte metadata, int x, int y, int z);
bool block_should_face_be_rendered(
    int x, int y, int z, block_data self, block_face face);
float block_fluid_get_height(int x, int y, int z, block_id self_id);

#endif
