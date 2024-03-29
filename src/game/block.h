#ifndef B173C_BLOCK_H
#define B173C_BLOCK_H

#include "common.h"
#include "block_ids.h"
#include "mathlib.h"

typedef enum {
    BLOCK_FACE_Y_NEG = 0,
    BLOCK_FACE_Y_POS = 1,
    BLOCK_FACE_Z_NEG = 2,
    BLOCK_FACE_Z_POS = 3,
    BLOCK_FACE_X_NEG = 4,
    BLOCK_FACE_X_POS = 5
} block_face;

#define IS_X_FACE(f)    ((f) == BLOCK_FACE_X_NEG || (f) == BLOCK_FACE_X_POS)
#define IS_Y_FACE(f)    ((f) == BLOCK_FACE_Y_NEG || (f) == BLOCK_FACE_Y_POS)
#define IS_Z_FACE(f)    ((f) == BLOCK_FACE_Z_NEG || (f) == BLOCK_FACE_Z_POS)
#define IS_SIDE_FACE(f) (!(IS_Y_FACE((f))))
#define IS_POS_FACE(f)  (((f) & 1) != 0)
#define IS_NEG_FACE(f)  (((f) & 1) == 0)

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
    float hardness;
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
     (b).id == BLOCK_LAVA_STILL  || (b).id == BLOCK_LAVA_MOVING)


#define block_is_transparent(b) (block_get_properties((b).id).opaque == 0)

#define block_is_semi_transparent(b)                                \
    ((b).id == BLOCK_WATER_STILL || (b).id == BLOCK_WATER_MOVING || \
     (b).id == BLOCK_ICE)

block_properties block_get_properties(block_id id);
int block_get_texture_index(block_id id, block_face face, ubyte metadata, int x, int y, int z);
bool block_should_render_face(int x, int y, int z, block_data self, block_face face);
float block_fluid_get_percent_air(ubyte metadata);
float block_fluid_get_height(int x, int y, int z, block_id self_id);
vec3_t block_fluid_get_flow_direction(int x, int y, int z);
bbox_t block_get_bbox(block_data self, int x, int y, int z, bool selectmode);
bool block_is_collidable(block_data block);
bool block_is_selectable(block_data block);
bool block_is_flammable(block_id id);
const char *block_face_to_str(block_face f);

#define METADATA_DOOR_ROT (3)
#define METADATA_DOOR_OPEN (4)
#define METADATA_DOOR_BOTTOM (8)

#define METADATA_BED_PILLOW (8)
#define METADATA_BED_ROT (3)

#define METADATA_REPEATER_ROT (3)
#define METADATA_REPEATER_TICK (12)

#endif
