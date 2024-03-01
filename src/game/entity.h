#ifndef B173C_ENTITY_H
#define B173C_ENTITY_H

#include "mathlib.h"
#include "hashmap.c/hashmap.h"
#include "block.h"

typedef enum {
    ENTITY_ITEM = 1,
    ENTITY_FISHING_PARTICLE = 2,
    ENTITY_THUNDERBOLT = 3,

    ENTITY_PAINTING = 9,

    ENTITY_ARROW = 10,
    ENTITY_ITEM_PROJECTILE = 11,

    ENTITY_TNTPRIMED = 20,
    ENTITY_FALLINGBLOCK = 21,

    ENTITY_MINECART = 40,
    ENTITY_BOAT = 41,

    ENTITY_LIVING = 48,
    ENTITY_MOB = 49,

    ENTITY_CREEPER = 50,
    ENTITY_SKELETON = 51,
    ENTITY_SPIDER = 52,
    ENTITY_GIANTZOMBIE = 53,
    ENTITY_ZOMBIE = 54,
    ENTITY_SLIME = 55,
    ENTITY_GHAST = 56,
    ENTITY_PIGZOMBIE = 57,

    ENTITY_PIG = 90,
    ENTITY_SHEEP = 91,
    ENTITY_COW = 92,
    ENTITY_CHICKEN = 93,
    ENTITY_SQUID = 94,
    ENTITY_WOLF = 95,

    ENTITY_PLAYER = 200,
} entity_type;

extern entity_type nonliving_type_lookup[256];

typedef struct entity {
    int id;
    entity_type type;
    char *name; // for named entities (players)

    vec3_t position;
    vec3_t position_old;
    vec3_t velocity;
    float move_side, move_forward;
    float smooth_step_view_height_offset, eye_offset;
    vec3_t rotation;
    bbox_t bbox;
    bool onground;
    bool was_onground; // fix for unlocked physics fps
    bool collided_horizontally;
    bool collided_vertically;

    struct entity *vehicle;

    // per-type data
    union {
        struct {
            int id;
            int count;
            int metadata;
        } item;
        struct {
            enum { MINECART_EMPTY, MINECART_CHEST, MINECART_FURNACE } type;
        } minecart;
        struct {
            enum {
                PROJECTILE_SNOWBALL,
                PROJECTILE_EGG,
                PROJECTILE_FIREBALL
            } type;
            struct entity *owner;
        } projectile;
        struct {
            struct entity *owner;
        } arrow;
        struct {
            block_id id;
        } falling_block;
        struct {
            enum painting_type {
                PAINTING_KEBAB,
                PAINTING_AZETC,
                PAINTING_ALBAN,
                PAINTING_AZTEC2,
                PAINTING_BOMB,
                PAINTING_PLANT,
                PAINTING_WASTELAND,
                PAINTING_POOL,
                PAINTING_COURBET,
                PAINTING_SEA,
                PAINTING_SUNSET,
                PAINTING_CREEBET,
                PAINTING_WANDERER,
                PAINTING_GRAHAM,
                PAINTING_MATCH,
                PAINTING_BUST,
                PAINTING_STAGE,
                PAINTING_VOID,
                PAINTING_SKULL_AND_ROSES,
                PAINTING_FIGHTERS,
                PAINTING_POINTER,
                PAINTING_PIG_SCENE,
                PAINTING_BURNING_SKULL,
                PAINTING_SKELETON,
                PAINTING_DONKEY_KONG,
                PAINTING_TYPE_COUNT
            } type;
        } painting;
    };
} entity;

extern struct hashmap *world_entity_map;

void entity_set_position(entity *ent, vec3_t pos);
void entity_update(entity *ent, float dt);
void entity_handle_status_update(entity *ent, byte status);

bool entity_in_water(entity *ent);
bool entity_in_lava(entity *ent);

errcode entity_renderer_init(void);
void entity_renderer_shutdown(void);
void entity_renderer_render(void);

#endif
