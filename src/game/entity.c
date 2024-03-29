#include "entity.h"
#include "common.h"
#include "client/client.h"

void entity_handle_status_update(entity *ent, byte status)
{
    if(ent->type >= ENTITY_LIVING) {
        switch(status) {
            case 2: {
                // got hit, play sfx, make red etc.
            } break;
            case 3: {
                // dead
            } break;
        }
    }

    if(ent->type == ENTITY_WOLF) {
        switch(status) {
            case 6:
            case 7: {
                // show taming particles
                // status 6 is smoke particle
                // status 7 is heart particle
            } break;
            case 8: {
                // start shake animation?
            } break;
        }
    }
}

void entity_set_position(entity *ent, vec3_t pos)
{
    // size for a player tho
    float w = 0.6f / 2.0f;
    float h = 1.8f;

    ent->position_old = pos; // yes
    ent->position = pos;
    ent->bbox = (bbox_t) {
        .mins = vec3(pos.x - w, pos.y - ent->eye_offset + ent->height_offset, pos.z - w),
        .maxs = vec3(pos.x + w, pos.y - ent->eye_offset + ent->height_offset + h, pos.z + w)
    };
}

bool entity_in_water(entity *ent)
{
    block_data block = world_get_blockf(ent->position.x, ent->bbox.mins.y+0.4f, ent->position.z);
    return block.id == BLOCK_WATER_MOVING || block.id == BLOCK_WATER_STILL;
}

bool entity_in_lava(entity *ent)
{
    block_data block = world_get_blockf(ent->position.x, ent->bbox.mins.y+0.4f, ent->position.z);
    return block.id == BLOCK_LAVA_MOVING || block.id == BLOCK_LAVA_STILL;
}

bool entity_on_ladder(entity *ent)
{
    block_data block = world_get_blockf(ent->position.x, ent->bbox.mins.y, ent->position.z);
    return block.id == BLOCK_LADDER;
}
