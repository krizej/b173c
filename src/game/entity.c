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
    float h = 1.8f / 2.0f;

    ent->position = pos;
    ent->bbox = (bbox_t) {
        .mins = vec3_from(pos.x - w, pos.y - ent->y_offset + ent->y_size, pos.z - w),
        .maxs = vec3_from(pos.x + w, pos.y - ent->y_offset + ent->y_size + h, pos.z + w)
    };
}
