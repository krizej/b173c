/* useful links:
 * https://www.mcpk.wiki/wiki/45_Strafe#Explanation
 * https://www.mcpk.wiki/wiki/Collisions
 * https://www.mcpk.wiki/wiki/Player_Movement
 * https://www.mcpk.wiki/wiki/Movement_Formulas
 */
#include "entity.h"
#include "world.h"
#include "client/client.h"
#include "mathlib.h"
#include "client/cvar.h"
#include <math.h>

#define TO_20_TPS(dt) ((dt) * 20.0f)

static float calc_offset(bbox_t self, bbox_t other, float dist, int axis)
{
    int axis2 = (axis + 1) % 3;
    int axis3 = (axis + 2) % 3;

    if(other.maxs.array[axis2] > self.mins.array[axis2] && other.mins.array[axis2] < self.maxs.array[axis2]) {
        if(other.maxs.array[axis3] > self.mins.array[axis3] && other.mins.array[axis3] < self.maxs.array[axis3]) {
            // if we are overlapping on the other axes:

            float off;
            if(dist > 0.0f && other.maxs.array[axis] <= self.mins.array[axis]) {
                off = self.mins.array[axis] - other.maxs.array[axis];
                if(off < dist) {
                    dist = off;
                }
            }

            if(dist < 0.0f && other.mins.array[axis] >= self.maxs.array[axis]) {
                off = self.maxs.array[axis] - other.mins.array[axis];
                if(off > dist) {
                    dist = off;
                }
            }
        }
    }
    return dist;
}

static float testmove(bbox_t *colliders, entity *ent, float vel, int axis)
{
    for(bbox_t *collider = colliders; !bbox_null(*collider); collider++)
        vel = calc_offset(*collider, ent->bbox, vel, axis);
    return vel;
}

static void move_entity(entity *ent, vec3_t vel)
{
    const float step_height = 0.5f;
    bool onground2; // used for stepping code
    vec3_t oldvel = vel;
    bbox_t *colliders;
    bbox_t bbox_pre_move = ent->bbox;

    colliders = world_get_colliding_blocks(bbox_offset(ent->bbox, vel));

    // collision order: Y-X-Z
    vel.y = testmove(colliders, ent, vel.y, Y);
    ent->bbox = bbox_offset(ent->bbox, vec3(0.0f, vel.y, 0.0f));
    onground2 = ent->onground || (oldvel.y != vel.y && oldvel.y < 0.0f);

    vel.x = testmove(colliders, ent, vel.x, X);
    ent->bbox = bbox_offset(ent->bbox, vec3(vel.x, 0.0f, 0.0f));

    vel.z = testmove(colliders, ent, vel.z, Z);
    ent->bbox = bbox_offset(ent->bbox, vec3(0.0f, 0.0f, vel.z));

    // stepping code
    if(onground2 && (ent->smooth_step_view_height_offset < 0.05f) && (oldvel.x != vel.x || oldvel.z != vel.z)) {
        vec3_t vel_pre_step = vel;
        bbox_t bbox_after_move = ent->bbox;
        vel.x = oldvel.x;
        vel.y = step_height;
        vel.z = oldvel.z;

        ent->bbox = bbox_pre_move;

        colliders = world_get_colliding_blocks(bbox_offset(ent->bbox, vel));

        vel.y = testmove(colliders, ent, vel.y, Y);
        ent->bbox = bbox_offset(ent->bbox, vec3(0.0f, vel.y, 0.0f));

        vel.x = testmove(colliders, ent, vel.x, X);
        ent->bbox = bbox_offset(ent->bbox, vec3(vel.x, 0.0f, 0.0f));

        vel.z = testmove(colliders, ent, vel.z, Z);
        ent->bbox = bbox_offset(ent->bbox, vec3(0.0f, 0.0f, vel.z));

        vel.y = -step_height;
        vel.y = testmove(colliders, ent, vel.y, Y);
        ent->bbox = bbox_offset(ent->bbox, vec3(0.0f, vel.y, 0.0f));

        if(vec3_len(vel_pre_step) >= vec3_len(vel)) {
            // too far
            vel = vel_pre_step;
            ent->bbox = bbox_after_move;
        } else {
            // step up
            float dy = ent->bbox.mins.y - (float)((int)ent->bbox.mins.y);
            if(cl_smoothstep.integer == 2) {
                // apply fix
                dy = ent->bbox.mins.y - bbox_pre_move.mins.y;
            }
            if(dy > 0.0f) {
                ent->smooth_step_view_height_offset += dy + 0.01f;
            }
        }
    }

    ent->position.x = (ent->bbox.mins.x + ent->bbox.maxs.x) / 2.0f;
    ent->position.y = ent->bbox.mins.y + ent->eye_offset;
    ent->position.z = (ent->bbox.mins.z + ent->bbox.maxs.z) / 2.0f;

    if(cl_smoothstep.integer) {
        ent->position.y -= ent->smooth_step_view_height_offset;
    }

    ent->collided_horizontally = oldvel.x != vel.x || oldvel.z != vel.z;
    ent->collided_vertically = oldvel.y != vel.y;

    ent->onground = oldvel.y != vel.y && oldvel.y < 0.0f;
    if(ent->onground) {
        ent->was_onground = true;
    }

    if(oldvel.x != vel.x)
        ent->velocity.x = 0.0f;
    if(oldvel.y != vel.y)
        ent->velocity.y = 0.0f;
    if(oldvel.z != vel.z)
        ent->velocity.z = 0.0f;

    // TODO: step sound code

    // TODO: step on block code (e.g. soul sand -> reduce speed)
}

static void update_velocity(entity *ent, float accel)
{
    // see https://www.mcpk.wiki/wiki/45_Strafe
    float move_side = ent->move_side;
    float move_fwd = ent->move_forward;
    float length = sqrtf(move_side * move_side + move_fwd * move_fwd);
    vec3_t side, fwd;

    cam_angles(&fwd, &side, NULL, ent->rotation.yaw, 0);

    if(length >= 0.01f) {
        vec3_t add_fwd, add_side;

        if(length < 1.0f)
            length = 1.0f;

        length = accel / length;
        move_side *= length;
        move_fwd *= length;

        add_fwd = vec3_mul(fwd, move_fwd);
        add_side = vec3_mul(side, move_side);

        ent->velocity = vec3_add(ent->velocity, add_fwd);
        ent->velocity = vec3_add(ent->velocity, add_side);
    }
}

static bool any_blocks_in_bbox_are_liquid(bbox_t box)
{
    int x0 = (int)floorf(box.mins.x);
    int y0 = (int)floorf(box.mins.y);
    int z0 = (int)floorf(box.mins.z);
    int x1 = (int)floorf(box.maxs.x + 1.0f);
    int y1 = (int)floorf(box.maxs.y + 1.0f);
    int z1 = (int)floorf(box.maxs.z + 1.0f);

    if(box.mins.x < 0.0f)
        x0--;
    if(box.mins.y < 0.0f)
        y0--;
    if(box.mins.z < 0.0f)
        z0--;

    for(int x = x0; x < x1; x++) {
        for(int z = z0; z < z1; z++) {
            for(int y = y0; y < y1; y++) {
                block_data block = world_get_block(x, y, z);
                if(block_is_liquid(block)) {
                    return true;
                }
            }
        }
    }

    return false;
}

static bool is_in_position_to_jump_out_of_liquid(entity *ent, float dt)
{
    vec3_t vel = vec3_mul(ent->velocity, dt);
    vec3_t offset = vec3(vel.x, vel.y + 0.6f - ent->position.y + ent->position_old.y, vel.z);
    bbox_t bbox = bbox_offset(ent->bbox, offset);
    bbox_t *colliders = world_get_colliding_blocks(bbox);
    if(!bbox_null(*colliders))
        return false;
    return !any_blocks_in_bbox_are_liquid(bbox);
}

void entity_update(entity *ent, float dt)
{
    const float accel_in_air = 0.02f;
    const float drag = 0.02f;
    const float gravity = 0.08f;
    const float gravity_in_liquid = 0.02f;
    const float base_friction_ground = 0.91f;
    const float base_friction_water = 0.8f;
    const float base_friction_lava = 0.5f;

    float friction;

    dt = TO_20_TPS(dt);

    if(!world_is_init())
        return;

    ent->move_side *= 0.98f;
    ent->move_forward *= 0.98f;

    ent->position_old = ent->position;

    if(entity_in_water(ent) || entity_in_lava(ent)) {
        // TODO: being pushed by flowing liquid

        friction = entity_in_lava(ent) ? base_friction_lava : base_friction_water;

        update_velocity(ent, accel_in_air * dt);
        move_entity(ent, vec3_mul(ent->velocity, dt));

        ent->velocity.x -= (ent->velocity.x - ent->velocity.x * friction) * dt;
        ent->velocity.y -= (ent->velocity.y - ent->velocity.y * friction) * dt;
        ent->velocity.z -= (ent->velocity.z - ent->velocity.z * friction) * dt;
        ent->velocity.y -= gravity_in_liquid * dt;

        if(ent->collided_horizontally && is_in_position_to_jump_out_of_liquid(ent, dt)) {
            ent->velocity.y = 0.3f;
        }
    } else {
        // on ground
        float accel;

        friction = base_friction_ground;

        if (ent->onground) {
            block_data block = world_get_blockf(ent->position.x, ent->bbox.mins.y - 1, ent->position.z);
            friction = block.id != BLOCK_ICE ? 0.546f : 0.8918f; // fixme: hardcoding :/
        }

        if (ent->onground)
            /* (0.6 * 0.91)^3  --  0.6 = base slipperiness, 0.91 = base friction */
            /* means accel is lower on slippery surfaces (ice) */
            accel = 0.016277136f / (friction * friction * friction);
        else
            accel = accel_in_air;

        // fixme: this feels different
        ent->smooth_step_view_height_offset -= ent->smooth_step_view_height_offset * 0.6f * dt;

        update_velocity(ent, accel * dt);
        move_entity(ent, vec3_mul(ent->velocity, dt));

        // gravity
        ent->velocity.y -= gravity * dt;

        // drag
        ent->velocity.y -= ent->velocity.y * drag * dt;

        // friction
        ent->velocity.x -= (ent->velocity.x - ent->velocity.x * friction) * dt;
        ent->velocity.z -= (ent->velocity.z - ent->velocity.z * friction) * dt;
    }

    if(!vec3_equ(ent->position, ent->position_old)) {
        cl.game.moved = true;
    }
}




