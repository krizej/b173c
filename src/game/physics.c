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

void entity_update(entity *ent, float dt)
{
    const float drag = 0.02f;
    const float gravity = 0.08f;
    const float base_friction_ground = 0.91f;
    float friction;

    dt = TO_20_TPS(dt);

    if(!world_is_init())
        return;

    ent->move_side *= 0.98f;
    ent->move_forward *= 0.98f;

    if(1) { // !block_is_liquid(world_get_blockf(vec3_unpack(ent->position)))) {
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
            accel = 0.02f;

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




