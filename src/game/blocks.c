#include "common.h"
#include "block.h"
#include "client/cvar.h"
#include "world.h"

ubyte block_get_texture_index(block_id id, block_face face, ubyte metadata, int x, int y, int z)
{
    block_properties props = block_get_properties(id);
    int off = 0;

    switch(id) {
        /* non-directional blocks */
    case BLOCK_SAPLING:
        return props.texture_indices[face] + 16 * (metadata + 2);
    case BLOCK_WOOD_LOG:
        if(IS_SIDE_FACE(face))
            return props.texture_indices[face] + (metadata == 0 ? 0 : (16 * 6 + (metadata - 1)));
        break;
    case BLOCK_LEAVES:
        return props.texture_indices[face] +
               (!r_fancyleaves.integer) + // +1 is for opaque leaves
               (metadata == 1 ? (16 * 5) : 0); // oak/birch leaves use the same texture - move only if spruce
    case BLOCK_TALLGRASS:
        if(metadata == 0)
            off = 16;
        else if(metadata == 2)
            off = 16 + 1;
        return props.texture_indices[face] + off;
    case BLOCK_CLOTH:
        if(metadata == 0)
            return props.texture_indices[face];
        metadata = ~(metadata & 15);
        return (7 * 16 + 1) + ((metadata & 8) >> 3) + 16 * (metadata & 7); // :P
    case BLOCK_SLAB_SINGLE:
    case BLOCK_SLAB_DOUBLE:
        if(metadata == 0) {
            // smooth stone
            if(IS_TOPBOT_FACE(face))
                return 6;
            return 5;
        } else if(metadata == 1) {
            // sandstone
            switch(face) {
            case BLOCK_FACE_Y_POS:
                return 13 * 16;
            case BLOCK_FACE_Y_NEG:
                return 15 * 16;
            default:
                return 14 * 16;
            }
        } else if(metadata == 2) {
            // planks
            return 4;
        } else if(metadata == 3) {
            // cobblestone
            return 16;
        }
        break;
    case BLOCK_WORKBENCH:
        if(IS_SIDE_FACE(face) && (face == BLOCK_FACE_Z_NEG || face == BLOCK_FACE_X_NEG))
            return props.texture_indices[face] + 1;
        break;
    case BLOCK_CROP_WHEAT:
        return props.texture_indices[face] + metadata;
    case BLOCK_FARMLAND:
        if(face == BLOCK_FACE_Y_POS)
            return props.texture_indices[BLOCK_FACE_Y_POS] - (int) (metadata != 0);
        break;
        /* directional blocks */
    case BLOCK_BED: // todo
        break;
    case BLOCK_PISTON_BASE: // todo am lazy
    case BLOCK_PISTON_BASE_STICKY:
        break;
    case BLOCK_CHEST: // todo this is stupid :/
        break;
    case BLOCK_DISPENSER:
    case BLOCK_FURNACE_ACTIVE:
    case BLOCK_FURNACE_IDLE:
        off = (id == BLOCK_DISPENSER ? 1 : (id == BLOCK_FURNACE_IDLE ? -1 : 16));
        if(IS_SIDE_FACE(face) && metadata == face)
            return props.texture_indices[face] + off;
        break;
    case BLOCK_PUMPKIN_LANTERN:
        off = 1;
        /* fall through */
    case BLOCK_PUMPKIN:
        if(IS_SIDE_FACE(face)) {
            // bruh..
            const block_face d[4] = {BLOCK_FACE_Z_POS, BLOCK_FACE_X_NEG, BLOCK_FACE_Z_NEG, BLOCK_FACE_X_POS};
            if(d[metadata] == face) {
                return props.texture_indices[face] + 1 + off;
            }
        }
        break;
    default:
        break;
    }

    return props.texture_indices[face];
}

bbox_t block_get_bbox(block_data block, int x, int y, int z, bool selectmode)
{
    const bbox_t no_bbox = {vec3_1(-1), vec3_1(-1)};

    const float _1 = 1.0f / 16.0f;
    const float _2 = 2.0f / 16.0f;
    const float _3 = 3.0f / 16.0f;
    const float _4 = 4.0f / 16.0f;
    const float _5 = 5.0f / 16.0f;
    const float _6 = 6.0f / 16.0f;
    const float _7 = 7.0f / 16.0f;
    const float _8 = 8.0f / 16.0f;
    const float _9 = 9.0f / 16.0f;
    const float _10 = 10.0f / 16.0f;
    const float _11 = 11.0f / 16.0f;
    const float _12 = 12.0f / 16.0f;
    const float _13 = 13.0f / 16.0f;
    const float _14 = 14.0f / 16.0f;
    const float _15 = 15.0f / 16.0f;
    const float _16 = 16.0f / 16.0f;

    switch(block.id) {
    case BLOCK_AIR:
    case BLOCK_WATER_MOVING:
    case BLOCK_WATER_STILL:
    case BLOCK_LAVA_MOVING:
    case BLOCK_LAVA_STILL:
    case BLOCK_FIRE:
        return no_bbox;

    case BLOCK_FLOWER_DANDELION:
    case BLOCK_FLOWER_ROSE:
    case BLOCK_MUSHROOM_BROWN:
    case BLOCK_MUSHROOM_RED:
        if(!selectmode) {
            return no_bbox;
        } else {
            return bbox_offset((bbox_t) {vec3(0.3f, 0.0f, 0.3f), vec3(0.7f, 0.6f, 0.7f)}, vec3(x, y, z));
        }
    case BLOCK_SUGAR_CANE:
        if(!selectmode) {
            return no_bbox;
        } else {
            return bbox_offset((bbox_t) {vec3(_2, 0, _2), vec3(_14, 1, _14)}, vec3(x, y, z));
        }
    case BLOCK_TALLGRASS:
    case BLOCK_DEADBUSH:
    case BLOCK_SAPLING:
        if(!selectmode) {
            return no_bbox;
        } else {
            return bbox_offset((bbox_t) {vec3(0.1f, 0.0f, 0.1f), vec3(0.9f, 0.8f, 0.9f)}, vec3(x, y, z));
        }
    case BLOCK_CROP_WHEAT:
        if(!selectmode) {
            return no_bbox;
        } else {
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, _4, 1)}, vec3(x, y, z));
        }
    case BLOCK_REDSTONE_DUST:
        if(!selectmode) {
            return no_bbox;
        } else {
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, _1, 1)}, vec3(x, y, z));
        }
    case BLOCK_RAIL_POWERED:
    case BLOCK_RAIL_DETECTOR:
    case BLOCK_RAIL:
        if(!selectmode) {
            return no_bbox;
        } else {
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, _2, 1)}, vec3(x, y, z));
        }
    case BLOCK_COBWEB:
        if(!selectmode) {
            return no_bbox;
        } else {
            break; /* full block hitbox */
        }
    case BLOCK_TORCH:
    case BLOCK_TORCH_REDSTONE_DISABLED:
    case BLOCK_TORCH_REDSTONE_ENABLED:
        if(!selectmode) {
            return no_bbox;
        } else {
            switch(block.metadata & 7) {
            case 1:
                return bbox_offset((bbox_t) {vec3(0.00f, 0.20f, 0.35f), vec3(0.30f, 0.80f, 0.65f)}, vec3(x, y, z));
            case 2:
                return bbox_offset((bbox_t) {vec3(0.70f, 0.20f, 0.35f), vec3(1.00f, 0.80f, 0.65f)}, vec3(x, y, z));
            case 3:
                return bbox_offset((bbox_t) {vec3(0.35f, 0.20f, 0.00f), vec3(0.65f, 0.80f, 0.30f)}, vec3(x, y, z));
            case 4:
                return bbox_offset((bbox_t) {vec3(0.35f, 0.20f, 0.70f), vec3(0.65f, 0.80f, 1.00f)}, vec3(x, y, z));
            default:
                return bbox_offset((bbox_t) {vec3(0.4f, 0.0f, 0.4f), vec3(0.6f, 0.6f, 0.6f)}, vec3(x, y, z));
            }
        }
    case BLOCK_SIGN_POST:
        if(!selectmode) {
            return no_bbox;
        } else {
            return bbox_offset((bbox_t) {vec3(_4, 0, _4), vec3(_12, 1, _12)}, vec3(x, y, z));
        }
    case BLOCK_SIGN_WALL:
        if(!selectmode) {
            return no_bbox;
        } else {
            float _9_32 = 9.0f / 32.0f;
            float _25_32 = 25.0f / 32.0f;
            float _4_32 = 4.0f / 32.0f;
            float _28_32 = 28.0f / 32.0f;
            switch(block.metadata) {
            case 2:
                return bbox_offset((bbox_t) {vec3(0, _9_32, _28_32), vec3(1, _25_32, 1)}, vec3(x, y, z));
            case 3:
                return bbox_offset((bbox_t) {vec3(0, _9_32, 0), vec3(1, _25_32, _4_32)}, vec3(x, y, z));
            case 4:
                return bbox_offset((bbox_t) {vec3(_28_32, _9_32, 0), vec3(1, _25_32, 1)}, vec3(x, y, z));
            case 5:
                return bbox_offset((bbox_t) {vec3(0, _9_32, 0), vec3(_4_32, _25_32, 1)}, vec3(x, y, z));
            default:
                break;
            }
            break; /* full block hitbox */
        }
    case BLOCK_PORTAL:
        if(!selectmode) {
            return no_bbox;
        } else {
            if(world_get_block(x - 1, y, z).id == block.id || world_get_block(x + 1, y, z).id == block.id) {
                return bbox_offset((bbox_t) {vec3(0, 0, _6), vec3(1, 1, _10)}, vec3(x, y, z));
            } else {
                return bbox_offset((bbox_t) {vec3(_6, 0, 0), vec3(_10, 1, 1)}, vec3(x, y, z));
            }
        }

    case BLOCK_PRESSURE_PLATE_WOOD:
    case BLOCK_PRESSURE_PLATE_STONE:
        if(!selectmode) {
            return no_bbox;
        } else {
            float h = _1;
            if(block.metadata != 0)
                h *= 0.5f;
            return bbox_offset((bbox_t) {vec3(_1, 0, _1), vec3(_15, h, _15)}, vec3(x, y, z));
        }

    case BLOCK_LEVER:
        if(!selectmode) {
            return no_bbox;
        } else {
            switch(block.metadata & 7) {
            case 1:
                return bbox_offset((bbox_t) {vec3(0.0f, 0.2f, _5), vec3(_6, 0.8f, _11)}, vec3(x, y, z));
            case 2:
                return bbox_offset((bbox_t) {vec3(_10, 0.2f, _5), vec3(1.0f, 0.8f, _11)}, vec3(x, y, z));
            case 3:
                return bbox_offset((bbox_t) {vec3(_5, 0.2f, 0.0f), vec3(_11, 0.8f, _6)}, vec3(x, y, z));
            case 4:
                return bbox_offset((bbox_t) {vec3(_5, 0.2f, _10), vec3(_11, 0.8f, 1.0f)}, vec3(x, y, z));
            default:
                return bbox_offset((bbox_t) {vec3(_4, 0.0f, _4), vec3(_12, 0.6f, _12)}, vec3(x, y, z));
            }
        }

    case BLOCK_BUTTON:
        if(!selectmode) {
            return no_bbox;
        } else {
            int side = block.metadata & 7;
            bool pressed = (block.metadata & 8) != 0;
            float d = pressed ? _1 : _2;
            switch(side) {
            case 1:
                return bbox_offset((bbox_t) {vec3(0, _6, _5), vec3(d, _10, _11)}, vec3(x, y, z));
            case 2:
                return bbox_offset((bbox_t) {vec3(1 - d, _6, _5), vec3(_16, _10, _11)}, vec3(x, y, z));
            case 3:
                return bbox_offset((bbox_t) {vec3(_5, _6, 0), vec3(_11, _10, d)}, vec3(x, y, z));
            case 4:
                return bbox_offset((bbox_t) {vec3(_5, _6, 1 - d), vec3(_11, _10, _16)}, vec3(x, y, z));
            default:
                break;
            }
            break;
        }
    case BLOCK_SLAB_SINGLE:
        return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, 0.5f, 1)}, vec3(x, y, z));

    case BLOCK_BED:
        return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, _9, 1)}, vec3(x, y, z));

    case BLOCK_SOUL_SAND:
        return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, _14, 1)}, vec3(x, y, z));

    case BLOCK_FARMLAND:
        return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, _15, 1)}, vec3(x, y, z));

    case BLOCK_REDSTONE_REPEATER_DISABLED:
    case BLOCK_REDSTONE_REPEATER_ENABLED:
        return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, _2, 1)}, vec3(x, y, z));

        /* todo later */
    case BLOCK_PISTON_BASE_STICKY:
        break;
    case BLOCK_PISTON_BASE:
        break;
    case BLOCK_PISTON_EXTENSION:
        break;
    case BLOCK_PISTON_MOVING:
        break;

    case BLOCK_SNOW_LAYER: {
        if((block.metadata & 7) < 3)
            return no_bbox;
        return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, 0.5f, 1)}, vec3(x, y, z));
    }
    case BLOCK_LADDER: {
        switch(block.metadata) {
        case BLOCK_FACE_Z_NEG:
            return bbox_offset((bbox_t) {vec3(0, 0, _14), vec3(1, 1, 1)}, vec3(x, y, z));
        case BLOCK_FACE_Z_POS:
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, 1, _2)}, vec3(x, y, z));
        case BLOCK_FACE_X_NEG:
            return bbox_offset((bbox_t) {vec3(_14, 0, 0), vec3(1, 1, 1)}, vec3(x, y, z));
        case BLOCK_FACE_X_POS:
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(_2, 1, 1)}, vec3(x, y, z));
        default:
            break; /* full block hitbox */
        }
        break;
    }
    case BLOCK_DOOR_WOOD:
    case BLOCK_DOOR_IRON: {
        bool open = (block.metadata & 4) != 0;
        int rotation = block.metadata & 3;
        int state = open ? rotation : rotation - 1;

        switch(state) {
        case 0:
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, 1, _3)}, vec3(x, y, z));
        case 1:
            return bbox_offset((bbox_t) {vec3(_13, 0, 0), vec3(1, 1, 1)}, vec3(x, y, z));
        case 2:
            return bbox_offset((bbox_t) {vec3(0, 0, _13), vec3(1, 1, 1)}, vec3(x, y, z));
        case 3:
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(_3, 1, 1)}, vec3(x, y, z));
        default:
            // yes, this is stupid
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, 2, 1)}, vec3(x, y, z));
        }
    }
    case BLOCK_TRAPDOOR: {
        bool open = (block.metadata & 4) != 0;
        int rotation = block.metadata & 3;

        if(!open)
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, _3, 1)}, vec3(x, y, z));

        switch(rotation) {
        case 0:
            return bbox_offset((bbox_t) {vec3(0, 0, _13), vec3(1, 1, 1)}, vec3(x, y, z));
        case 1:
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, 1, _3)}, vec3(x, y, z));
        case 2:
            return bbox_offset((bbox_t) {vec3(_13, 0, 0), vec3(1, 1, 1)}, vec3(x, y, z));
        case 3:
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(_3, 1, 1)}, vec3(x, y, z));
        default:
            break;
        }
        break;
    }
    case BLOCK_CACTUS:
        return bbox_offset((bbox_t) {vec3(_1, 0, _1), vec3(_15, (selectmode ? _16 : _15), _15)}, vec3(x, y, z));

    case BLOCK_FENCE:
        if(!selectmode) {
            return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, 1.5f, 1)}, vec3(x, y, z));
        } else {
            break; /* full block hitbox */
        }
    case BLOCK_CAKE: {
        float size = _1 + (float) (block.metadata) / 8.0f;
        return bbox_offset((bbox_t) {vec3(size, 0, _1), vec3(_15, _7, _15)}, vec3(x, y, z));
    }
    default:
        break;
    }

    /* full block hitbox */
    return bbox_offset((bbox_t) {vec3(0, 0, 0), vec3(1, 1, 1)}, vec3(x, y, z));
}
