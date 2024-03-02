#include "block.h"
#include "common.h"
#include "world.h"
#include "client/console.h"
#include "client/client.h"
#include "client/cvar.h"

enum { // this is an enum because of syntax highlighting
    TRANSPARENT = 0, OPAQUE = 1
};

#define TEXTURE_ALL(idx) {idx,idx,idx,idx,idx,idx}
#define TEXTURE_TOP_BOT_SIDE(top, bot, side) {bot, top, side, side, side, side}
#define TEXTURE_TOPBOT_SIDE(topbot, side) {topbot, topbot, side, side, side, side}

static block_properties blocks[256] = {
        [BLOCK_AIR] = {"air", TEXTURE_ALL(0), TRANSPARENT, RENDER_NONE},
        [BLOCK_STONE] = {"stone", TEXTURE_ALL(1), OPAQUE, RENDER_CUBE},
        [BLOCK_GRASS] = {"grass", TEXTURE_TOP_BOT_SIDE(0, 2, 3), OPAQUE, RENDER_CUBE},
        [BLOCK_DIRT] = {"dirt", TEXTURE_ALL(2), OPAQUE, RENDER_CUBE},
        [BLOCK_COBBLESTONE] = {"cobblestone", TEXTURE_ALL(16), OPAQUE, RENDER_CUBE},
        [BLOCK_WOOD_PLANKS] = {"planks", TEXTURE_ALL(4), OPAQUE, RENDER_CUBE},
        [BLOCK_SAPLING] = {"sapling", TEXTURE_ALL(15), TRANSPARENT, RENDER_CROSS},
        [BLOCK_BEDROCK] = {"bedrock", TEXTURE_ALL(17), OPAQUE, RENDER_CUBE},
        [BLOCK_WATER_MOVING] = {"water_flowing", TEXTURE_ALL(255 - 32), TRANSPARENT, RENDER_FLUID},
        [BLOCK_WATER_STILL] = {"water_still", TEXTURE_ALL(255 - 32), TRANSPARENT, RENDER_FLUID},
        [BLOCK_LAVA_MOVING] = {"lava_flowing", TEXTURE_ALL(255), TRANSPARENT, RENDER_FLUID},
        [BLOCK_LAVA_STILL] = {"lava_still", TEXTURE_ALL(255), TRANSPARENT, RENDER_FLUID},
        [BLOCK_SAND] = {"sand", TEXTURE_ALL(18), OPAQUE, RENDER_CUBE},
        [BLOCK_GRAVEL] = {"gravel", TEXTURE_ALL(19), OPAQUE, RENDER_CUBE},
        [BLOCK_ORE_GOLD] = {"gold_ore", TEXTURE_ALL(32), OPAQUE, RENDER_CUBE},
        [BLOCK_ORE_IRON] = {"iron_ore", TEXTURE_ALL(33), OPAQUE, RENDER_CUBE},
        [BLOCK_ORE_COAL] = {"coal_ore", TEXTURE_ALL(34), OPAQUE, RENDER_CUBE},
        [BLOCK_WOOD_LOG] = {"log", TEXTURE_TOPBOT_SIDE(21, 20), OPAQUE, RENDER_CUBE},
        [BLOCK_LEAVES] = {"leaves", TEXTURE_ALL(52), TRANSPARENT, RENDER_CUBE},
        [BLOCK_SPONGE] = {"sponge", TEXTURE_ALL(48), OPAQUE, RENDER_CUBE},
        [BLOCK_GLASS] = {"glass", TEXTURE_ALL(49), TRANSPARENT, RENDER_CUBE},
        [BLOCK_ORE_LAPIS] = {"lapis_ore", TEXTURE_ALL(160), OPAQUE, RENDER_CUBE},
        [BLOCK_BLOCK_LAPIS] = {"lapis_block", TEXTURE_ALL(144), OPAQUE, RENDER_CUBE},
        [BLOCK_DISPENSER] = {"dispenser", TEXTURE_TOPBOT_SIDE(45 + 17, 45), OPAQUE, RENDER_CUBE},
        [BLOCK_SANDSTONE] = {"sandstone", TEXTURE_TOP_BOT_SIDE(192 - 16, 192 + 16, 192), OPAQUE, RENDER_CUBE},
        [BLOCK_NOTEBLOCK] = {"noteblock", TEXTURE_ALL(74), OPAQUE, RENDER_CUBE},
        [BLOCK_BED] = {"bed", TEXTURE_ALL(134), TRANSPARENT, RENDER_BED},
        [BLOCK_RAIL_POWERED] = {"rail_powered", TEXTURE_ALL(179), TRANSPARENT, RENDER_RAIL},
        [BLOCK_RAIL_DETECTOR] = {"rail_detector", TEXTURE_ALL(195), TRANSPARENT, RENDER_RAIL},
        [BLOCK_PISTON_BASE_STICKY] = {"piston_sticky", TEXTURE_TOP_BOT_SIDE(106, 106 + 3, 106 + 2), TRANSPARENT, RENDER_PISTON_BASE},
        [BLOCK_COBWEB] = {"cobweb", TEXTURE_ALL(11), TRANSPARENT, RENDER_CROSS},
        [BLOCK_TALLGRASS] = {"tallgrass", TEXTURE_ALL(39), TRANSPARENT, RENDER_CROSS},
        [BLOCK_DEADBUSH] = {"deadbush", TEXTURE_ALL(55), TRANSPARENT, RENDER_CROSS},
        [BLOCK_PISTON_BASE] = {"piston", TEXTURE_TOP_BOT_SIDE(107, 107 + 2, 107 + 1), TRANSPARENT, RENDER_PISTON_BASE},
        [BLOCK_PISTON_EXTENSION] = {"piston_head", TEXTURE_ALL(250), TRANSPARENT, RENDER_PISTON_EXTENSION},
        [BLOCK_CLOTH] = {"wool", TEXTURE_ALL(64), OPAQUE, RENDER_CUBE},
        [BLOCK_PISTON_MOVING] = {"piston_moving (idk)", TEXTURE_ALL(240), TRANSPARENT, RENDER_NONE},
        [BLOCK_FLOWER_DANDELION] = {"flower_yellow", TEXTURE_ALL(13), TRANSPARENT, RENDER_CROSS},
        [BLOCK_FLOWER_ROSE] = {"flower_red", TEXTURE_ALL(12), TRANSPARENT, RENDER_CROSS},
        [BLOCK_MUSHROOM_BROWN] = {"mushroom_brown", TEXTURE_ALL(29), TRANSPARENT, RENDER_CROSS},
        [BLOCK_MUSHROOM_RED] = {"mushroom_red", TEXTURE_ALL(28), TRANSPARENT, RENDER_CROSS},
        [BLOCK_BLOCK_GOLD] = {"gold_block", TEXTURE_ALL(23), OPAQUE, RENDER_CUBE},
        [BLOCK_BLOCK_IRON] = {"iron_block", TEXTURE_ALL(22), OPAQUE, RENDER_CUBE},
        [BLOCK_SLAB_DOUBLE] = {"slab_double", TEXTURE_ALL(6), OPAQUE, RENDER_CUBE},
        [BLOCK_SLAB_SINGLE] = {"slab_single", TEXTURE_ALL(6), TRANSPARENT, RENDER_CUBE_SPECIAL},
        [BLOCK_BRICK] = {"bricks", TEXTURE_ALL(7), OPAQUE, RENDER_CUBE},
        [BLOCK_TNT] = {"tnt", TEXTURE_TOP_BOT_SIDE(8 + 1, 8 + 2, 8), OPAQUE, RENDER_CUBE},
        [BLOCK_BOOKSHELF] = {"bookshelf", TEXTURE_TOPBOT_SIDE(4, 35), OPAQUE, RENDER_CUBE},
        [BLOCK_COBBLESTONE_MOSSY] = {"mossy_cobblestone", TEXTURE_ALL(36), OPAQUE, RENDER_CUBE},
        [BLOCK_OBSIDIAN] = {"obsidian", TEXTURE_ALL(37), OPAQUE, RENDER_CUBE},
        [BLOCK_TORCH] = {"torch", TEXTURE_ALL(80), TRANSPARENT, RENDER_TORCH},
        [BLOCK_FIRE] = {"fire", TEXTURE_ALL(31), TRANSPARENT, RENDER_FIRE},
        [BLOCK_MOB_SPAWNER] = {"spawner", TEXTURE_ALL(65), TRANSPARENT, RENDER_CUBE},
        [BLOCK_STAIRS_WOOD] = {"stair_wood", TEXTURE_ALL(4), TRANSPARENT, RENDER_STAIRS},
        [BLOCK_CHEST] = {"chest", TEXTURE_TOPBOT_SIDE(26 - 1, 26), OPAQUE, RENDER_CUBE},
        [BLOCK_REDSTONE_DUST] = {"redstone_wire", TEXTURE_ALL(164), TRANSPARENT, RENDER_WIRE},
        [BLOCK_ORE_DIAMOND] = {"diamond_ore", TEXTURE_ALL(50), OPAQUE, RENDER_CUBE},
        [BLOCK_BLOCK_DIAMOND] = {"diamond_block", TEXTURE_ALL(24), OPAQUE, RENDER_CUBE},
        [BLOCK_WORKBENCH] = {"workbench", TEXTURE_TOP_BOT_SIDE(59 - 16, 4, 59), OPAQUE, RENDER_CUBE},
        [BLOCK_CROP_WHEAT] = {"crops", TEXTURE_ALL(88), TRANSPARENT, RENDER_CROPS},
        [BLOCK_FARMLAND] = {"farmland", TEXTURE_TOP_BOT_SIDE(87, 2, 2), TRANSPARENT, RENDER_CUBE_SPECIAL},
        [BLOCK_FURNACE_IDLE] = {"furnace", TEXTURE_TOPBOT_SIDE(45 + 17, 45), OPAQUE, RENDER_CUBE},
        [BLOCK_FURNACE_ACTIVE] = {"furnace_lit", TEXTURE_TOPBOT_SIDE(45 + 17, 45), OPAQUE, RENDER_CUBE},
        [BLOCK_SIGN_POST] = {"sign_post", TEXTURE_ALL(4), TRANSPARENT, RENDER_NONE}, // todo
        [BLOCK_DOOR_WOOD] = {"door_wood", TEXTURE_ALL(97), TRANSPARENT, RENDER_DOOR},
        [BLOCK_LADDER] = {"ladder", TEXTURE_ALL(83), TRANSPARENT, RENDER_LADDER},
        [BLOCK_RAIL] = {"rail", TEXTURE_ALL(128), TRANSPARENT, RENDER_RAIL},
        [BLOCK_STAIRS_STONE] = {"stair_stone", TEXTURE_ALL(16), TRANSPARENT, RENDER_STAIRS},
        [BLOCK_SIGN_WALL] = {"sign_wall", TEXTURE_ALL(4), TRANSPARENT, RENDER_NONE}, // todo
        [BLOCK_LEVER] = {"lever", TEXTURE_ALL(96), TRANSPARENT, RENDER_LEVER},
        [BLOCK_PRESSURE_PLATE_STONE] = {"pressureplate_stone", TEXTURE_ALL(1), TRANSPARENT, RENDER_CUBE_SPECIAL},
        [BLOCK_DOOR_IRON] = {"door_iron", TEXTURE_ALL(98), TRANSPARENT, RENDER_DOOR},
        [BLOCK_PRESSURE_PLATE_WOOD] = {"pressureplate_wood", TEXTURE_ALL(4), TRANSPARENT, RENDER_CUBE_SPECIAL},
        [BLOCK_ORE_REDSTONE] = {"redstone_ore", TEXTURE_ALL(51), OPAQUE, RENDER_CUBE},
        [BLOCK_ORE_REDSTONE_GLOWING] = {"redstone_ore_lit", TEXTURE_ALL(51), OPAQUE, RENDER_CUBE},
        [BLOCK_TORCH_REDSTONE_DISABLED] = {"redstone_torch", TEXTURE_ALL(115), TRANSPARENT, RENDER_TORCH},
        [BLOCK_TORCH_REDSTONE_ENABLED] = {"redstone_torch_lit", TEXTURE_ALL(99), TRANSPARENT, RENDER_TORCH},
        [BLOCK_BUTTON] = {"button", TEXTURE_ALL(1), TRANSPARENT, RENDER_CUBE_SPECIAL},
        [BLOCK_SNOW_LAYER] = {"snow_layer", TEXTURE_ALL(66), TRANSPARENT, RENDER_CUBE_SPECIAL},
        [BLOCK_ICE] = {"ice", TEXTURE_ALL(67), TRANSPARENT, RENDER_CUBE},
        [BLOCK_SNOW] = {"snow", TEXTURE_ALL(66), OPAQUE, RENDER_CUBE},
        [BLOCK_CACTUS] = {"cactus", TEXTURE_TOP_BOT_SIDE(70 - 1, 70 + 1, 70), TRANSPARENT, RENDER_CACTUS},
        [BLOCK_CLAY] = {"clay", TEXTURE_ALL(72), OPAQUE, RENDER_CUBE},
        [BLOCK_SUGAR_CANE] = {"reeds", TEXTURE_ALL(73), TRANSPARENT, RENDER_CROSS},
        [BLOCK_JUKEBOX] = {"jukebox", TEXTURE_TOP_BOT_SIDE(75, 74, 74), OPAQUE, RENDER_CUBE},
        [BLOCK_FENCE] = {"fence", TEXTURE_ALL(4), TRANSPARENT, RENDER_FENCE},
        [BLOCK_PUMPKIN] = {"pumpkin", TEXTURE_TOPBOT_SIDE(102, 102 + 16), OPAQUE, RENDER_CUBE},
        [BLOCK_NETTHERRACK] = {"netherrack", TEXTURE_ALL(103), OPAQUE, RENDER_CUBE},
        [BLOCK_SOUL_SAND] = {"soulsand", TEXTURE_ALL(104), TRANSPARENT, RENDER_CUBE},
        [BLOCK_GLOWSTONE] = {"glowstone", TEXTURE_ALL(105), OPAQUE, RENDER_CUBE},
        [BLOCK_PORTAL] = {"portal", TEXTURE_ALL(240), TRANSPARENT, RENDER_CUBE_SPECIAL},
        [BLOCK_PUMPKIN_LANTERN] = {"pumpkin_lit", TEXTURE_TOPBOT_SIDE(102, 102 + 16), OPAQUE, RENDER_CUBE},
        [BLOCK_CAKE] = {"cake", TEXTURE_TOP_BOT_SIDE(121, 121 + 3, 121 + 1), TRANSPARENT, RENDER_CUBE_SPECIAL},
        [BLOCK_REDSTONE_REPEATER_DISABLED] = {"redstone_repeater", TEXTURE_ALL(131), TRANSPARENT, RENDER_REPEATER},
        [BLOCK_REDSTONE_REPEATER_ENABLED] = {"redstone_repeater_lit", TEXTURE_ALL(147), TRANSPARENT, RENDER_REPEATER},
        [BLOCK_TRAPDOOR] = {"trapdoor", TEXTURE_ALL(84), TRANSPARENT, RENDER_CUBE_SPECIAL},
};

block_properties block_get_properties(block_id id) {
    return blocks[id & 255];
}

bool block_should_face_be_rendered(int x, int y, int z, block_data self, block_face face) {
    vec3_t face_offsets[6] = {
            [BLOCK_FACE_Y_NEG] = vec3(0, -1, 0),
            [BLOCK_FACE_Y_POS] = vec3(0, 1, 0),
            [BLOCK_FACE_Z_NEG] = vec3(0, 0, -1),
            [BLOCK_FACE_Z_POS] = vec3(0, 0, 1),
            [BLOCK_FACE_X_NEG] = vec3(-1, 0, 0),
            [BLOCK_FACE_X_POS] = vec3(1, 0, 0),
    };

    block_data other;
    vec3_t pos2 = vec3(x, y, z);
    pos2 = vec3_add(pos2, face_offsets[face]);

	other = world_get_block(pos2.x, pos2.y, pos2.z);

    if(block_is_semi_transparent(self) && block_is_semi_transparent(other))
        return false;

    if(self.id != BLOCK_LEAVES || r_smartleaves.integer)
        if(other.id == self.id)
            return false;

    if(block_get_properties(self.id).render_type == RENDER_FLUID) {
        if(face == BLOCK_FACE_Y_POS) {
            return true;
        }
    }
    return block_is_transparent(other);
}

static float fluid_get_percent_air(ubyte metadata) {
    if(metadata >= 8)
        metadata = 0;
    return (float) (metadata + 1) / 9.0f;
}

static bool is_same_fluid_material(block_id self, block_id other) {
    if(other == self)
        return true;
    if(self == BLOCK_WATER_MOVING)
        return other == BLOCK_WATER_STILL;
    if(self == BLOCK_WATER_STILL)
        return other == BLOCK_WATER_MOVING;
    if(self == BLOCK_LAVA_MOVING)
        return other == BLOCK_LAVA_STILL;
    if(self == BLOCK_LAVA_STILL)
        return other == BLOCK_LAVA_MOVING;
    return false;
}

float block_fluid_get_height(int x, int y, int z, block_id self_id) {
    int total = 0;
    float total_air = 0.0f;

    for(int side = 0; side < 4; side++) {
        int xoff = x - (side & 1);
        int zoff = z - (side >> 1 & 1);
        block_data other;

        if(world_get_block(xoff, y + 1, zoff).id == self_id) {
            return 1.0f;
        }

        other = world_get_block(xoff, y, zoff);
        if(other.id != self_id && !is_same_fluid_material(self_id, other.id)) {
            if(!block_is_solid(other)) {
                total_air++;
                total++;
            }
        } else {
            if(other.metadata >= 8 || other.metadata == 0) {
                total_air += fluid_get_percent_air(other.metadata) * 10.0f;
                total += 10;
            }
            total_air += fluid_get_percent_air(other.metadata);
            total++;
        }
    }
    return 1.0f - total_air / (float) total;
}

void onchange_block_render_modes(void) {
    blocks[BLOCK_LEAVES].opaque = !r_fancyleaves.integer;
    if(world_is_init()) {
        world_mark_all_for_remesh();
    }
}

bool block_is_collidable(block_data block)
{
    return !bbox_null(block_get_bbox(block, 0, 0, 0, false));
}

bool block_is_selectable(block_data block)
{
    return !bbox_null(block_get_bbox(block, 0, 0, 0, true));
}
