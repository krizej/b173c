#include "net_internal.h"
#include "game/world.h"
#include "client/console.h"
#include "client/client.h"
#include "client/cvar.h"
#include "vid/vid.h"
#include "net/packets.h"
#include "../client/client.h"

#define UNPACK_ANGLE(angle) ((float)(angle * 360) / 256.0f)
#define EMPTY_HANDLER(pkt) void net_handle_ ## pkt (pkt p attr(unused)) {}

entity_type nonliving_type_lookup[256] = {
    [1] = ENTITY_BOAT,
    [10] = ENTITY_MINECART, // empty
    [11] = ENTITY_MINECART, // chest
    [12] = ENTITY_MINECART, // furnace
    [50] = ENTITY_TNTPRIMED,
    [60] = ENTITY_ARROW,
    [61] = ENTITY_ITEM_PROJECTILE, // snowball
    [62] = ENTITY_ITEM_PROJECTILE, // egg
    [63] = ENTITY_ITEM_PROJECTILE, // fireball
    [70] = ENTITY_FALLINGBLOCK, // sand
    [71] = ENTITY_FALLINGBLOCK, // gravel
    [90] = ENTITY_FISHING_PARTICLE,
};

const char *painting_names[PAINTING_TYPE_COUNT] = {
    [PAINTING_KEBAB] = "Kebab",
    [PAINTING_AZETC] = "Aztec",
    [PAINTING_ALBAN] = "Alban",
    [PAINTING_AZTEC2] = "Aztec2",
    [PAINTING_BOMB] = "Bomb",
    [PAINTING_PLANT] = "Plant",
    [PAINTING_WASTELAND] = "Wasteland",
    [PAINTING_POOL] = "Pool",
    [PAINTING_COURBET] = "Courbet",
    [PAINTING_SEA] = "Sea",
    [PAINTING_SUNSET] = "Sunset",
    [PAINTING_CREEBET] = "Creebet",
    [PAINTING_WANDERER] = "Wanderer",
    [PAINTING_GRAHAM] = "Graham",
    [PAINTING_MATCH] = "Match",
    [PAINTING_BUST] = "Bust",
    [PAINTING_STAGE] = "Stage",
    [PAINTING_VOID] = "Void",
    [PAINTING_SKULL_AND_ROSES] = "SkullAndRoses",
    [PAINTING_FIGHTERS] = "Fighters",
    [PAINTING_POINTER] = "Pointer",
    [PAINTING_PIG_SCENE] = "Pigscene",
    [PAINTING_BURNING_SKULL] = "BurningSkull",
    [PAINTING_SKELETON] = "Skeleton",
    [PAINTING_DONKEY_KONG] = "DonkeyKong",
};

void net_handle_pkt_login_request(pkt_login_request pkt)
{
    cl.game.seed = pkt.seed;
    cl.state = cl_connected;
    cl.game.our_ent = world_get_entity(pkt.entity_id_or_protocol_version);
    if(!cl.game.our_ent) {
        // create
        net_handle_pkt_named_entity_spawn((pkt_named_entity_spawn) {
           .entity_id = pkt.entity_id_or_protocol_version,
           .name = net_make_string16(cvar_name.string)
        });
        cl.game.our_ent = world_get_entity(pkt.entity_id_or_protocol_version);
    }

    vid_unlock_fps();
    con_printf(CON_STYLE_DARK_GREEN"connected!\n");

    net_free_string16(pkt.username);
}

void net_handle_pkt_handshake(pkt_handshake pkt)
{
    // if pkt.conn_hash[0] == '+', auth to mojang or something

    string16 username = net_make_string16(cvar_name.string);

	cl.state = cl_connecting;
    net_write_pkt_login_request((pkt_login_request) {
        .entity_id_or_protocol_version = PROTOCOL_VERSION,
        .username = username,
        .dimension = 0,
        .seed = 0
    });

	con_printf(CON_STYLE_LIGHT_GRAY "awaiting login approval...\n");

    net_free_string16(pkt.connection_hash_or_username);
    net_free_string16(username);
}

void net_handle_pkt_chat_message(pkt_chat_message pkt)
{
	con_printf("%s\n", utf16toutf8(pkt.message.data, pkt.message.length));
    net_free_string16(pkt.message);
}

void net_handle_pkt_time_update(pkt_time_update pkt)
{
	cl.game.time = pkt.time;
}

EMPTY_HANDLER(pkt_player_inventory)
EMPTY_HANDLER(pkt_spawn_position)
EMPTY_HANDLER(pkt_use_entity)

void net_handle_pkt_update_health(pkt_update_health pkt)
{
    con_printf("new health %d\n", pkt.new_health);
	if(pkt.new_health <= 0) {
		con_printf("u died idiot\n");
	}
}

EMPTY_HANDLER(pkt_respawn)

// not sent by the server {
EMPTY_HANDLER(pkt_flying)
EMPTY_HANDLER(pkt_player_move)
EMPTY_HANDLER(pkt_player_look)
// }

void net_handle_pkt_player_look_move(pkt_player_look_move pkt)
{
    // server tp'd us back so we made a mistake - correct our position

    cl.game.our_ent->y_size = 0;
    cl.game.our_ent->velocity = vec3_from1(0);

    if(!cl.game.unstuck) {
        entity_set_position(cl.game.our_ent, vec3_from(pkt.x, pkt.y_or_stance, pkt.z));
    } else {
        cl.game.unstuck = false;
    }

	cl.game.our_ent->rotation.yaw = -pkt.yaw;
	cl.game.our_ent->rotation.pitch = pkt.pitch;

	// now apologize
    pkt.stance_or_y = cl.game.our_ent->position.y;
    pkt.y_or_stance = cl.game.our_ent->position.y + 0.2f;

    net_write_pkt_player_look_move(pkt);
}

// not sent by the server {
EMPTY_HANDLER(pkt_block_dig)
EMPTY_HANDLER(pkt_place)
// }

EMPTY_HANDLER(pkt_holding_change)
EMPTY_HANDLER(pkt_multiplayer_sleep)
EMPTY_HANDLER(pkt_animation)
EMPTY_HANDLER(pkt_entity_action)

void net_handle_pkt_named_entity_spawn(pkt_named_entity_spawn pkt)
{
    entity ent = {0};
    ent.type = ENTITY_PLAYER;
    ent.id = pkt.entity_id;
    ent.position = vec3_div(vec3_from(pkt.x, pkt.y, pkt.z), 32.0f);
    ent.rotation.yaw = UNPACK_ANGLE(-pkt.yaw);
    ent.rotation.pitch = UNPACK_ANGLE(pkt.pitch);
    ent.y_offset = 1.62f;

    ent.name = mem_alloc(pkt.name.length + 1);
    memcpy(ent.name, utf16toutf8(pkt.name.data, pkt.name.length), pkt.name.length);
    ent.name[pkt.name.length] = 0;

    world_add_entity(&ent);

    net_free_string16(pkt.name);
}

void net_handle_pkt_item_spawn(pkt_item_spawn pkt)
{
    entity ent = {0};
    ent.id = pkt.entity_id;
    ent.type = ENTITY_ITEM;
    ent.position.x = pkt.x / 32.0f;
    ent.position.y = pkt.y / 32.0f;
    ent.position.z = pkt.z / 32.0f;
    ent.rotation.pitch = UNPACK_ANGLE(pkt.pitch);
    ent.rotation.yaw = UNPACK_ANGLE(-pkt.yaw);
    ent.rotation.roll = UNPACK_ANGLE(pkt.roll);
    ent.item.id = pkt.item_id;
    ent.item.count = pkt.count;
    ent.item.metadata = pkt.metadata;
    world_add_entity(&ent);
}

void net_handle_pkt_collect_item(pkt_collect_item pkt)
{
    entity *ent = world_get_entity(pkt.item_entity_id);

    if(!ent)
        return;

    if(pkt.collector_entity_id == cl.game.our_ent->id && cl_2b2tmode.integer != 0) {
        net_write_pkt_chat_message((pkt_chat_message) {
            .message = net_make_string16(va("I just picked up %d %d!\n", ent->item.count, ent->item.id))
        });
    }

    world_remove_entity(ent->id); // todo: pickup animation
}

void net_handle_pkt_nonliving_entity_spawn(pkt_nonliving_entity_spawn pkt)
{
    entity ent = {0};
    ent.id = pkt.entity_id;
    ent.type = nonliving_type_lookup[pkt.type];
    ent.position.x = pkt.x / 32.0f;
    ent.position.y = pkt.y / 32.0f;
    ent.position.z = pkt.z / 32.0f;
    ent.rotation.pitch = 0.0f;
    ent.rotation.yaw = 0.0f;
    ent.velocity.x = pkt.velocity_x / 8000.0f;
    ent.velocity.y = pkt.velocity_y / 8000.0f;
    ent.velocity.z = pkt.velocity_z / 8000.0f;

    switch(ent.type) {
        case ENTITY_MINECART:
            switch(pkt.type) {
                case 10: ent.minecart.type = MINECART_EMPTY; break;
                case 11: ent.minecart.type = MINECART_CHEST; break;
                case 12: ent.minecart.type = MINECART_FURNACE; break;
            }
            break;
        case ENTITY_ITEM_PROJECTILE:
            switch(pkt.type) {
                case 61: ent.projectile.type = PROJECTILE_SNOWBALL; break;
                case 62: ent.projectile.type = PROJECTILE_EGG; break;
                case 63: ent.projectile.type = PROJECTILE_FIREBALL; break;
            }
            ent.projectile.owner = world_get_entity(pkt.owner_id);
            break;
        case ENTITY_FALLINGBLOCK:
            switch(pkt.type) {
                case 70: ent.falling_block.id = BLOCK_SAND; break;
                case 71: ent.falling_block.id = BLOCK_GRAVEL; break;
            }
            break;
        case ENTITY_ARROW:
            ent.arrow.owner = world_get_entity(pkt.owner_id);
            break;
        default:
            break;
    }

    world_add_entity(&ent);
}

void net_handle_pkt_mob_spawn(pkt_mob_spawn pkt)
{
    entity ent = {0};
    ent.id = pkt.entity_id;
    ent.type = pkt.type;
    ent.position.x = pkt.x / 32.0f;
    ent.position.y = pkt.y / 32.0f;
    ent.position.z = pkt.z / 32.0f;
    ent.rotation.pitch = UNPACK_ANGLE(pkt.pitch);
    ent.rotation.yaw = UNPACK_ANGLE(-pkt.yaw);

    world_add_entity(&ent);
}

void net_handle_pkt_entity_painting(pkt_entity_painting pkt)
{
    const char *title;
    entity ent = {0};
    ent.id = pkt.entity_id;
    ent.type = ENTITY_PAINTING;
    ent.position.x = pkt.x / 32.0f;
    ent.position.y = pkt.y / 32.0f;
    ent.position.z = pkt.z / 32.0f;

    title = utf16toutf8(pkt.title.data, pkt.title.length);
    for(ent.painting.type = 0; ent.painting.type < PAINTING_TYPE_COUNT; ent.painting.type++)
        if(!strncmp(title, painting_names[ent.painting.type], pkt.title.length))
            break;

    switch(pkt.face) {
        // todo: set yaw
    }

    world_add_entity(&ent);

    net_free_string16(pkt.title);
}

EMPTY_HANDLER(pkt_stance_update)

void net_handle_pkt_entity_velocity(pkt_entity_velocity pkt)
{
    entity *ent = world_get_entity(pkt.entity_id);
    if(!ent)
        return;

    ent->velocity = vec3_div(vec3_from(pkt.velocity_x, pkt.velocity_y, pkt.velocity_z), 8000.0f);
}

void net_handle_pkt_destroy_entity(pkt_destroy_entity pkt)
{
    world_remove_entity(pkt.entity_id);
}

EMPTY_HANDLER(pkt_entity) // base for pkt_entity_look/move packets, does not do anything by itself

void net_handle_pkt_entity_move(pkt_entity_move pkt)
{
	entity *ent = world_get_entity(pkt.entity_id);

    if(!ent)
        return;

    entity_set_position(ent, vec3_add(ent->position, vec3_div(vec3_from(pkt.rel_x, pkt.rel_y, pkt.rel_z), 32.0f)));
}

void net_handle_pkt_entity_look(pkt_entity_look pkt)
{
    entity *ent = world_get_entity(pkt.entity_id);
    if(!ent)
        return;

    ent->rotation.yaw = UNPACK_ANGLE(-pkt.yaw);
    ent->rotation.pitch = UNPACK_ANGLE(pkt.pitch);
}

void net_handle_pkt_entity_look_move(pkt_entity_look_move pkt)
{
    entity *ent = world_get_entity(pkt.entity_id);
    if(!ent)
        return;

    entity_set_position(ent, vec3_add(ent->position, vec3_div(vec3_from(pkt.rel_x, pkt.rel_y, pkt.rel_z), 32.0f)));
    ent->rotation.yaw = UNPACK_ANGLE(-pkt.yaw);
    ent->rotation.pitch = UNPACK_ANGLE(pkt.pitch);
}

void net_handle_pkt_entity_teleport(pkt_entity_teleport pkt)
{
    entity *ent = world_get_entity(pkt.entity_id);
    if(!ent)
        return;


    entity_set_position(ent, vec3_div(vec3_from(pkt.x, pkt.y, pkt.z), 32.0f));

    net_handle_pkt_entity_look((pkt_entity_look) {
        .entity_id = pkt.entity_id,
        .pitch = pkt.pitch,
        .yaw = pkt.yaw
    });
}

void net_handle_pkt_entity_status(pkt_entity_status pkt)
{
    entity *ent = world_get_entity(pkt.entity_id);
    if(!ent)
        return;

    entity_handle_status_update(ent, pkt.status);
}

void net_handle_pkt_attach_entity(pkt_attach_entity pkt)
{
    entity *ent = world_get_entity(pkt.entity_id);
    if(!ent)
        return;

    if(pkt.vehicle_entity_id == -1) {
        ent->vehicle = NULL; // dismount
    } else {
        ent->vehicle = world_get_entity(pkt.vehicle_entity_id);
    }
}

void net_handle_pkt_entity_metadata(pkt_entity_metadata pkt)
{
    entity *ent = world_get_entity(pkt.entity_id);
    if(!ent)
        return;

    // todo!
}

void net_handle_pkt_pre_chunk(pkt_pre_chunk pkt)
{
	if(pkt.load) {
		world_alloc_chunk(pkt.x, pkt.z);
	} else {
		world_free_chunk(pkt.x, pkt.z);
	}
}

void net_handle_pkt_map_chunk(pkt_map_chunk pkt)
{
	world_load_compressed_chunk_data(pkt.x, pkt.y, pkt.z, (int)pkt.size_x + 1, (int)pkt.size_y + 1, (int)pkt.size_z + 1, (size_t) pkt.data_size, pkt.data);
}

void net_handle_pkt_multi_block_change(pkt_multi_block_change pkt)
{
	if(!world_chunk_exists(pkt.chunk_x, pkt.chunk_z))
		return;

	for(int i = 0; i < pkt.size; i++) {
		int x, y, z;
		int c = pkt.coords[i];
		block_id id = pkt.block_ids[i];
		ubyte metadata = pkt.metadatas[i];

		/* unpack */
		x = ((c >> 12) & 15) + pkt.chunk_x * WORLD_CHUNK_SIZE;
		z = ((c >> 8) & 15) + pkt.chunk_z * WORLD_CHUNK_SIZE;
		y = c & 255;

		world_set_block_id(x, y, z, id);
		world_set_block_metadata(x, y, z, metadata);
	}
}

void net_handle_pkt_block_change(pkt_block_change pkt)
{
	world_set_block_id(pkt.x, pkt.y, pkt.z, pkt.block_id);
	world_set_block_metadata(pkt.x, pkt.y, pkt.z, pkt.metadata);
}

// noteblock: data1 = instrument  data2 = pitch
// piston:    data1 = state       data2 = direction
EMPTY_HANDLER(pkt_block_action)
EMPTY_HANDLER(pkt_explosion)
EMPTY_HANDLER(pkt_sound_effect)
void net_handle_pkt_rain_or_bed_message(pkt_rain_or_bed_message pkt)
{
    switch(pkt.type) {
        case 0: {
            con_printf("tile.bed.notValid\n"); // todo: localization
        } break;
        case 1: {
            // todo: begin raining
        } break;
        case 2: {
            // todo: stop raining
        } break;
    }
}

void net_handle_pkt_thunderbolt(pkt_thunderbolt pkt)
{
    entity e = {0};
    e.id = pkt.entity_id;
    e.type = ENTITY_THUNDERBOLT;
    e.position.x = pkt.x / 32.0f;
    e.position.y = pkt.y / 32.0f;
    e.position.z = pkt.z / 32.0f;

    world_add_entity(&e);
}

void net_handle_pkt_open_window(pkt_open_window pkt)
{
    net_free_string8(pkt.window_title);
}

EMPTY_HANDLER(pkt_close_window)
EMPTY_HANDLER(pkt_window_click)
EMPTY_HANDLER(pkt_set_slot)
EMPTY_HANDLER(pkt_window_items)
EMPTY_HANDLER(pkt_update_progress_bar)
EMPTY_HANDLER(pkt_transaction)

void net_handle_pkt_update_sign(pkt_update_sign pkt)
{
    net_free_string16(pkt.line1);
    net_free_string16(pkt.line2);
    net_free_string16(pkt.line3);
    net_free_string16(pkt.line4);
}

EMPTY_HANDLER(pkt_item_data)
EMPTY_HANDLER(pkt_increment_statistic)

extern entity dummy_ent;
void net_handle_pkt_disconnect(pkt_disconnect pkt)
{
	con_printf("you got kicked: %s\n", utf16toutf8(pkt.reason.data, pkt.reason.length));
	cmd_exec("disconnect");
	cl.state = cl_disconnected;
    cl.game.our_ent = &dummy_ent;

    net_free_string16(pkt.reason);
}
