#include "net/packets.h"
#include "net_internal.h"
#include "client/client.h"
#include "client/console.h"
#include "client/cvar.h"
#include "../client/client.h"

void net_write_packets(void)
{
	static bool sent_handshake = false;

	if(cl.state == cl_disconnected && sent_handshake) {
		// this is executed at the end of net_process (in net.c)
		// used to reset this handshake flag
		sent_handshake = false;
		return;
	}

	if(cl.state == cl_connecting && !sent_handshake) {
		// send handshake to the server
        string16 username = net_make_string16(cvar_name.string);
        net_write_pkt_handshake((pkt_handshake) {
            .connection_hash_or_username = username
        });
        net_free_string16(username);

		sent_handshake = true;
		con_printf(CON_STYLE_GRAY "awaiting handshake...\n");
	}

	if(cl.state != cl_connected)
		return;

	// write movement packet depending on what the user did
	if(cl.game.moved && cl.game.rotated) {
		cl.game.moved = false;
		cl.game.rotated = false;
        net_write_pkt_player_look_move((pkt_player_look_move) {
            .x = cl.game.our_ent->position.x,
            .stance_or_y = cl.game.our_ent->bbox.mins.y,
            .y_or_stance = cl.game.our_ent->position.y,
            .z = cl.game.our_ent->position.z,
            .yaw = -cl.game.our_ent->rotation.yaw,
            .pitch = cl.game.our_ent->rotation.pitch,
            .on_ground = cl.game.our_ent->onground
        });
	} else if(cl.game.moved) {
        net_write_pkt_player_move((pkt_player_move) {
			.x = cl.game.our_ent->position.x,
			.y = cl.game.our_ent->bbox.mins.y,
			.stance = cl.game.our_ent->position.y,
			.z = cl.game.our_ent->position.z,
			.on_ground = cl.game.our_ent->onground
        });
		cl.game.moved = false;
	} else if(cl.game.rotated) {
        net_write_pkt_player_look((pkt_player_look) {
			.yaw = -cl.game.our_ent->rotation.yaw,
			.pitch = cl.game.our_ent->rotation.pitch,
			.on_ground = cl.game.our_ent->onground
        });
		cl.game.rotated = false;
	} else {
        net_write_pkt_flying((pkt_flying) {
			.on_ground = cl.game.our_ent->onground
		});
	}
}

void write_window_items_payload(struct ni_wi_payload data)
{
    net_write_short(data.item_id);
    if(data.item_id != -1) {
        net_write_byte(data.count);
        net_write_short(data.metadata);
    }
}

void write_metadata(struct net_entity_metadata data attr(unused))
{
    // todo :-(
    net_write_byte(0x7f);
}

#define UBYTE(name) net_write_byte(this.name);
#define BYTE(name) net_write_byte(this.name);
#define SHORT(name) net_write_short(this.name);
#define INT(name) net_write_int(this.name);
#define LONG(name) net_write_long(this.name);
#define FLOAT(name) net_write_float(this.name);
#define DOUBLE(name) net_write_double(this.name);
#define STRING8(name) net_write_string8(this.name);
#define STRING16(name) net_write_string16(this.name);
#define BOOL(name) net_write_byte(this.name);
#define METADATA(name) write_metadata(this.name);
#define WINDOW_ITEMS_PAYLOAD(name) write_window_items_payload(this.name);

#define OPT(cond, stuff) if(cond) { stuff }
#define BUF(type, name, size) for(size_t i = 0; i < (size_t) (size); i++) type(name[i])

#define PACKET(id, name, stuff) void net_write_pkt_ ## name (pkt_ ## name this) { net_write_byte(id); stuff }

#include "packets_def.h"
