#ifndef PACKET
#define PACKET_DEFINED
#define UBYTE(name)
#define BYTE(name)
#define SHORT(name)
#define INT(name)
#define LONG(name)
#define FLOAT(name)
#define DOUBLE(name)
#define STRING8(name)
#define STRING16(name)
#define BOOL(name)
#define METADATA(name)
#define WINDOW_ITEMS_PAYLOAD(name)
#define OPT(cond, stuff)
#define BUF(type, name, size)
#define PACKET(id, name, stuff)
#endif // #ifndef PACKET

PACKET(0x01, login_request,
       INT(entity_id_or_protocol_version)
       STRING16(username)
       LONG(seed)
       BYTE(dimension)
)

PACKET(0x02, handshake,
       STRING16(connection_hash_or_username)
)

PACKET(0x03, chat_message,
       STRING16(message)
)

PACKET(0x04, time_update,
       LONG(time)
)

PACKET(0x05, player_inventory,
       INT(entity_id)
       SHORT(slot)
       SHORT(item_id)
       SHORT(metadata)
)

PACKET(0x06, spawn_position,
       INT(x)
       INT(y)
       INT(z)
)

PACKET(0x07, use_entity,
       INT(user_id)
       INT(target_id)
       BOOL(is_left_click)
)

PACKET(0x08, update_health,
       SHORT(new_health)
)

PACKET(0x09, respawn,
       BYTE(dimension)
)

PACKET(0x0A, flying,
       BOOL(on_ground)
)

PACKET(0x0B, player_move,
       DOUBLE(x)
       DOUBLE(y)
       DOUBLE(stance)
       DOUBLE(z)
       BOOL(on_ground)
)

PACKET(0x0C, player_look,
       FLOAT(yaw)
       FLOAT(pitch)
       BOOL(on_ground)
)

// stance and y are swapped when sending to server
// client to server order: y, stance
PACKET(0x0D, player_look_move,
       DOUBLE(x)
       DOUBLE(stance_or_y)
       DOUBLE(y_or_stance)
       DOUBLE(z)
       FLOAT(yaw)
       FLOAT(pitch)
       BOOL(on_ground)
)

PACKET(0x0E, block_dig,
       BYTE(status)
       INT(x)
       BYTE(y)
       INT(z)
       BYTE(face)
)

PACKET(0x0F, place,
       INT(x)
       BYTE(y)
       INT(z)
       BYTE(face)
       SHORT(item_id)
       OPT(this.item_id >= 0,
           BYTE(amount)
           SHORT(metadata)
       )
)

PACKET(0x10, holding_change,
       SHORT(slot_id)
)

PACKET(0x11, multiplayer_sleep,
       INT(entity_id)
       BYTE(unknown)
       INT(x)
       BYTE(y)
       INT(z)
)

PACKET(0x12, animation,
       INT(entity_id)
       BYTE(animation)
)

PACKET(0x13, entity_action,
       INT(entity_id)
       BYTE(action)
)

PACKET(0x14, named_entity_spawn,
       INT(entity_id)
       STRING16(name)
       INT(x)
       INT(y)
       INT(z)
       BYTE(yaw)
       BYTE(pitch)
       SHORT(item)
)

PACKET(0x15, item_spawn,
       INT(entity_id)
       SHORT(item_id)
       BYTE(count)
       SHORT(metadata)
       INT(x)
       INT(y)
       INT(z)
       BYTE(yaw)
       BYTE(pitch)
       BYTE(roll)
)

PACKET(0x16, collect_item,
       INT(item_entity_id)
       INT(collector_entity_id)
)

PACKET(0x17, nonliving_entity_spawn,
       INT(entity_id)
       BYTE(type)
       INT(x)
       INT(y)
       INT(z)
       INT(owner_id)
       OPT(this.owner_id > 0,
           SHORT(velocity_x)
           SHORT(velocity_y)
           SHORT(velocity_z)
       )
)

PACKET(0x18, mob_spawn,
       INT(entity_id)
       BYTE(type)
       INT(x)
       INT(y)
       INT(z)
       BYTE(yaw)
       BYTE(pitch)
       METADATA(metadata)
)

PACKET(0x19, entity_painting,
       INT(entity_id)
       STRING16(title)
       INT(x)
       INT(y)
       INT(z)
       INT(face)
)

PACKET(0x1B, stance_update,
       FLOAT(unknown1)
       FLOAT(unknown2)
       FLOAT(unknown3)
       FLOAT(unknown4)
       BOOL(unknown5)
       BOOL(unknown6)
)

PACKET(0x1C, entity_velocity,
       INT(entity_id)
       SHORT(velocity_x)
       SHORT(velocity_y)
       SHORT(velocity_z)
)

PACKET(0x1D, destroy_entity,
       INT(entity_id)
)

PACKET(0x1E, entity,
       INT(entity_id)
)

PACKET(0x1F, entity_move,
       INT(entity_id)
       BYTE(rel_x)
       BYTE(rel_y)
       BYTE(rel_z)
)

PACKET(0x20, entity_look,
       INT(entity_id)
       BYTE(yaw)
       BYTE(pitch)
)

PACKET(0x21, entity_look_move,
       INT(entity_id)
       BYTE(rel_x)
       BYTE(rel_y)
       BYTE(rel_z)
       BYTE(yaw)
       BYTE(pitch)
)

PACKET(0x22, entity_teleport,
       INT(entity_id)
       INT(x)
       INT(y)
       INT(z)
       BYTE(yaw)
       BYTE(pitch)
)

PACKET(0x26, entity_status,
       INT(entity_id)
       BYTE(status)
)

PACKET(0x27, attach_entity,
       INT(entity_id)
       INT(vehicle_entity_id)
)

PACKET(0x28, entity_metadata,
       INT(entity_id)
       METADATA(metadata)
)

PACKET(0x32, pre_chunk,
       INT(x)
       INT(z)
       BOOL(load)
)

PACKET(0x33, map_chunk,
       INT(x)
       SHORT(y)
       INT(z)
       BYTE(size_x)
       BYTE(size_y)
       BYTE(size_z)
       INT(data_size)
       BUF(UBYTE, data, this.data_size)
)

PACKET(0x34, multi_block_change,
       INT(chunk_x)
       INT(chunk_z)
       SHORT(size)
       BUF(SHORT, coords, this.size)
       BUF(BYTE, block_ids, this.size)
       BUF(BYTE, metadatas, this.size)
)

PACKET(0x35, block_change,
       INT(x)
       BYTE(y)
       INT(z)
       BYTE(block_id)
       BYTE(metadata)
)

PACKET(0x36, block_action,
       INT(x)
       SHORT(y)
       INT(z)
       BYTE(data1)
       BYTE(data2)
)

PACKET(0x3C, explosion,
       DOUBLE(x)
       DOUBLE(y)
       DOUBLE(z)
       FLOAT(radius)
       INT(count)
       BUF(BYTE, affected_offsets, this.count * 3)
)

PACKET(0x3D, sound_effect,
       INT(effect_id)
       INT(x)
       BYTE(y)
       INT(z)
       INT(sound_data)
)

// this packet is stupid :/
PACKET(0x46, rain_or_bed_message,
       BYTE(type)
)

PACKET(0x47, thunderbolt,
       INT(entity_id)
       BYTE(type)
       INT(x)
       INT(y)
       INT(z)
)

PACKET(0x64, open_window,
       BYTE(window_id)
       BYTE(inventory_type)
       STRING8(window_title)
       BYTE(slot_count)
)

PACKET(0x65, close_window,
       BYTE(window_id)
)

PACKET(0x66, window_click,
       BYTE(window_id)
       SHORT(slot)
       BYTE(mouse_button)
       SHORT(action_number)
       BOOL(shift)
       SHORT(item_id)
       OPT(this.item_id != -1,
           BYTE(item_count)
           SHORT(item_metadata)
       )
)

PACKET(0x67, set_slot,
       BYTE(window_id)
       SHORT(slot)
       SHORT(item_id)
       OPT(this.item_id != -1,
           BYTE(item_count)
           SHORT(item_metadata)
       )
)

PACKET(0x68, window_items,
       BYTE(window_id)
       SHORT(count)
       BUF(WINDOW_ITEMS_PAYLOAD, items, this.count)
)

PACKET(0x69, update_progress_bar,
       BYTE(window_id)
       SHORT(bar_id)
       SHORT(progress)
)

PACKET(0x6A, transaction,
       BYTE(window_id)
       SHORT(action_number)
       BOOL(accepted)
)

PACKET(0x82, update_sign,
       INT(x)
       SHORT(y)
       INT(z)
       STRING16(line1)
       STRING16(line2)
       STRING16(line3)
       STRING16(line4)
)

PACKET(0x83, item_data,
       SHORT(item_type)
       SHORT(item_id)
       UBYTE(data_length)
       BUF(BYTE, data, this.data_length)
)

PACKET(0xC8, increment_statistic,
       INT(stat_id)
       BYTE(amount)
)

PACKET(0xFF, disconnect,
       STRING16(reason)
)

#ifdef PACKET_DEFINED
#undef UBYTE
#undef BYTE
#undef SHORT
#undef INT
#undef LONG
#undef FLOAT
#undef DOUBLE
#undef STRING8
#undef STRING16
#undef BOOL
#undef METADATA
#undef WINDOW_ITEMS_PAYLOAD
#undef OPT
#undef BUF
#undef PACKET
#endif
