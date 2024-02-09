#include <zlib.h>
#include "world.h"
#include "client/console.h"
#include "hashmap.h"

const static block_data AIR_BLOCK_DATA = {.id = 0, .metadata = 0, .skylight = 15, .blocklight = 0};
const static block_data EMPTY_BLOCK_DATA = {.id = 0, .metadata = 0, .skylight = 0, .blocklight = 0};
const static block_data SOLID_BLOCK_DATA = {.id = 1, .metadata = 0, .skylight = 0, .blocklight = 0};

struct hashmap *world_chunk_map = NULL;
ulong time = 0;

/* functions for the hashmap */
int chunk_compare(const void *a, const void *b, void *udata attr(unused))
{
	const world_chunk *ca = a, *cb = b;
	return !(ca->x == cb->x && ca->z == cb->z);
}

uint64_t chunk_hash(const void *item, uint64_t seed0, uint64_t seed1)
{
	const world_chunk *c = item;
	struct {
		int x, z;
	} coords = {c->x, c->z};
	return hashmap_xxhash3(&coords, sizeof(coords), seed0, seed1);
}

void chunk_free(void *c)
{
	world_chunk *chunk = c;

	world_free_chunk_glbufs(chunk);
	mem_free(chunk->data);
}

void world_init(void)
{
	world_chunk_map = hashmap_new(sizeof(world_chunk), 0, 0, 0, chunk_hash, chunk_compare, chunk_free, NULL);
}

void world_shutdown(void)
{
	hashmap_free(world_chunk_map);
	world_set_time(0);
}

void world_alloc_chunk(int chunk_x, int chunk_z)
{
	world_chunk chunk = {0};

	if(world_chunk_exists(chunk_x, chunk_z))
		world_free_chunk(chunk_x, chunk_z);

	chunk.x = chunk_x;
	chunk.z = chunk_z;
	chunk.data = mem_alloc(WORLD_CHUNK_SIZE * WORLD_CHUNK_SIZE * WORLD_CHUNK_HEIGHT * sizeof(block_data));
	world_init_chunk_glbufs(&chunk);

	/* the chunk is copied by hashmap_set, so it is fine to allocate it on the stack */
	hashmap_set(world_chunk_map, &chunk);
}

void world_free_chunk(int chunk_x, int chunk_z)
{
	world_chunk key = {.x = chunk_x, .z = chunk_z};
	world_chunk *value;

	value = (world_chunk *) hashmap_get(world_chunk_map, &key);
	if(!value)
		return;

	chunk_free(value);

	hashmap_delete(world_chunk_map, &key);
}

bool world_chunk_exists(int chunk_x, int chunk_z)
{
	return world_get_chunk(chunk_x, chunk_z) != NULL;
}

world_chunk *world_get_chunk(int chunk_x, int chunk_z)
{
	world_chunk key = {.x = chunk_x, .z = chunk_z};
	return (world_chunk *) hashmap_get(world_chunk_map, &key);
}

void world_mark_region_for_remesh(int x_start, int y_start, int z_start, int x_end, int y_end, int z_end)
{
	int cxs = x_start >> 4;
	int cxe = x_end >> 4;
	int cys = y_start >> 4;
	int cye = y_end >> 4;
	int czs = z_start >> 4;
	int cze = z_end >> 4;
	for(int cx = cxs; cx <= cxe; cx++) {
		for(int cz = czs; cz <= cze; cz++) {
			for(int cy = cys; cy <= cye; cy++) {
				world_chunk *chunk = world_get_chunk(cx, cz);
				if(!chunk)
					continue;
				if(cy >= 0 && cy < 8) {
					chunk->glbufs[cy].needs_remesh = true;
					chunk->needs_remesh = true;
				}
			}
		}
	}
}

void world_mark_all_for_remesh(void)
{
	void *it;
	size_t i = 0;
	while(hashmap_iter(world_chunk_map, &i, &it)) {
		world_chunk *chunk = it;
		chunk->needs_remesh = true;
		for(int j = 0; j < 8; j++) {
			chunk->glbufs[j].needs_remesh = true;
		}
	}
}

static int inflate_data(ubyte *in, ubyte *out, size_t size_in, size_t size_out)
{
	int ret;
	z_stream strm;

	strm.zalloc = Z_NULL;
	strm.zfree = Z_NULL;
	strm.opaque = Z_NULL;
	strm.avail_in = 0;
	strm.next_in = Z_NULL;
	ret = inflateInit(&strm);
	if(ret != Z_OK) {
		con_printf("inflate_data: zlib error: %s\n", zError(ret));
		return ret;
	}

	strm.avail_in = size_in;
	strm.next_in = in;
	strm.avail_out = size_out;
	strm.next_out = out;

	ret = inflate(&strm, Z_NO_FLUSH);
	switch(ret) {
		case Z_NEED_DICT:
			ret = Z_DATA_ERROR;
			/* fall through */
		case Z_DATA_ERROR:
		case Z_MEM_ERROR:
			return ret;
	}

	inflateEnd(&strm);

	return Z_OK;
}

static int world_set_chunk_data(world_chunk *chunk, const ubyte *data, int x_start, int y_start, int z_start, int x_end, int y_end, int z_end, int data_pos)
{
	for(int x = x_start; x < x_end; x++) {
		for(int z = z_start; z < z_end; z++) {
			for(int y = y_start; y < y_end; y++, data_pos++) {
				int index = IDX_FROM_COORDS(x, y, z);
				chunk->data[index].id = data[data_pos];
			}
		}
	}

	for(int x = x_start; x < x_end; x++) {
		for(int z = z_start; z < z_end; z++) {
			bool next = false;
			for(int y = y_start; y < y_end; y++) {
				int index = IDX_FROM_COORDS(x, y, z);
				ubyte value = data[data_pos];
				chunk->data[index].metadata = !(y & 1) ? (value & 15) : ((value >> 4) & 15);
				if(next)
					data_pos++;
				next = !next;
			}
		}
	}

	for(int x = x_start; x < x_end; x++) {
		for(int z = z_start; z < z_end; z++) {
			bool next = false;
			for(int y = y_start; y < y_end; y++) {
				int index = IDX_FROM_COORDS(x, y, z);
				ubyte value = data[data_pos];
				chunk->data[index].blocklight = !(y & 1) ? (value & 15) : ((value >> 4) & 15);
				if(next)
					data_pos++;
				next = !next;
			}
		}
	}

	for(int x = x_start; x < x_end; x++) {
		for(int z = z_start; z < z_end; z++) {
			bool next = false;
			for(int y = y_start; y < y_end; y++) {
				int index = IDX_FROM_COORDS(x, y, z);
				ubyte value = data[data_pos];
				chunk->data[index].skylight = !(y & 1) ? (value & 15) : ((value >> 4) & 15);
				if(next)
					data_pos++;
				next = !next;
			}
		}
	}

	return data_pos;
}

void world_load_compressed_chunk_data(int x, int y, int z, int size_x, int size_y, int size_z, size_t compressed_size, ubyte *compressed)
{
	// 1 byte id, 0.5 byte metadata, 2x0.5 byte light data
	ubyte data[size_x * size_y * size_z * 5 / 2];
	int i;
	int chunk_x_start = x >> 4;
	int chunk_z_start = z >> 4;
	int chunk_x_end = (x + size_x - 1) >> 4;
	int chunk_z_end = (z + size_z - 1) >> 4;
	int y_start, y_end;

	if((i = inflate_data(compressed, data, compressed_size, sizeof(data))) != Z_OK) {
		con_printf(CON_STYLE_RED"error while decompressing chunk data:"CON_STYLE_WHITE"%s\n", zError(i)); // should this print?
		return;
	}

	y_start = y;
	y_end = y + size_y;

	if(y_start < 0)
		y_start = 0;
	if(y_end > 128)
		y_end = 128;

	i = 0;
	for(int cx = chunk_x_start; cx <= chunk_x_end; cx++) {
		int x_start = x - cx * 16;
		int x_end = x + size_x - cx * 16;

		if(x_start < 0)
			x_start = 0;
		if(x_end > 16)
			x_end = 16;

		for(int cz = chunk_z_start; cz <= chunk_z_end; cz++) {
			int z_start = z - cz * 16;
			int z_end = z + size_z - cz * 16;

			if(z_start < 0)
				z_start = 0;
			if(z_end > 16)
				z_end = 16;

			/* should have been allocated by a PreChunk packet, but we check anyway */
			if(!world_chunk_exists(cx, cz))
				world_alloc_chunk(cx, cz);

			world_mark_region_for_remesh(cx * 16 + x_start - 1, y_start - 1, cz * 16 + z_start - 1, cx * 16 + x_end + 1, y_end + 1, cz * 16 + z_end + 1);
			i = world_set_chunk_data(world_get_chunk(cx, cz), data, x_start, y_start, z_start, x_end, y_end, z_end, i);
		}
	}
}

block_data world_get_block(int x, int y, int z)
{
	static struct {
		int x, z;
		world_chunk *chunk;
	} cache;

	if(y < 0) // below the world
		return SOLID_BLOCK_DATA; // returning solid helps avoid creating mesh faces which won't be seen
	if(y >= 128) // way up in the sky
		return AIR_BLOCK_DATA;

	if(cache.x != (x >> 4) || cache.z != (z >> 4)) {
		cache.x = (x >> 4);
		cache.z = (z >> 4);
		cache.chunk = world_get_chunk(cache.x, cache.z);
	}

	if(cache.chunk != NULL)
		return cache.chunk->data[IDX_FROM_COORDS(x, y, z)];
	return EMPTY_BLOCK_DATA;
}

static ubyte chunk_get_block_lighting(world_chunk *c, int x, int y, int z)
{
	int i = IDX_FROM_COORDS(x, y, z);
	block_data block = c->data[i];
	int sl, bl;
	if(block.id == BLOCK_SLAB_SINGLE || block.id == BLOCK_FARMLAND || block.id == BLOCK_STAIRS_WOOD || block.id == BLOCK_STAIRS_STONE) {
		// fixme?
		int py = world_get_block_lighting(x, y + 1, z);
		int px = world_get_block_lighting(x + 1, y, z);
		int nx = world_get_block_lighting(x - 1, y, z);
		int pz = world_get_block_lighting(x, y, z + 1);
		int nz = world_get_block_lighting(x, y, z - 1);
		if(px > py) {
			py = px;
		}

		if(nx > py) {
			py = nx;
		}

		if(pz > py) {
			py = pz;
		}

		if(nz > py) {
			py = nz;
		}

		return py;
	}
	sl = block.skylight;
	bl = block.blocklight;
	return (bl > sl ? bl : sl) & 15;
}

ubyte world_get_block_lighting(int x, int y, int z)
{
	world_chunk *cp;
	if(y < 0)
		return 0;
	if(y >= 128)
		return 15;
	cp = world_get_chunk(x >> 4, z >> 4);
	if(!cp)
		return 0;
	return chunk_get_block_lighting(cp, x & 15, y, z & 15);
}


void world_set_block(int x, int y, int z, block_data data)
{
	world_chunk *chunk;

	if(y < 0 || y >= 128)
		return;

	chunk = world_get_chunk(x >> 4, z >> 4);
	if(!chunk)
		return;

	world_mark_region_for_remesh(x-1, y-1, z-1, x+1, y+1, z+1);

	chunk->data[IDX_FROM_COORDS(x, y, z)] = data;
}

void world_set_block_id(int x, int y, int z, block_id new_id)
{
	block_data data = world_get_block(x, y, z);
	data.id = new_id;
	world_set_block(x, y, z, data);
}

void world_set_block_metadata(int x, int y, int z, byte new_metadata)
{
	block_data data = world_get_block(x, y, z);
	data.metadata = new_metadata;
	world_set_block(x, y, z, data);
}

void world_set_time(ulong t)
{
	time = t;
}

ulong world_get_time(void)
{
	return time;
}


#define is_between(t, a, b) t >= a && t <= b
vec4 world_calculate_sky_color(void)
{
	static vec4 color = {.a = 1};

	int t = time % 24000;

	if(is_between(t, 14000, 22000)) {
		// night
		color.r = 0.01f;
		color.g = 0.01f;
		color.b = 0.03f;
	} else if(is_between(t, 11000, 14000)) {
		// start of night
		// interpolate between day (11000) and night (14000)
		float f = (float) (t - 11000) / 3000.0f;
		color.r = f * 0.01f + (1.0f - f) * 0.55f;
		color.g = f * 0.01f + (1.0f - f) * 0.7f;
		color.b = f * 0.03f + (1.0f - f) * 1.0f;
	} else if(is_between(t, 22000, 24000)) {
		// start of day
		// interpolate between night (22000) and day (24000)
		float f = (float) (t - 22000) / 2000.0f;
		color.r = f * 0.55f + (1.0f - f) * 0.01f;
		color.g = f * 0.7f + (1.0f - f) * 0.01f;
		color.b = f * 1.0f + (1.0f - f) * 0.03f;
	} else {
		// day
		color.r = 0.55f;
		color.g = 0.7f;
		color.b = 1.0f;
	}

	return color;
}

float world_calculate_sun_angle(void)
{
	return 0.0f; // todo
}

float world_calculate_sky_light_modifier(void)
{
	float lmod;
	int t = time % 24000;
	if(is_between(t, 14000, 22000)) {
		// darkest night
		lmod = 1.0f;
	} else if(is_between(t, 11000, 14000)) {
		// start of night
		// interpolate between day (11000) and night (14000)
		float f = (float) (t - 11000) / 3000.0f;
		lmod = f * 1.0f + (1.0f - f) * 15.0f;
	} else if(is_between(t, 22000, 24000)) {
		// start of day
		// interpolate between night (22000) and day (24000)
		float f = (float) (t - 22000) / 2000.0f;
		lmod = f * 15.0f + (1.0f - f) * 1.0f;
	} else {
		// day
		lmod = 15.0f;
	}
	return lmod / 15.0f;
}

struct trace_result world_trace_ray(vec3 origin, vec3 dir, float maxlen)
{
	struct trace_result res = {0};
	float dist = 0.0f;
	int step[3];
	block_face ofsface[3];
	vec3 delta;
	vec3 edgedist;

	for(int axis = 0; axis < 3; axis++) {
		delta.array[axis] = dir.array[axis] == 0.0f ? 9999.0f : fabsf(1.0f / dir.array[axis]);

		if(dir.array[axis] < 0.0f) {
			step[axis] = -1;
			ofsface[axis] = 1;
			edgedist.array[axis] = (origin.array[axis] - floorf(origin.array[axis])) * delta.array[axis];
		} else {
			step[axis] = 1;
			ofsface[axis] = 0;
			edgedist.array[axis] = (floorf(origin.array[axis]) + 1.0 - origin.array[axis]) * delta.array[axis];
		}
	}

	res.x = floorf(origin.x);
	res.y = floorf(origin.y);
	res.z = floorf(origin.z);

	res.reached_end = true; // by default

	while(dist < maxlen) {
		if(edgedist.x < edgedist.y && edgedist.x < edgedist.z) {
			edgedist.x += delta.x;
			res.x += step[0];
			res.hit_face = BLOCK_FACE_X_NEG + ofsface[0];
		} else if(edgedist.y < edgedist.z) {
			edgedist.y += delta.y;
			res.y += step[1];
			res.hit_face = BLOCK_FACE_Y_NEG + ofsface[1];
		} else {
			edgedist.z += delta.z;
			res.z += step[2];
			res.hit_face = BLOCK_FACE_Z_NEG + ofsface[2];
		}

		res.block = world_get_block(res.x, res.y, res.z);
		if(block_is_collidable(res.block)) {
			res.reached_end = false;
			break;
		}

		dist += 1.0f;
	}

	return res;
}

