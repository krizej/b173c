/*
 * fixme: replace all reallocs with mem_realloc
 *
 */

#include "meshbuilder.h"
#include "common.h"
#include "hashmap.c/hashmap.h"

#define DEFAULT_CAPACITY 1024

static struct {
	size_t count;
	size_t capacity;
	size_t elem_size;
	void *data;
} vertices;

static struct {
	size_t count;
	size_t capacity;
	MESHBUILDER_INDEX_TYPE *data;
} indices;

struct meshbuilder_pair {
	void *vertex;
	MESHBUILDER_INDEX_TYPE index;
};

static struct hashmap *vtx_2_idx_map;
static bool hashmap_init = false;

int cmpfunc(const void *v1, const void *v2, void *_ attr(unused))
{
	const struct meshbuilder_pair *p1 = v1, *p2 = v2;
	return memcmp(p1->vertex, p2->vertex, vertices.elem_size);
}

uint64_t hashfunc(const void *v, uint64_t seed0, uint64_t seed1)
{
	const struct meshbuilder_pair *p = v;
	return hashmap_xxhash3(p->vertex, vertices.elem_size, seed0, seed1);
}

void meshbuilder_start(size_t vtx_size)
{
	vertices.elem_size = vtx_size;
	vertices.count = 0;
	vertices.capacity = DEFAULT_CAPACITY;
	vertices.data = mem_alloc(vertices.capacity * vertices.elem_size);

	indices.count = 0;
	indices.capacity = DEFAULT_CAPACITY;
	indices.data = mem_alloc(indices.capacity * sizeof(MESHBUILDER_INDEX_TYPE));

	if(!hashmap_init) {
		vtx_2_idx_map = hashmap_new(sizeof(struct meshbuilder_pair), 0, 0, 0, hashfunc, cmpfunc, NULL, NULL);
		hashmap_init = true;
	}
}

void meshbuilder_finish(void **verts_dest, size_t *num_verts_dest, MESHBUILDER_INDEX_TYPE **indices_dest, size_t *num_indices_dest)
{
	*verts_dest = reallocarray(vertices.data, vertices.count, vertices.elem_size);
	*num_verts_dest = vertices.count;

	*indices_dest = reallocarray(indices.data, indices.count, sizeof(MESHBUILDER_INDEX_TYPE));
	*num_indices_dest = indices.count;

	memset(&vertices, 0, sizeof(vertices));
	memset(&indices, 0, sizeof(indices));

	hashmap_clear(vtx_2_idx_map, false);
}

void meshbuilder_add_index(MESHBUILDER_INDEX_TYPE idx)
{
	indices.data[indices.count] = idx;
	indices.count++;

	if(indices.count >= indices.capacity) {
		indices.capacity += DEFAULT_CAPACITY;
		indices.data = reallocarray(indices.data, indices.capacity, sizeof(MESHBUILDER_INDEX_TYPE));
	}
}

void meshbuilder_add_vert(void *v)
{
	//struct meshbuilder_pair key = {.vertex = v};
	//const struct meshbuilder_pair *value;

	/*if((value = hashmap_get(vtx_2_idx_map, &key))) {
		// vertex already exists
		meshbuilder_add_index(value->index);
		return;
	}*/

	memcpy((byte *) vertices.data + vertices.count * vertices.elem_size, v, vertices.elem_size);
	meshbuilder_add_index(vertices.count);
	vertices.count++;

	if(vertices.count >= vertices.capacity) {
		vertices.capacity += DEFAULT_CAPACITY;
		vertices.data = reallocarray(vertices.data, vertices.capacity, vertices.elem_size);
	}
}

void meshbuilder_add_quad(void *tl, void *tr, void *bl, void *br)
{
	meshbuilder_add_vert(tl);
	meshbuilder_add_vert(bl);
	meshbuilder_add_vert(tr);

	meshbuilder_add_vert(bl);
	meshbuilder_add_vert(br);
	meshbuilder_add_vert(tr);
}
