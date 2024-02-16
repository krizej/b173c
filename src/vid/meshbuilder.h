#ifndef B173C_MESHBUILDER_H
#define B173C_MESHBUILDER_H

#include "mathlib.h"
#include <stdint.h>

#define MESHBUILDER_INDEX_TYPE    uint16_t
#define MESHBUILDER_INDEX_TYPE_GL GL_UNSIGNED_SHORT

void meshbuilder_start(size_t vert_size);

void meshbuilder_finish(
    void **verts_dest, size_t *num_verts_dest,
    MESHBUILDER_INDEX_TYPE **indices_dest, size_t *num_indices_dest);

void meshbuilder_add_index(MESHBUILDER_INDEX_TYPE index);

void meshbuilder_add_vert(void *vert);

void meshbuilder_add_quad(
    void *top_left, void *top_right, void *bottom_left, void *bottom_right);

#endif
