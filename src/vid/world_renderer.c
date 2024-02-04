#include "vid.h"
#include "mathlib.h"
#include "common.h"
#include "client/client.h"
#include "glad/glad.h"
#include "game/world.h"
#include "client/console.h"
#include "hashmap.h"
#include "meshbuilder.h"
#include "assets.h"

// todo: mesher thread

#define VIEW_HEIGHT 1.62f

static mat4 view_mat = {0};
static mat4 proj_mat = {0};
static uint gl_world_vao;
static uint gl_world_texture; // fixme
static uint loc_chunkpos, loc_proj, loc_view, loc_nightlightmod;

struct {
	struct plane {
		vec3 normal;
		float dist;
	} top, bottom, right, left, far, near;
} frustum = {0};

struct block {
	float x, y, z;
	// float data1, data2;
} __attribute__((packed));

static void recalculate_projection_matrix(void);

cvar fov = {"fov", "90", recalculate_projection_matrix};
cvar r_zfar = {"r_zfar", "256", recalculate_projection_matrix};
cvar r_znear = {"r_znear", "0.1", recalculate_projection_matrix};
cvar r_max_remeshes = {"r_max_remeshes", "2"};
cvar gl_polygon_mode = {"gl_polygon_mode", "GL_FILL"};

extern cvar vid_width, vid_height;

extern struct gl_state gl;

static float get_dist_to_plane(const struct plane *p, const vec3 point)
{
	return vec3_dot(p->normal, point) - p->dist;
}

static struct plane make_plane(vec3 point, vec3 normal)
{
	struct plane p;
	vec3_copy(p.normal, normal);
	p.dist = vec3_dot(normal, point);
	return p;
}

static bool is_visible_on_frustum(const vec3 point, float radius)
{
	return
		get_dist_to_plane(&frustum.right, point) > -radius &&
		get_dist_to_plane(&frustum.left, point) > -radius &&
		get_dist_to_plane(&frustum.bottom, point) > -radius &&
		get_dist_to_plane(&frustum.top, point) > -radius &&
		get_dist_to_plane(&frustum.far, point) > -radius &&
		get_dist_to_plane(&frustum.near, point) > -radius;
}

void world_renderer_update_chunk_visibility(world_chunk *chunk);

static void update_view_matrix_and_frustum(void)
{
	float half_h = r_zfar.value * tanf(DEG2RAD(fov.value) * 0.5f);
	float half_w = half_h * (vid_width.value / vid_height.value);
	vec3 right, up, forward;
	vec3 fwdFar, fwdNear;
	vec3 normal;
	vec3 tmp;

	cl.game.pos[1] += VIEW_HEIGHT;

	/* find camera vectors */
	cam_angles(forward, right, up, cl.game.rot[1], cl.game.rot[0]);
	vec3_mul_scalar(fwdFar, forward, r_zfar.value);
	vec3_mul_scalar(fwdNear, forward, r_znear.value);

	/* update frustum */
	vec3_add(tmp, cl.game.pos, fwdFar);
	vec3_invert(normal, forward);
	frustum.far = make_plane(tmp, normal);

	vec3_add(tmp, cl.game.pos, fwdNear);
	vec3_copy(normal, forward);
	frustum.near = make_plane(tmp, normal);

	vec3_mul_scalar(tmp, right, half_w);
	vec3_sub(tmp, fwdFar, tmp);
	vec3_cross(normal, tmp, up);
	frustum.left = make_plane(cl.game.pos, normal);

	vec3_mul_scalar(tmp, right, half_w);
	vec3_add(tmp, fwdFar, tmp);
	vec3_cross(normal, up, tmp);
	frustum.right = make_plane(cl.game.pos, normal);

	vec3_mul_scalar(tmp, up, half_h);
	vec3_sub(tmp, fwdFar, tmp);
	vec3_cross(normal, right, tmp);
	frustum.top = make_plane(cl.game.pos, normal);

	vec3_mul_scalar(tmp, up, half_h);
	vec3_add(tmp, fwdFar, tmp);
	vec3_cross(normal, tmp, right);
	frustum.bottom = make_plane(cl.game.pos, normal);

	/* update view matrix */
	mat_view(view_mat, cl.game.pos, cl.game.rot);

	cl.game.pos[1] -= VIEW_HEIGHT;

	/* update visibility of chunks */
	if(world_chunk_map != NULL && hashmap_count(world_chunk_map) > 0) {
		size_t i = 0;
		void *it;
		while(hashmap_iter(world_chunk_map, &i, &it)) {
			world_chunk *chunk = (world_chunk *) it;
			world_renderer_update_chunk_visibility(chunk);
		}
	}
}

static void recalculate_projection_matrix(void)
{
	mat_projection(proj_mat, fov.value, (vid_width.value / vid_height.value), r_znear.value, r_zfar.value);
	update_view_matrix_and_frustum();
}

void world_renderer_init(void)
{
	asset_image *terrain_asset;

	cvar_register(&fov);
	cvar_register(&r_zfar);
	cvar_register(&r_znear);
	cvar_register(&r_max_remeshes);
	cvar_register(&gl_polygon_mode);

	recalculate_projection_matrix();

	loc_chunkpos = glGetUniformLocation(gl.shader3d, "CHUNK_POS");
	loc_view = glGetUniformLocation(gl.shader3d, "VIEW");
	loc_proj = glGetUniformLocation(gl.shader3d, "PROJECTION");

	loc_nightlightmod = glGetUniformLocation(gl.shader3d, "NIGHTTIME_LIGHT_MODIFIER");

	/* init vao */
	glGenVertexArrays(1, &gl_world_vao);

	glBindVertexArray(gl_world_vao);
	glVertexAttribFormat(0, 3, GL_FLOAT, GL_FALSE, 0);
	glVertexAttribIFormat(1, 1, GL_UNSIGNED_INT, 12);
	glVertexAttribBinding(0, 0);
	glVertexAttribBinding(1, 0);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glBindVertexArray(0);

	/* load terrain texture */
	terrain_asset = asset_get_image(ASSET_TEXTURE_TERRAIN);

	glGenTextures(1, &gl_world_texture);
	glBindTexture(GL_TEXTURE_2D, gl_world_texture);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, terrain_asset->width, terrain_asset->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, terrain_asset->data);

	glBindTexture(GL_TEXTURE_2D, 0);
}

void world_renderer_shutdown(void)
{
}

static void add_block_face(struct world_vertex v, block_face face)
{
	const vec3 offsets[6][4] = {
		[BLOCK_FACE_Y_NEG] = {{0, 0, 0}, {0, 0, 1}, {1, 0, 0}, {1, 0, 1}},
		[BLOCK_FACE_Y_POS] = {{0, 1, 0}, {1, 1, 0}, {0, 1, 1}, {1, 1, 1}},
		[BLOCK_FACE_Z_NEG] = {{1, 1, 0}, {0, 1, 0}, {1, 0, 0}, {0, 0, 0}},
		[BLOCK_FACE_Z_POS] = {{0, 1, 1}, {1, 1, 1}, {0, 0, 1}, {1, 0, 1}},
		[BLOCK_FACE_X_NEG] = {{0, 1, 0}, {0, 1, 1}, {0, 0, 0}, {0, 0, 1}},
		[BLOCK_FACE_X_POS] = {{1, 1, 1}, {1, 1, 0}, {1, 0, 1}, {1, 0, 0}}
	};

	struct world_vertex tl, tr, bl, br;

	tl = v;
	tr = v;
	bl = v;
	br = v;

	tl.x += offsets[face][0][0];
	tl.y += offsets[face][0][1];
	tl.z += offsets[face][0][2];

	tr.x += offsets[face][1][0];
	tr.y += offsets[face][1][1];
	tr.z += offsets[face][1][2];

	bl.x += offsets[face][2][0];
	bl.y += offsets[face][2][1];
	bl.z += offsets[face][2][2];

	br.x += offsets[face][3][0];
	br.y += offsets[face][3][1];
	br.z += offsets[face][3][2];

	meshbuilder_add_quad(&tl, &tr, &bl, &br);
}

void render_fluid(int x, int y, int z, block_data self)
{
	if(block_should_face_be_rendered(x, y, z, self, BLOCK_FACE_Y_POS)) {
		float h_00, h_10, h_01, h_11;
		block_properties props = block_get_properties(self.id);
		struct world_vertex v1, v2, v3, v4;
		ubyte light;
		//vec3 flowdir; todo
		//float flowangle;

		//if(!block_get_fluid_flow_direction(flowdir, x, y, z, self.id)) {
		//	flowangle = 0.0f;
		//} else {
		//	flowangle = RAD2DEG(atan2f(flowdir[2], flowdir[0]) - PI * 0.5f);
		//	con_printf("%f %f %f -> %f\n", flowdir[0], flowdir[1], flowdir[2], flowangle);
		//}


		h_00 = block_fluid_get_height(x, y, z, self.id);
		h_10 = block_fluid_get_height(x + 1, y, z, self.id);
		h_01 = block_fluid_get_height(x, y, z + 1, self.id);
		h_11 = block_fluid_get_height(x + 1, y, z + 1, self.id);

		light = world_get_block_lighting(x, y, z);
		v1 = world_make_vertex((x&15), (y&15) + h_00, (z&15), props.texture_indices[BLOCK_FACE_Y_POS], BLOCK_FACE_Y_POS, light);
		v2 = world_make_vertex((x&15)+1, (y&15) + h_10, (z&15), props.texture_indices[BLOCK_FACE_Y_POS], BLOCK_FACE_Y_POS, light);
		v3 = world_make_vertex((x&15), (y&15) + h_01, (z&15)+1, props.texture_indices[BLOCK_FACE_Y_POS], BLOCK_FACE_Y_POS, light);
		v4 = world_make_vertex((x&15)+1, (y&15) + h_11, (z&15)+1, props.texture_indices[BLOCK_FACE_Y_POS], BLOCK_FACE_Y_POS, light);

		//if(flowangle < 0)
		//	flowangle += 360.0f;
		//v1.extradata = (short) flowangle;
		//v2.extradata = (short) flowangle;
		//v3.extradata = (short) flowangle;
		//v4.extradata = (short) flowangle;

		meshbuilder_add_quad(&v1, &v2, &v3, &v4);
	}
}

void render_null(int x, int y, int z, block_data self)
{

}

void render_cross(int x, int y, int z, block_data self)
{
	struct world_vertex tl, tr, bl, br;
	ubyte light = world_get_block_lighting(x, y, z);
	ubyte texture = block_get_texture_index(self.id, 0, self.metadata);

	x &= 15;
	y &= 15;
	z &= 15;

	tl = world_make_vertex(x+1, y+1, z, texture, 1, light);
	tr = world_make_vertex(x, y+1, z+1, texture, 1, light);
	bl = world_make_vertex(x+1, y, z, texture, 1, light);
	br = world_make_vertex(x, y, z+1, texture, 1, light);
	meshbuilder_add_quad(&tl, &tr, &bl, &br);
	meshbuilder_add_quad(&tr, &tl, &br, &bl);

	tl = world_make_vertex(x, y+1, z, texture, 1, light);
	tr = world_make_vertex(x+1, y+1, z+1, texture, 1, light);
	bl = world_make_vertex(x, y, z, texture, 1, light);
	br = world_make_vertex(x+1, y, z+1, texture, 1, light);
	meshbuilder_add_quad(&tl, &tr, &bl, &br);
	meshbuilder_add_quad(&tr, &tl, &br, &bl);
}

void render_crops(int x, int y, int z, block_data self)
{
	struct world_vertex tl, tr, bl, br;
	ubyte light = world_get_block_lighting(x, y, z);
	ubyte texture = block_get_texture_index(self.id, 0, self.metadata);
	float x0, x1, z0, z1;

	x &= 15;
	y &= 15;
	z &= 15;

	x0 = x + 0.25f;
	x1 = x + 0.75f;
	z0 = z;
	z1 = z + 1.0f;

	tl = world_make_vertex(x0, y+1, z0, texture, 1, light);
	tr = world_make_vertex(x0, y+1, z1, texture, 1, light);
	bl = world_make_vertex(x0, y, z0, texture, 1, light);
	br = world_make_vertex(x0, y, z1, texture, 1, light);
	meshbuilder_add_quad(&tl, &tr, &bl, &br);
	meshbuilder_add_quad(&tr, &tl, &br, &bl);

	tl = world_make_vertex(x1, y+1, z0, texture, 1, light);
	tr = world_make_vertex(x1, y+1, z1, texture, 1, light);
	bl = world_make_vertex(x1, y, z0, texture, 1, light);
	br = world_make_vertex(x1, y, z1, texture, 1, light);
	meshbuilder_add_quad(&tl, &tr, &bl, &br);
	meshbuilder_add_quad(&tr, &tl, &br, &bl);

	x0 = x;
	x1 = x + 1.0f;
	z0 = z + 0.25f;
	z1 = z + 0.75f;


	tl = world_make_vertex(x0, y+1, z0, texture, 1, light);
	tr = world_make_vertex(x1, y+1, z0, texture, 1, light);
	bl = world_make_vertex(x0, y, z0, texture, 1, light);
	br = world_make_vertex(x1, y, z0, texture, 1, light);
	meshbuilder_add_quad(&tl, &tr, &bl, &br);
	meshbuilder_add_quad(&tr, &tl, &br, &bl);

	tl = world_make_vertex(x0, y+1, z1, texture, 1, light);
	tr = world_make_vertex(x1, y+1, z1, texture, 1, light);
	bl = world_make_vertex(x0, y, z1, texture, 1, light);
	br = world_make_vertex(x1, y, z1, texture, 1, light);
	meshbuilder_add_quad(&tl, &tr, &bl, &br);
	meshbuilder_add_quad(&tr, &tl, &br, &bl);
}


void (*render_funcs[RENDER_TYPE_COUNT])(int, int, int, block_data) = {
	[RENDER_CUBE] = render_null, // handled elsewhere
	[RENDER_CROSS] = render_cross,
	[RENDER_TORCH] = render_null,
	[RENDER_FIRE] = render_null,
	[RENDER_FLUID] = render_null,
	[RENDER_WIRE] = render_null,
	[RENDER_CROPS] = render_crops,
	[RENDER_DOOR] = render_null,
	[RENDER_LADDER] = render_null,
	[RENDER_RAIL] = render_null,
	[RENDER_STAIRS] = render_null,
	[RENDER_FENCE] = render_null,
	[RENDER_LEVER] = render_null,
	[RENDER_CACTUS] = render_null,
	[RENDER_BED] = render_null,
	[RENDER_REPEATER] = render_null,
	[RENDER_PISTON_BASE] = render_null,
	[RENDER_PISTON_EXTENSION] = render_null,
	[RENDER_CUBE_SPECIAL] = render_null,
	[RENDER_NONE] = render_null
};

void remesh_chunk(world_chunk *chunk)
{
	static const vec3 axes[3] = {{1, 0, 0},
								 {0, 1, 0},
								 {0, 0, 1}};

	world_renderer_update_chunk_visibility(chunk);

	for(int bufidx = 7; bufidx >= 0; bufidx--) {
		struct world_chunk_glbuf *glbuf = &chunk->glbufs[bufidx];

		if(!glbuf->needs_remesh)
			continue;

		glbuf->needs_remesh = false;

		/* free old data */
		mem_free(glbuf->vertices);
		mem_free(glbuf->indices);
		mem_free(glbuf->alpha_vertices);
		mem_free(glbuf->alpha_indices);

		meshbuilder_start(sizeof(*glbuf->vertices));

		for(int xoff = 0; xoff < WORLD_CHUNK_SIZE; xoff++) {
			for(int zoff = 0; zoff < WORLD_CHUNK_SIZE; zoff++) {
				for(int yoff = 0; yoff < WORLD_CHUNK_SIZE; yoff++) {
					int x = (chunk->x << 4) + xoff;
					int y = bufidx * WORLD_CHUNK_SIZE + yoff;
					int z = (chunk->z << 4) + zoff;
					block_data block = world_get_block(x, y, z);

					if(block_is_semi_transparent(block)) {
						continue;
					} else if(block_get_properties(block.id).render_type != RENDER_CUBE) {
						render_funcs[block_get_properties(block.id).render_type](x, y, z, block);
					} else if(!block_is_empty(block)) {
						// greedy meshing here?
						block_properties props = block_get_properties(block.id);

						if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_Y_NEG)) {
							add_block_face(world_make_vertex(
								xoff, yoff, zoff,
								block_get_texture_index(block.id, BLOCK_FACE_Y_NEG, block.metadata),
								BLOCK_FACE_Y_NEG, world_get_block_lighting(x, y-1, z)), BLOCK_FACE_Y_NEG
							);
						}

						if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_Y_POS)) {
							add_block_face(world_make_vertex(
								xoff, yoff, zoff,
								block_get_texture_index(block.id, BLOCK_FACE_Y_POS, block.metadata),
								BLOCK_FACE_Y_POS, world_get_block_lighting(x, y+1, z)), BLOCK_FACE_Y_POS
							);
						}

						if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_X_POS)) {
							add_block_face(world_make_vertex(
								xoff, yoff, zoff,
								block_get_texture_index(block.id, BLOCK_FACE_X_POS, block.metadata),
								BLOCK_FACE_X_POS, world_get_block_lighting(x+1, y, z)), BLOCK_FACE_X_POS
							);
						}

						if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_X_NEG)) {
							add_block_face(world_make_vertex(
								xoff, yoff, zoff,
								block_get_texture_index(block.id, BLOCK_FACE_X_NEG, block.metadata),
								BLOCK_FACE_X_NEG, world_get_block_lighting(x-1, y, z)), BLOCK_FACE_X_NEG
							);
						}

						if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_Z_POS)) {
							add_block_face(world_make_vertex(
								xoff, yoff, zoff,
								block_get_texture_index(block.id, BLOCK_FACE_Z_POS, block.metadata),
								BLOCK_FACE_Z_POS, world_get_block_lighting(x, y, z+1)), BLOCK_FACE_Z_POS
							);
						}

						if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_Z_NEG)) {
							add_block_face(world_make_vertex(
								xoff, yoff, zoff,
								block_get_texture_index(block.id, BLOCK_FACE_Z_NEG, block.metadata),
								BLOCK_FACE_Z_NEG, world_get_block_lighting(x, y, z-1)), BLOCK_FACE_Z_NEG
							);
						}
					}
				}
			}
		}

		meshbuilder_finish(&glbuf->vertices, &glbuf->num_vertices, &glbuf->indices, &glbuf->num_indices);

		glBindBuffer(GL_ARRAY_BUFFER, glbuf->vbo);
		glBufferData(GL_ARRAY_BUFFER,
			/* size */ glbuf->num_vertices * sizeof(*glbuf->vertices),
			/* data */ glbuf->vertices,
			/* usag */ GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);


		/* semi-transparent faces */
		// todo: implement
		// todo: greedy meshing on still water

		/*meshbuilder_start(sizeof(*glbuf->alpha_vertices));

		for(int xoff = 0; xoff < WORLD_CHUNK_SIZE; xoff++) {
			for(int zoff = 0; zoff < WORLD_CHUNK_SIZE; zoff++) {
				for(int yoff = 0; yoff < WORLD_CHUNK_SIZE; yoff++) {
					int x = (chunk->x << 4) + xoff;
					int y = bufidx * WORLD_CHUNK_SIZE + yoff;
					int z = (chunk->z << 4) + zoff;
					block_data block = world_get_block(x, y, z);

					if(!block_is_semi_transparent(block) || block_get_properties(block.id).render_type != RENDER_CUBE) {
						if(block_is_semi_transparent(block)) {
							// it is water
							render_fluid(x, y, z, block);
						}
					} else {
						// greedy meshing here?
						if(!block_is_empty(block)) {
							block_properties props = block_get_properties(block.id);

							if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_Y_NEG)) {
								add_block_face(world_make_vertex(
									xoff, yoff, zoff,
									props.texture_indices[BLOCK_FACE_Y_NEG],
									BLOCK_FACE_Y_NEG, world_get_block_lighting(x, y-1, z)), BLOCK_FACE_Y_NEG
								);
							}

							if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_Y_POS)) {
								add_block_face(world_make_vertex(
									xoff, yoff, zoff,
									props.texture_indices[BLOCK_FACE_Y_POS],
									BLOCK_FACE_Y_POS, world_get_block_lighting(x, y+1, z)), BLOCK_FACE_Y_POS
								);
							}

							if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_X_POS)) {
								add_block_face(world_make_vertex(
									xoff, yoff, zoff,
									props.texture_indices[BLOCK_FACE_X_POS],
									BLOCK_FACE_X_POS, world_get_block_lighting(x+1, y, z)), BLOCK_FACE_X_POS
								);
							}

							if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_X_NEG)) {
								add_block_face(world_make_vertex(
									xoff, yoff, zoff,
									props.texture_indices[BLOCK_FACE_X_NEG],
									BLOCK_FACE_X_NEG, world_get_block_lighting(x-1, y, z)), BLOCK_FACE_X_NEG
								);
							}

							if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_Z_POS)) {
								add_block_face(world_make_vertex(
									xoff, yoff, zoff,
									props.texture_indices[BLOCK_FACE_Z_POS],
									BLOCK_FACE_Z_POS, world_get_block_lighting(x, y, z+1)), BLOCK_FACE_Z_POS
								);
							}

							if(block_should_face_be_rendered(x, y, z, block, BLOCK_FACE_Z_NEG)) {
								add_block_face(world_make_vertex(
									xoff, yoff, zoff,
									props.texture_indices[BLOCK_FACE_Z_NEG],
									BLOCK_FACE_Z_NEG, world_get_block_lighting(x, y, z-1)), BLOCK_FACE_Z_NEG
								);
							}
						}
					}
				}
			}
		}

		meshbuilder_finish(&glbuf->alpha_vertices, &glbuf->num_alpha_vertices, &glbuf->alpha_indices, &glbuf->num_alpha_indices);

		glBindBuffer(GL_ARRAY_BUFFER, glbuf->alpha_vbo);
		glBufferData(GL_ARRAY_BUFFER,*/
//			/* size */ glbuf->num_alpha_vertices * sizeof(*glbuf->alpha_vertices),
//			/* data */ glbuf->alpha_vertices,
//			/* usag */ GL_DYNAMIC_DRAW);
//		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
}

void world_renderer_update_chunk_visibility(world_chunk *chunk)
{
	static const float cube_diagonal_half = SQRT_3 * WORLD_CHUNK_SIZE / 2.0f;

	int x_center = chunk->x * WORLD_CHUNK_SIZE + WORLD_CHUNK_SIZE / 2;
	int z_center = chunk->z * WORLD_CHUNK_SIZE + WORLD_CHUNK_SIZE / 2;

	chunk->visible = false;

	for(int bufidx = 0; bufidx < 8; bufidx++) {
		struct world_chunk_glbuf *glbuf = &chunk->glbufs[bufidx];
		int y_center = bufidx * WORLD_CHUNK_SIZE + WORLD_CHUNK_SIZE / 2;

		glbuf->visible = is_visible_on_frustum((float[]) {x_center, y_center, z_center}, cube_diagonal_half);

		/* if at least 1 glbuf is visible, mark the chunk as visible too */
		if(glbuf->visible) {
			chunk->visible = true;
		}
	}
}

void world_render(void)
{
	size_t i;
	void *it;
	int num_remeshed;

	if(cl.state < cl_connected)
		return;

	if(cl.game.rotated || cl.game.moved)
		recalculate_projection_matrix();

	if(!strcasecmp(gl_polygon_mode.string, "GL_LINE"))
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else if(!strcasecmp(gl_polygon_mode.string, "GL_POINT"))
		glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glPointSize(5.0f);

	glUniformMatrix4fv(loc_view, 1, GL_FALSE, (const GLfloat *) view_mat);
	glUniformMatrix4fv(loc_proj, 1, GL_FALSE, (const GLfloat *) proj_mat);
	glUniform1f(loc_nightlightmod, world_calculate_sky_light_modifier());

	glBindTexture(GL_TEXTURE_2D, gl_world_texture);
	glBindVertexArray(gl_world_vao);

	i = 0;
	num_remeshed = 0;
	while(hashmap_iter(world_chunk_map, &i, &it)) {
		world_chunk *chunk = (world_chunk *) it;

		if(chunk->needs_remesh && num_remeshed < r_max_remeshes.integer) {
			remesh_chunk(chunk);
			chunk->needs_remesh = false;
			num_remeshed++;
		}

		if(chunk->visible) {
			/* go from top to bottom so that the gpu can discard pixels of cave meshes which won't be seen */
			for(int j = 7; j >= 0; j--) {
				vec3 chunk_pos = {chunk->x << 4, j << 4, chunk->z << 4};
				glUniform3fv(loc_chunkpos, 1, chunk_pos);

				if(!chunk->glbufs[j].visible || chunk->glbufs[j].num_vertices <= 0)
					continue;

				glBindVertexBuffer(0, chunk->glbufs[j].vbo, 0, sizeof(struct world_vertex));
				glDrawArrays(GL_TRIANGLES, 0, chunk->glbufs[j].num_vertices);
			}
		}
	}

	i = 0;
	while(hashmap_iter(world_chunk_map, &i, &it)) {
		world_chunk *chunk = (world_chunk *) it;

		if(chunk->visible) {
			/* go from top to bottom so that the gpu can discard pixels of cave meshes which won't be seen */
			for(int j = 7; j >= 0; j--) {
				vec3 chunk_pos = {chunk->x << 4, j << 4, chunk->z << 4};
				glUniform3fv(loc_chunkpos, 1, chunk_pos);

				if(!chunk->glbufs[j].visible || chunk->glbufs[j].num_alpha_vertices <= 0)
					continue;

				glBindVertexBuffer(0, chunk->glbufs[j].alpha_vbo, 0, sizeof(struct world_vertex));
				glDrawArrays(GL_TRIANGLES, 0, chunk->glbufs[j].num_alpha_vertices);
			}
		}
	}

	/* done */
	glBindVertexArray(0);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void world_init_chunk_glbufs(world_chunk *chunk)
{
	for(int i = 0; i < 8; i++) {
		memset(&chunk->glbufs[i], 0, sizeof(chunk->glbufs[i]));
		glGenBuffers(1, &chunk->glbufs[i].vbo);
		glGenBuffers(1, &chunk->glbufs[i].alpha_vbo);
	}
}

void world_free_chunk_glbufs(world_chunk *chunk)
{
	for(int i = 0; i < 8; i++) {
		mem_free(chunk->glbufs[i].vertices);
		mem_free(chunk->glbufs[i].alpha_vertices);
		mem_free(chunk->glbufs[i].indices);
		mem_free(chunk->glbufs[i].alpha_indices);
		glDeleteBuffers(1, &chunk->glbufs[i].vbo);
		glDeleteBuffers(1, &chunk->glbufs[i].alpha_vbo);
		memset(&chunk->glbufs[i], 0, sizeof(chunk->glbufs[i]));
	}
}

struct world_vertex world_make_vertex(float x, float y, float z, ubyte texture_index, ubyte face, ubyte light)
{
	struct world_vertex v = {0};

	v.x = x;
	v.y = y;
	v.z = z;
	v.texture_index = texture_index;
	v.data = face | (light << 3);

	return v;
}
