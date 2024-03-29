#ifndef B173C_MATHLIB_H
#define B173C_MATHLIB_H

#include "mathlib.h"
#include "common.h"
#include <math.h>

#define SQRT_2 1.4142f
#define SQRT_3 1.7321f
#define PI     3.1415f

#define fequ(a, b) (fabsf((a) - (b)) < 0.0001f)

#define deg2rad(d) ((d) * (PI / 180.0f))
#define rad2deg(r) ((r) * (180.0f / PI))

#ifndef max // stupid win32
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef min // stupid win32
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#define bound(low, x, up) max((low), min((x), (up)))

#define sign(f) ((f) < 0 ? -1 : 1)

#define X 0
#define Y 1
#define Z 2

typedef union {
    float array[2];
    struct { float x, y; };
    struct { float u, v; };
} vec2_t;

typedef union {
    float array[3];
    struct { float x, y, z; };
    struct { float r, g, b; };
    struct { float pitch, yaw, roll; };
} vec3_t;

typedef union {
    float array[4];
    struct { float x, y, z, w; };
    struct { float r, g, b, a; };
} vec4_t;

typedef float mat4_t[4][4];

typedef struct {
    vec3_t mins, maxs;
} bbox_t;

#define not0(v) ((v) == 0 ? 0.001f : (v))

#define vec2(x, y) ((vec2_t){{x, y}})
vec2_t vec2_rotate(vec2_t v, float angle, vec2_t origin);

#define vec3_unpack(v) (v).x, (v).y, (v).z
#define vec3(x, y, z) ((vec3_t){{x, y, z}})
#define vec3_1(x)  vec3((x), (x), (x))
#define vec3_add(a, b) vec3((a).x + (b).x, (a).y + (b).y, (a).z + (b).z)
#define vec3_sub(a, b) vec3((a).x - (b).x, (a).y - (b).y, (a).z - (b).z)
#define vec3_mul(v, s) vec3((v).x * (s)  , (v).y * (s)  , (v).z * (s)  )
#define vec3_div(v, s) vec3((v).x / (s)  , (v).y / (s)  , (v).z / (s)  )
#define vec3_invdiv(s, v) vec3((s) / (v).x, (s) / (v).y, (s) / (v).z)
#define vec3_invert(v) vec3_sub(vec3_1(0), (v))
#define vec3_dot(a, b) ((a).x * (b).x + (a).y * (b).y + (a).z * (b).z)
#define vec3_len(a)    sqrtf(vec3_dot((a), (a)))
#define vec3_equ(a, b) (fequ((a).x, (b).x) && fequ((a).y, (b).y) && fequ((a).z, (b).z))
// these are not macros because clion could not handle them lololol
vec3_t vec3_normalize(vec3_t v);
vec3_t vec3_cross(vec3_t a, vec3_t b);

#define vec4(x, y, z, w) ((vec4_t){{x, y, z, w}})

void mat4_multiply(mat4_t dest, mat4_t a, mat4_t b);
vec4_t mat4_multiply_vec4(mat4_t m, vec4_t v);
void mat4_identity(mat4_t dest);
void mat4_translation(mat4_t dest, vec3_t translation);
void mat4_rotation(mat4_t dest, vec3_t rotation);
void mat4_scale(mat4_t dest, vec3_t scale);
void mat4_view(mat4_t dest, vec3_t pos, vec3_t ang);
void mat4_frustrum(mat4_t dest, float l, float r, float b, float t, float n,
                   float f);
void mat4_projection(mat4_t dest, float fov, float aspect, float znear,
                     float zfar);

// todo: roll? (and in mat_view as well)
// todo replace with vec3_t
void cam_angles(vec3_t *forward, vec3_t *right, vec3_t *up, float yaw, float pitch);
vec3_t cam_project_3d_to_2d(vec3_t pos, mat4_t proj, mat4_t modelview, vec2_t vp);

bbox_t bbox_offset(bbox_t bbox, vec3_t offset);
bool bbox_null(bbox_t bbox);
bool bbox_intersects(bbox_t self, bbox_t other);
bool bbox_intersects_line(bbox_t bbox, vec3_t start, vec3_t end, int *face);
bbox_t bbox_join(bbox_t a, bbox_t b);

#endif
