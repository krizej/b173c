#ifndef B173C_MATHLIB_H
#define B173C_MATHLIB_H

#include "mathlib.h"
#include "common.h"
#include <math.h>

#define SQRT_2 1.4142f
#define SQRT_3 1.7321f
#define PI     3.1415f

#define DEG2RAD(d) ((d) * (PI / 180.0f))
#define RAD2DEG(r) ((r) * (180.0f / PI))

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define bound(low, x, up) max((low), min((x), (up)))

typedef union {
    float array[2];
    struct {
        float x, y;
    };
    struct {
        float u, v;
    };
} vec2;

typedef union {
    float array[3];
    struct {
        float x, y, z;
    };
    struct {
        float r, g, b;
    };
    struct {
        float pitch, yaw, roll;
    };
} vec3;

typedef union {
    float array[4];
    struct {
        float x, y, z, w;
    };
    struct {
        float r, g, b, a;
    };
} vec4;

typedef float mat4[4][4];

typedef struct {
    vec3 mins, maxs;
} bbox;

// clang-format off
#define vec3_from(x, y, z) ((vec3){{x, y, z}})
#define vec3_from1(x)  vec3_from((x), (x), (x))
#define vec3_add(a, b) vec3_from((a).x + (b).x, (a).y + (b).y, (a).z + (b).z)
#define vec3_sub(a, b) vec3_from((a).x - (b).x, (a).y - (b).y, (a).z - (b).z)
#define vec3_mul(v, s) vec3_from((v).x * (s)  , (v).y * (s)  , (v).z * (s)  )
#define vec3_div(v, s) vec3_from((v).x / (s)  , (v).y / (s)  , (v).z / (s)  )
#define vec3_invert(v) vec3_sub(vec3_from1(0), (v))
#define vec3_dot(a, b) ((a).x * (b).x + (a).y * (b).y + (a).z * (b).z)
#define vec3_len(a)    sqrtf(vec3_dot((a), (a)))
// these are not macros because clion could not handle them lololol
vec3 vec3_normalize(vec3 v);
vec3 vec3_cross(vec3 a, vec3 b);
// clang-format on

void mat4_multiply(mat4 dest, mat4 a, mat4 b);
void mat4_identity(mat4 dest);
void mat4_translation(mat4 dest, vec3 translation);
void mat4_rotation(mat4 dest, vec3 rotation);
void mat4_scale(mat4 dest, vec3 scale);
void mat4_view(mat4 dest, vec3 pos, vec3 ang);
void mat4_frustrum(mat4 dest, float l, float r, float b, float t, float n,
                   float f);
void mat4_projection(mat4 dest, float fov, float aspect, float znear,
                     float zfar);

// todo: roll? (and in mat_view as well)
// todo replace with vec3
void cam_angles(vec3 *forward, vec3 *right, vec3 *up, float yaw, float pitch);

#endif
