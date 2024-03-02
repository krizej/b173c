#include "mathlib.h"
#include "common.h"
#include <math.h>

vec3_t vec3_normalize(vec3_t v)
{
    return vec3_div(v, vec3_len(v));
}

vec3_t vec3_cross(vec3_t a, vec3_t b)
{
    vec3_t s;

    s.x = a.y * b.z - a.z * b.y;
    s.y = a.z * b.x - a.x * b.z;
    s.z = a.x * b.y - a.y * b.x;

    return vec3_normalize(s);
}

void mat4_multiply(mat4_t dest, mat4_t a, mat4_t b)
{
    dest[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0] + a[0][3] * b[3][0];
    dest[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1] + a[0][3] * b[3][1];
    dest[0][2] = a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2] + a[0][3] * b[3][2];
    dest[0][3] = a[0][0] * b[0][3] + a[0][1] * b[1][3] + a[0][2] * b[2][3] + a[0][3] * b[3][3];

    dest[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0] + a[1][3] * b[3][0];
    dest[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1] + a[1][3] * b[3][1];
    dest[1][2] = a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2] + a[1][3] * b[3][2];
    dest[1][3] = a[1][0] * b[0][3] + a[1][1] * b[1][3] + a[1][2] * b[2][3] + a[1][3] * b[3][3];

    dest[2][0] = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0] + a[2][3] * b[3][0];
    dest[2][1] = a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1] + a[2][3] * b[3][1];
    dest[2][2] = a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2] + a[2][3] * b[3][2];
    dest[2][3] = a[2][0] * b[0][3] + a[2][1] * b[1][3] + a[2][2] * b[2][3] + a[2][3] * b[3][3];

    dest[3][0] = a[3][0] * b[0][0] + a[3][1] * b[1][0] + a[3][2] * b[2][0] + a[3][3] * b[3][0];
    dest[3][1] = a[3][0] * b[0][1] + a[3][1] * b[1][1] + a[3][2] * b[2][1] + a[3][3] * b[3][1];
    dest[3][2] = a[3][0] * b[0][2] + a[3][1] * b[1][2] + a[3][2] * b[2][2] + a[3][3] * b[3][2];
    dest[3][3] = a[3][0] * b[0][3] + a[3][1] * b[1][3] + a[3][2] * b[2][3] + a[3][3] * b[3][3];
}

void mat4_identity(mat4_t dest)
{
    memset(dest, 0, 16 * sizeof(float));
    for(int i = 0; i < 4; i++) {
        dest[i][i] = 1;
    }
}

void mat4_translation(mat4_t dest, vec3_t translation)
{
    dest[3][0] = translation.x;
    dest[3][1] = translation.y;
    dest[3][2] = translation.z;
}

void mat4_rotation(mat4_t dest, vec3_t rotation)
{
    // https://en.wikipedia.org/wiki/Rotation_matrix#General_3D_rotations
    // THX

    float sina = sinf(DEG2RAD(rotation.pitch));
    float cosa = cosf(DEG2RAD(rotation.pitch));
    float sinb = sinf(DEG2RAD(rotation.yaw));
    float cosb = cosf(DEG2RAD(rotation.yaw));
    float siny = sinf(DEG2RAD(rotation.roll));
    float cosy = cosf(DEG2RAD(rotation.roll));

    dest[0][0] = cosb * cosy;
    dest[0][1] = cosb * siny;
    dest[0][2] = -sinb;

    dest[1][0] = sina * sinb * cosy - cosa * siny;
    dest[1][1] = sina * sinb * siny + cosa * cosy;
    dest[1][2] = sina * cosb;

    dest[2][0] = cosa * sinb * cosy + sina * siny;
    dest[2][1] = cosa * sinb * siny - sina * cosy;
    dest[2][2] = cosa * cosb;
}

void mat4_scale(mat4_t dest, vec3_t scale)
{
    dest[0][0] = scale.x;
    dest[1][1] = scale.y;
    dest[2][2] = scale.z;
}

void cam_angles(vec3_t *fwd, vec3_t *side, vec3_t *up, float yaw, float pitch)
{
    float cp, sp, cy, sy;
    cp = cosf(DEG2RAD(pitch));
    sp = sinf(DEG2RAD(pitch));
    cy = cosf(DEG2RAD(yaw));
    sy = sinf(DEG2RAD(yaw));

    if(side != NULL) {
        side->x = -cy;
        side->y = 0;
        side->z = sy;
    }

    if(up != NULL) {
        up->x = sy * sp;
        up->y = cp;
        up->z = cy * sp;
    }

    if(fwd != NULL) {
        fwd->x = sy * cp;
        fwd->y = -sp;
        fwd->z = cp * cy;
    }
}

void mat4_view(mat4_t dest, vec3_t pos, vec3_t ang)
{
    vec3_t x, y, z;

    cam_angles(&z, &x, &y, ang.yaw, ang.pitch);
    z = vec3_invert(z);

    for(int i = 0; i < 3; i++) {
        dest[i][0] = x.array[i];
        dest[i][1] = y.array[i];
        dest[i][2] = z.array[i];
        dest[i][3] = 0.0f;
    }

    dest[3][0] = -vec3_dot(x, pos);
    dest[3][1] = -vec3_dot(y, pos);
    dest[3][2] = -vec3_dot(z, pos);
    dest[3][3] = 1.0f;
}

void mat4_frustrum(mat4_t dest, float l, float r, float b, float t, float n, float f)
{
    float z2, dw, dh, dz;

    memset(dest, 0, 16 * sizeof(float));

    z2 = 2.0f * n;
    dw = r - l;
    dh = t - b;
    dz = f - n;
    dest[0][0] = z2 / dw;
    dest[1][1] = z2 / dh;
    dest[2][0] = (r + l) / dw;
    dest[2][1] = (t + b) / dh;
    dest[2][2] = (-f + n) / dz;
    dest[2][3] = -1.0f;
    dest[3][2] = (-z2 * f) / dz;
}

void mat4_projection(mat4_t dest, float fov, float aspect, float znear, float zfar)
{
    float ymax, xmax;
    ymax = znear * tanf(DEG2RAD(fov) * 0.5f);
    xmax = ymax * aspect;
    mat4_frustrum(dest, -xmax, xmax, -ymax, ymax, znear, zfar);
}

bbox_t bbox_offset(bbox_t bbox, vec3_t offset)
{
    bbox_t ret = bbox;
    ret.mins = vec3_add(ret.mins, offset);
    ret.maxs = vec3_add(ret.maxs, offset);
    return ret;
}

bool bbox_null(bbox_t bbox)
{
    return bbox.mins.x == -1 && bbox.mins.y == -1 && bbox.mins.z == -1 &&
           bbox.maxs.x == -1 && bbox.maxs.y == -1 && bbox.maxs.z == -1;
}

bool bbox_intersects_line(bbox_t bbox, vec3_t start, vec3_t end)
{
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
    // THX

    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    vec3_t bounds[] = {bbox.mins, bbox.maxs};
    vec3_t invdir = vec3_invdiv(1, vec3_normalize(vec3_sub(end, start)));
    int sign[3] = {(invdir.x < 0), (invdir.y < 0), (invdir.z < 0)};

    tmin = (bounds[sign[0]].x - start.x) * invdir.x;
    tmax = (bounds[1 - sign[0]].x - start.x) * invdir.x;
    tymin = (bounds[sign[1]].y - start.y) * invdir.y;
    tymax = (bounds[1 - sign[1]].y - start.y) * invdir.y;

    if((tmin > tymax) || (tymin > tmax))
        return false;

    if(tymin > tmin)
        tmin = tymin;
    if(tymax < tmax)
        tmax = tymax;

    tzmin = (bounds[sign[2]].z - start.z) * invdir.z;
    tzmax = (bounds[1 - sign[2]].z - start.z) * invdir.z;

    if((tmin > tzmax) || (tzmin > tmax))
        return false;

    return true;
}
