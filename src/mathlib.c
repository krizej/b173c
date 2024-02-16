#include "mathlib.h"
#include "common.h"
#include <math.h>

vec3 vec3_normalize(vec3 v)
{
	return vec3_div(v, vec3_len(v));
}

vec3 vec3_cross(vec3 a, vec3 b)
{
	vec3 s;

	s.x = a.y * b.z - a.z * b.y;
	s.y = a.z * b.x - a.x * b.z;
	s.z = a.x * b.y - a.y * b.x;

	return vec3_normalize(s);
}

void mat4_multiply(mat4 dest, mat4 a, mat4 b) {
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

void mat4_identity(mat4 dest)
{
	memset(dest, 0, 16 * sizeof(float));
	for(int i = 0; i < 4; i++) {
		dest[i][i] = 1;
	}
}

void mat4_translation(mat4 dest, vec3 translation)
{
	dest[3][0] = translation.x;
	dest[3][1] = translation.y;
	dest[3][2] = translation.z;
}

void mat4_rotation(mat4 dest, vec3 rotation)
{
	// https://en.wikipedia.org/wiki/Rotation_matrix#General_3D_rotations
	// THX

	float sina = sin(DEG2RAD(rotation.pitch));
	float cosa = cos(DEG2RAD(rotation.pitch));
	float sinb = sin(DEG2RAD(rotation.yaw));
	float cosb = cos(DEG2RAD(rotation.yaw));
	float siny = sin(DEG2RAD(rotation.roll));
	float cosy = cos(DEG2RAD(rotation.roll));

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

void mat4_scale(mat4 dest, vec3 scale)
{
  dest[0][0] = scale.x;
  dest[1][1] = scale.y;
  dest[2][2] = scale.z;
}

void cam_angles(vec3 *fwd, vec3 *side, vec3 *up, float yaw, float pitch)
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

void mat4_view(mat4 dest, vec3 pos, vec3 ang)
{
	vec3 x, y, z;

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

void mat4_frustrum(mat4 dest, float l, float r, float b, float t, float n, float f)
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

void mat4_projection(mat4 dest, float fov, float aspect, float znear, float zfar)
{
	float ymax, xmax;
	ymax = znear * tanf(DEG2RAD(fov) * 0.5f);
	xmax = ymax * aspect;
	mat4_frustrum(dest, -xmax, xmax, -ymax, ymax, znear, zfar);
}
