#ifndef B173C_CVAR_H
#define B173C_CVAR_H

#include "common.h"

typedef struct cvar {
    char *name;
    char *default_value;
    void (*onchange)(void);

    char *string;
    float value;
    int integer;

    struct cvar *next;
} cvar;

errcode cvar_init(void);

void cvar_register(cvar *c);
cvar *cvar_find(const char *name);
bool cvar_set(const char *name, const char *value);
// silent set doesn't call onchange functions
bool cvar_set_silent(const char *name, const char *value);

extern cvar fov;
extern cvar developer;
extern cvar cvar_name;
extern cvar sensitivity;

extern cvar r_zfar;
extern cvar r_znear;
extern cvar r_max_remeshes;
extern cvar r_fancyleaves;
extern cvar r_fancygrass;
extern cvar r_smartleaves;
extern cvar r_redstone_dot;

extern cvar gl_polygon_mode;

extern cvar vid_width;
extern cvar vid_height;

extern cvar cl_2b2tmode;
extern cvar cl_smoothstep;
extern cvar cl_freecamera;

extern cvar ui_scale;

#endif
