#ifndef B173C_VID_H
#define B173C_VID_H

#include "common.h"
#include "game/world.h"

struct gl_state {
    int w, h;
    uint shader_blocks, shader_text, shader_model;
};

errcode vid_init(void);
void vid_shutdown(void);
void vid_update(void);

void vid_update_viewport(void);
void vid_mouse_grab(bool grab);
void vid_lock_fps(void);
void vid_unlock_fps(void);

void vid_display_frame(void);

#endif
