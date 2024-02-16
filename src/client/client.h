#ifndef B173C_CLIENT_H
#define B173C_CLIENT_H

#include "common.h"
#include "mathlib.h"
#include "game/world.h"

extern struct client_state {
    struct {
        vec3 rot;
        vec3 pos;
        float stance;
        bool rotated, moved;
        int our_id;
        long seed;
        long time;
        struct trace_result look_trace;
    } game;

    enum { cl_disconnected, cl_connecting, cl_connected } state;

    bool done;   // used in the main loop
    bool active; // whether the window is focused

    ulong fps;
    float frametime;
} cl;

#endif
