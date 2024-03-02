#include "common.h"
#include "net/net.h"
#include <SDL2/SDL.h>
#include <stdlib.h>
#include "vid/vid.h"
#include "client/console.h"
#include "input.h"
#include "vid/ui.h"
#include "client/client.h"
#include "assets.h"
#include "client/cvar.h"

#define TPS 20

entity dummy_ent = {0};
struct client_state cl = {.game.our_ent = &dummy_ent};
void *_mem_alloc_impl(size_t sz, const char *file, int line)
{
    void *ptr = calloc(sz, 1);
    if(!ptr) {
        con_printf("%s:%d (%lu): it appears that you've run out of memory."
                   " i will help out by killing myself, meanwhile you go and"
                   " clean up your computer, buy more ram, or, if the number"
                   " in parenthesis is very big, report this error to the"
                   " developer.\n",
                   file, line, sz);
        exit(EXIT_FAILURE);
    }

    return ptr;
}

const char *va(const char *fmt, ...)
{
    static char buf[4096]; // fixme buf
    va_list va;

    va_start(va, fmt);
    vsnprintf(buf, sizeof(buf), fmt, va);
    va_end(va);

    return buf;
}

const char *utf16toutf8(const char16_t *str, size_t len)
{
    static char buf[4096]; // fixme buf
    memset(buf, 0, sizeof(buf));
    for(size_t i = 0; i < len && i < sizeof(buf); i++) {
        buf[i] = (char) str[i];
    }
    return buf;
}

float calc_frametime(void)
{
    static ulong now = 0;
    static ulong then = 0;

    then = now;
    now = SDL_GetPerformanceCounter();

    return ((float)(now - then) / (float)SDL_GetPerformanceFrequency());
}

const char *errmsgs[] = {
    [ERR_OK] = "",
    [ERR_FATAL] = "FATAL ERROR: initialization cannot continue, shutting down",
    [ERR_NETWORK] = "error: network play is not available"
};

#define INIT(func)                                      \
do {                                                    \
    errcode err;                                        \
    err = func();                                       \
    if(err != ERR_OK)                                   \
        con_printf("%s: %s\n", #func, errmsgs[err]);    \
} while(0)

void check_stuck(void)
{
    // server spawns us in blocks sometimes :/
    if(world_is_init()) {
        entity *ent = cl.game.our_ent;
        bbox_t *colliders = world_get_colliding_blocks(cl.game.our_ent->bbox);

        if(bbox_null(*colliders)) {
            // not yet (or at all)
            return;
        }

        while(!bbox_null(*colliders)) {
            entity_set_position(ent, vec3_add(ent->position, vec3(0, 1, 0)));
            colliders = world_get_colliding_blocks(cl.game.our_ent->bbox);
        }

        cl.game.moved = true;
        cl.game.unstuck = true;
    }
}

int main(void)
{
    float phys_timeout = 0.0f;
    
    INIT(cvar_init);
    INIT(con_init);
    cmd_register("unstuck", check_stuck);

    cmd_exec("exec config");
    cmd_exec("exec autoexec");

    INIT(assets_init);
    INIT(in_init);
    INIT(net_init);
    INIT(vid_init);
    INIT(ui_init);
    INIT(entity_renderer_init);
    INIT(world_init);

    while(!cl.done) {
        /* frame time stuff */
        cl.frametime = calc_frametime();
        cl.is_physframe = false;
        phys_timeout -= cl.frametime;
        if(phys_timeout <= 0.0f) {
            cl.is_physframe = true;
            phys_timeout = 1.0f / TPS;
        }

        /* user input */
        in_update();

        /* physics etc. */
        if(cl.is_physframe) {
            net_process();
            entity_update(cl.game.our_ent);
        }

        /* rendering */
        vid_update();
        ui_draw();
        vid_display_frame();

        if(!cl.active) {
            SDL_Delay(25);
        }
    }

    net_shutdown();
    in_shutdown();
    entity_renderer_shutdown();
    ui_shutdown();
    vid_shutdown();

    return 0;
}

