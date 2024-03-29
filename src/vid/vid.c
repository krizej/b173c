#include <SDL2/SDL.h>
#include <glad/glad.h>
#include "SDL_video.h"
#include "common.h"
#include "mathlib.h"
#include "vid.h"
#include "shaders.h"
#include "client/console.h"
#include "ui.h"
#include "client/client.h"
#include "game/world.h"
#include <SDL2/SDL_vulkan.h>
#include "client/cvar.h"

SDL_Window *window_handle;
SDL_GLContext glcontext;

// for fps because its a little more stable and looks nicer :)
uint64_t frames_drawn = 0;
uint64_t last_check_tick = 0;

struct gl_state gl;

#define check_shader_compile(shader) check_shader_compile_impl(shader, #shader)

static bool check_shader_compile_impl(uint32_t h, const char *name)
{
    int ok = true, loglen;
    char *log;

    glGetShaderiv(h, GL_COMPILE_STATUS, &ok);
    if(!ok) {
        glGetShaderiv(h, GL_INFO_LOG_LENGTH, &loglen);
        log = mem_alloc(loglen + 1);
        memset(log, 0, loglen + 1);
        glGetShaderInfoLog(h, loglen, &loglen, log);
        con_printf("shader '%s' failed to compile: %s\n", name, log);
        glDeleteShader(h);
        mem_free(log);
    }

    return ok;
}

static uint32_t check_program_compile(uint32_t h, uint32_t h_vs, uint32_t h_fs)
{
    int ok = true, loglen;
    char *log;

    glGetProgramiv(h, GL_LINK_STATUS, &ok);
    if(!ok) {
        glGetProgramiv(h, GL_INFO_LOG_LENGTH, &loglen);
        log = mem_alloc(loglen + 1);
        memset(log, 0, loglen + 1);
        glGetProgramInfoLog(h, loglen, &loglen, log);
        con_printf("shader program failed to compile:\n%s\n", log);
        mem_free(log);

        glDeleteProgram(h);
        glDeleteShader(h_vs);
        glDeleteShader(h_fs);

        return 0;
    }

    glDetachShader(h, h_vs);
    glDetachShader(h, h_fs);

    glDeleteShader(h_vs);
    glDeleteShader(h_fs);

    return h;
}

static uint32_t load_shader(const char *vs, const char *fs)
{
    uint32_t h_vs, h_fs, h_prog;

    // vertex shader
    h_vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(h_vs, 1, &vs, NULL);
    glCompileShader(h_vs);
    if(!check_shader_compile(h_vs))
        return 0;

    // fragment shader
    h_fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(h_fs, 1, &fs, NULL);
    glCompileShader(h_fs);
    if(!check_shader_compile(h_fs))
        return 0;

    h_prog = glCreateProgram();
    glAttachShader(h_prog, h_vs);
    glAttachShader(h_prog, h_fs);
    glLinkProgram(h_prog);
    return check_program_compile(h_prog, h_vs, h_fs);
}

// TODO: make this nicer
void gl_debug_message(GLenum source attr(unused), GLenum type,
                      GLuint id attr(unused), GLenum severity,
                      GLsizei length attr(unused), const GLchar *message,
                      const void *userParam attr(unused))
{
    fprintf(stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
            (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""),
            type, severity, message);
    if(type == GL_DEBUG_TYPE_ERROR) {
        exit(1);
    }
}

errcode vid_init(void)
{
    int flags, ok;
    int x = SDL_WINDOWPOS_CENTERED, y = SDL_WINDOWPOS_CENTERED;

    /* ----- SDL INIT ----- */
    ok = SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    if(ok < 0) {
        con_printf("SDL initialization failed: %s\n", SDL_GetError());
        goto err_sdl;
    }
    
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
    
    flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL | SDL_WINDOW_INPUT_FOCUS;
    window_handle = SDL_CreateWindow("b173c", x, y, vid_width.integer, vid_height.integer, flags);

    if(!window_handle) {
        con_printf("Failed to create a window: %s\n", SDL_GetError());
        goto err_window;
    }

    /* ----- OPENGL INIT ----- */
    glcontext = SDL_GL_CreateContext(window_handle);
    if(!glcontext) {
        con_printf("Failed to create OpenGL context: %s\n", SDL_GetError());
        goto err_glctx;
    }

    ok = gladLoadGLLoader((GLADloadproc) SDL_GL_GetProcAddress);
    if(!ok) {
        con_printf("Failed to initialize GLAD\n");
        goto err_glad;
    }

    /* enable debug output */
    // glEnable(GL_DEBUG_OUTPUT);
    // glDebugMessageCallback(gl_debug_message, 0);

    /* load shaders */
    gl.shader_blocks_simple = load_shader(simpleblocks_v_glsl, simpleblocks_f_glsl);
    gl.shader_blocks_complex = load_shader(complexblocks_v_glsl, complexblocks_f_glsl);
    gl.shader_model = load_shader(model_v_glsl, model_f_glsl);
    gl.shader_text = load_shader(text_v_glsl, text_f_glsl);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    world_renderer_init();

    return ERR_OK;

  err_glad:
    SDL_GL_DeleteContext(glcontext);
  err_glctx:
    SDL_DestroyWindow(window_handle);
  err_window:
    SDL_Quit();
  err_sdl:
    return ERR_FATAL;
}

void vid_lock_fps(void)
{
    SDL_GL_SetSwapInterval(-1);
}

void vid_unlock_fps(void)
{
    SDL_GL_SetSwapInterval(0);
}

void vid_shutdown(void)
{
    world_renderer_shutdown();

    glDeleteProgram(gl.shader_blocks_complex);
    glDeleteProgram(gl.shader_blocks_simple);
    glDeleteProgram(gl.shader_text);
    glDeleteProgram(gl.shader_model);
    
    SDL_GL_DeleteContext(glcontext);
    SDL_DestroyWindow(window_handle);
    SDL_Quit();
}

void vid_update_viewport(void)
{
    SDL_GetWindowSize(window_handle, &gl.w, &gl.h);
    glViewport(0, 0, gl.w * (16 / 9), gl.h);

    /* update cvars */
    cvar_set("vid_width", va("%d", gl.w));
    cvar_set("vid_height", va("%d", gl.h));

    /* hack alert: force recalculation of projection matrix and ui scaling */
    cvar_find("ui_scale")->onchange();
    cvar_find("fov")->onchange();
}

void vid_mouse_grab(bool grab)
{
    SDL_SetWindowGrab(window_handle, grab);
    SDL_SetRelativeMouseMode(grab);
}

void vid_update(void)
{
    if(SDL_GetRelativeMouseMode()) {
        SDL_WarpMouseInWindow(window_handle, gl.w / 2, gl.h / 2);
    }
}

void vid_display_frame(void)
{
    vec4_t clearcolor = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    if(world_is_init())
        clearcolor = world_calculate_sky_color();
    glClearColor(clearcolor.r, clearcolor.g, clearcolor.b, clearcolor.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* draw 3d stuff */
    world_render();
    glUseProgram(gl.shader_model);
    entity_renderer_render();

    /* draw 2d stuff */
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glUseProgram(gl.shader_text);
    ui_render();

    /* ! */
    SDL_GL_SwapWindow(window_handle);

    /* update fps counter */
    frames_drawn++;
    if(SDL_GetTicks64() - last_check_tick > 1000) {
        cl.fps = frames_drawn;
        last_check_tick = SDL_GetTicks64();
        frames_drawn = 0;
    }
}
