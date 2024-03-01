#include <glad/glad.h>
#include <stdarg.h>
#include <ctype.h>
#include "ui.h"
#include "common.h"
#include "client/console.h"
#include "vid.h"
#include "client/client.h"
#include "assets.h"
#include "client/cvar.h"
#include "../client/client.h"

// arbitrary limit, can be increased safely
#define MAX_CON_CHARS 8192

#define _CON_CHAR_SIZE 8
#define CON_CHAR_SPACE_WIDTH 6

int CON_CHAR_SIZE = _CON_CHAR_SIZE;

int ui_w, ui_h;

vec2_t glyph_vertices_original[] = {
    {.x=0.0f, .y=0.0f}, {.u=0.0f, .v=0.0f},
    {.x=1.0f, .y=0.0f}, {.u=1.0f, .v=0.0f},
    {.x=0.0f, .y=1.0f}, {.u=0.0f, .v=1.0f},
    {.x=1.0f, .y=1.0f}, {.u=1.0f, .v=1.0f},
};

vec2_t glyph_vertices[] = {
    {.x=0.0f, .y=0.0f}, {.u=0.0f, .v=1.0f},
    {.x=1.0f, .y=0.0f}, {.u=1.0f, .v=1.0f},
    {.x=0.0f, .y=1.0f}, {.u=0.0f, .v=0.0f},
    {.x=1.0f, .y=1.0f}, {.u=1.0f, .v=0.0f},
};

float char_widths[256] = {0};
vec4_t text_data[MAX_CON_CHARS] = {0};
int text_char_count = 0;
uint gl_ui_font_texture, gl_uniform_con_char_size;
uint gl_text_vao, gl_text_glyph_vbo = 0, gl_text_data_buffer;
asset_image *asset_font_image = NULL;

extern struct gl_state gl;

void ui_update_size(int x, int y)
{
    int i;

    ui_w = (int)((float)(x) / ui_scale.value);
    ui_h = (int)((float)(y) / ui_scale.value);

    for(i = 0; i < 8; i += 2) {
        // _CON_CHAR_SIZE must be used here
        glyph_vertices[i].x = glyph_vertices_original[i].x * _CON_CHAR_SIZE / (float)(ui_w);
        glyph_vertices[i].y = glyph_vertices_original[i].y * _CON_CHAR_SIZE / (float)(ui_h);
        glyph_vertices[i].y -= _CON_CHAR_SIZE / (float)(ui_h); // move origin of the font polygon to the top left corner
    }

    // this function could possibly be called before the buffer was created
    if(gl_text_glyph_vbo) {
        glBindBuffer(GL_ARRAY_BUFFER, gl_text_glyph_vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glyph_vertices), glyph_vertices, GL_STATIC_DRAW);
    }
}

void onchange_ui_scale(void)
{
    ui_update_size(vid_width.integer, vid_height.integer);
}

static void calculate_char_widths(void)
{
    int i;
    for(i = 0; i < 256; i++) {
        ubyte c = i & 255;
        int x, x2, y, y2, alpha_idx, xs, xe;
        bool start_set = false;

        if(c == ' ') {
            char_widths[c] = CON_CHAR_SPACE_WIDTH;
            continue;
        }

        xe = 1;
        xs = 0;

        x = (c % 16) * CON_CHAR_SIZE;
        y = (c / 16) * CON_CHAR_SIZE;
        for(x2 = x; x2 < x + CON_CHAR_SIZE; x2++) {
            bool empty = true;
            for(y2 = y; y2 < y + CON_CHAR_SIZE; y2++) {
                alpha_idx = (y2 * asset_font_image->width + x2) * asset_font_image->channels + 3;
                if(asset_font_image->data[alpha_idx] > 0) {
                    empty = false;
                    break;
                }
            }
            if(!start_set) {
                xs = x2 - x;
                if(!empty) {
                    start_set = true;
                }
            } else {
                if(!empty) {
                    xe = x2 - x + 1;
                }
            }
        }
        char_widths[c] = xe - xs + 1;
        if(char_widths[c] < 0) {
            char_widths[c] = 0;
        }
        // proper scaling
        char_widths[c] /= CON_CHAR_SIZE / _CON_CHAR_SIZE;
    }
}

errcode ui_init(void)
{
    asset_font_image = asset_get_image(ASSET_TEXTURE_FONT_DEFAULT);
    CON_CHAR_SIZE = _CON_CHAR_SIZE * asset_font_image->width / 128;

    // setup font texture
    glGenTextures(1, &gl_ui_font_texture);
    glBindTexture(GL_TEXTURE_2D, gl_ui_font_texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, asset_font_image->width, asset_font_image->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, asset_font_image->data);
    glBindTexture(GL_TEXTURE_2D, 0);

    // setup font buffers
    glGenVertexArrays(1, &gl_text_vao);
    glGenBuffers(1, &gl_text_glyph_vbo);
    glGenBuffers(1, &gl_text_data_buffer);
    glBindVertexArray(gl_text_vao);
        glBindBuffer(GL_ARRAY_BUFFER, gl_text_glyph_vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glyph_vertices), glyph_vertices, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(vec2_t), (void *) 0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(vec2_t), (void *) sizeof(vec2_t));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);

        // instance data
        glBindBuffer(GL_ARRAY_BUFFER, gl_text_data_buffer);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(vec4_t), (void *) 0);
        glEnableVertexAttribArray(2);
        glVertexAttribDivisor(2, 1);
    glBindVertexArray(0);

    gl_uniform_con_char_size = glGetUniformLocation(gl.shader_text, "CON_CHAR_SIZE");

    ui_update_size(vid_width.integer, vid_height.integer);

    calculate_char_widths();

    if(glGetError() != GL_NO_ERROR)
        return ERR_FATAL;
    return ERR_OK;
}

void ui_shutdown(void)
{

}

void ui_draw(void)
{
    ui_draw_console();
}

void ui_render(void)
{
    if(developer.integer) {
        static float maxy;
        float x = 1;
        float y = -8 + 1;
        if(cl.game.our_ent->position.y > maxy)
            maxy = cl.game.our_ent->position.y;
        ui_printf(x, y+=8, "b173c 0.0.0 (%lu fps)", cl.fps);
        ui_printf(x, y+=48, "x: %.14f (%d)", cl.game.our_ent->position.x, (int)cl.game.our_ent->position.x >> 4);
        ui_printf(x, y+=8, "y: %.14f (%f)", cl.game.our_ent->position.y, maxy);
        ui_printf(x, y+=8, "z: %.14f (%d)", cl.game.our_ent->position.z, (int)cl.game.our_ent->position.z >> 4);
        ui_printf(x, y+=8, "state: %d/%d", cl.game.our_ent->onground, cl.game.our_ent->was_onground);
        ui_printf(x, y+=24, "Seed: %ld", cl.game.seed);
        ui_printf(x, y+=8, "Time: %lu (day %lu)", cl.game.time, cl.game.time / 24000);
    }

    // crosshair
    // assume the char is a square :P
    // todo: crosshair cvar? later
    ui_printf((float)ui_w / 2.0f - (float)ui_charwidth('\x9') / 2.0f,
              (float)ui_h / 2.0f - (float)ui_charwidth('\x9') / 2.0f,
              "%c", '\x9');

    glBindBuffer(GL_ARRAY_BUFFER, gl_text_data_buffer);
    // todo: update only parts of the buffer that were changed?
    glBufferData(GL_ARRAY_BUFFER, text_char_count * sizeof(vec4_t), text_data, GL_STREAM_DRAW);

    glUniform1i(gl_uniform_con_char_size, CON_CHAR_SIZE);

    glBindVertexArray(gl_text_vao);
    glBindTexture(GL_TEXTURE_2D, gl_ui_font_texture);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, text_char_count);

    text_char_count = 0;
}

int ui_charwidth(ubyte c)
{
    return char_widths[c];
}

int ui_strwidth(const char *text)
{
    int w = 0;
    if(!text)
        return 0;
    while(*text)
        w += ui_charwidth(*text++);
    return w;
}

bool ui_drawchar(float x, float y, ubyte character, int color)
{
    if(color > 0) // draw shadow
        ui_drawchar(x + 1, y + 1, character, -color);

    if(text_char_count > MAX_CON_CHARS - 100) {
        con_printf(CON_STYLE_RED"font count is @ %d!!!! not drawing any text anymore\n", text_char_count);
        return false;
    }

    if(x <= -ui_charwidth(character) || x >= ui_w) // outside
        return false;
    if(y <= -LINE_HEIGHT_PX || y >= ui_h) // outside
        return false;

    text_data[text_char_count].x = x / ((float)ui_w);
    text_data[text_char_count].y = y / ((float)ui_h);
    text_data[text_char_count].z = (float)character;
    text_data[text_char_count].w = (float)color;
    text_char_count++;

    return true;
}

void ui_drawtext(float x, float y, char *text)
{
    byte color = 0xf;
    bool read_color_code = false;
    bool invisible = false;

    if(y == -LINE_HEIGHT_PX || y >= ui_h)
        return; // cant be seen anyway   fixme: what if there is a newline in the text

    // skip characters outside the screen
    while(x < -ui_charwidth(*text) && *text) {
        x += ui_charwidth(*text);
        text++;
    }

    while(*text) {
        if(read_color_code) {
            switch(*text) {
                case 'i': {
                    invisible = true;
                    read_color_code = false;
                    text++;
                    continue;
                } break;
                case 'p': {
                    read_color_code = false;
                    text++;
                    x += strtol(text, &text, 10);
                    continue;
                } break;
            }

            if(isdigit(*text))
                color = *text - '0'; // convert to decimal
            else if(*text >= 'a' && *text <= 'f')
                color = *text - 'a' + 0xa; // convert to hex
            else
                color = 0x0; // black

            if(color < 0x0 || color > 0xf)
                color = 0xf;

            invisible = false;
            read_color_code = false;
            text++;
            continue;
        } else if(*text == CON_STYLE_PREFIX_CHAR) {
            read_color_code = true;
            text++;
            continue;
        }

        if(!invisible) {
            if(*text != '\n') { // do not draw newlines
                if(!ui_drawchar(x, y, *text, color)) {
                    break;
                }
            }
        }

        x += ui_charwidth(*text++);
    }
}

void ui_printf(float x, float y, const char *fmt, ...)
{
    va_list va;
    char buf[4096]; // fixme buf

    va_start(va, fmt);
    vsnprintf(buf, sizeof(buf), fmt, va);
    va_end(va);

    ui_drawtext(x, y, buf);
}
