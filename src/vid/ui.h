#ifndef B173C_UI_H
#define B173C_UI_H

#include "common.h"

// _CON_CHAR_SIZE + 1 (see ui.c)
#define LINE_HEIGHT_PX 9

extern int ui_w, ui_h;

errcode ui_init(void);
void ui_shutdown(void);

void ui_draw(void);   // called before r_display_frame
void ui_render(void); // called in r_display_frame

bool ui_drawchar(float x, float y, ubyte character, int color);
void ui_drawtext(float x, float y, char *text);
void ui_printf(float x, float y, const char *fmt, ...);
int ui_charwidth(ubyte c);
int ui_strwidth(const char *text);

// console.c
void ui_draw_console(void);

#endif
