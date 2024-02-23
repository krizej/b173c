#ifndef B173C_CONSOLE_H
#define B173C_CONSOLE_H

#include "common.h"

#define CON_STYLE_PREFIX_STR  "\xa7"
#define CON_STYLE_PREFIX_CHAR '\xa7'

#define CON_STYLE_BLACK      CON_STYLE_PREFIX_STR "0"
#define CON_STYLE_DARK_BLUE  CON_STYLE_PREFIX_STR "1"
#define CON_STYLE_DARK_GREEN CON_STYLE_PREFIX_STR "2"
#define CON_STYLE_DARK_CYAN  CON_STYLE_PREFIX_STR "3"
#define CON_STYLE_DARK_RED   CON_STYLE_PREFIX_STR "4"
#define CON_STYLE_PURPLE     CON_STYLE_PREFIX_STR "6"
#define CON_STYLE_ORANGE     CON_STYLE_PREFIX_STR "7"
#define CON_STYLE_LIGHT_GRAY CON_STYLE_PREFIX_STR "7"
#define CON_STYLE_GRAY       CON_STYLE_PREFIX_STR "8"
#define CON_STYLE_BLUE       CON_STYLE_PREFIX_STR "9"
#define CON_STYLE_LIME       CON_STYLE_PREFIX_STR "a"
#define CON_STYLE_CYAN       CON_STYLE_PREFIX_STR "b"
#define CON_STYLE_RED        CON_STYLE_PREFIX_STR "c"
#define CON_STYLE_PINK       CON_STYLE_PREFIX_STR "d"
#define CON_STYLE_YELLOW     CON_STYLE_PREFIX_STR "e"
#define CON_STYLE_WHITE      CON_STYLE_PREFIX_STR "f"
#define CON_STYLE_INVISIBLE  CON_STYLE_PREFIX_STR "i"
#define CON_STYLE_PADPX      CON_STYLE_PREFIX_STR "p"

struct cmd {
    char *name;
    void (*func)(void);

    struct cmd *next;
};

struct alias {
    char *name;
    char *cmd;

    struct alias *next;
};

extern bool con_opened;
extern int con_scroll;

errcode con_init(void);
bool con_handle_key(int key, int keymod);

void con_show(void);
void con_hide(void);

void con_printf(char *text, ...);

// cvar/cmd related

errcode cmd_init(void);
void cmd_exec(char *text);

void cmd_register(const char *name, void (*func)(void));
void alias_register(const char *name, const char *command);

void alias_remove(const char *name);

struct cmd *cmd_find(const char *name);
struct alias *alias_find(const char *name);


char *cmd_argv(int i);
int cmd_argc(void);
char *cmd_args(int from, int to);

#endif
