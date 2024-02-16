#include "console.h"
#include "common.h"
#include "input.h"
#include "client.h"
#include "vid/ui.h"
#include "cvar.h"
#include <SDL2/SDL.h>
#include <bsd/string.h>

extern cvar *cvarlist;
extern struct cmd *cmdlist;
extern struct alias *aliaslist;

void cmdlist_f(void)
{
	struct cmd *c;
	for(c = cmdlist; c != NULL; c = c->next) {
		con_printf("%s\n", c->name);
	}
}

void cvarlist_f(void)
{
	cvar *c;
	int maxwidth = 0;

	for(c = cvarlist; c != NULL; c = c->next) {
		int w = ui_strwidth(c->name);
		if(w > maxwidth) {
			maxwidth = w;
		}
	}

	for(c = cvarlist; c != NULL; c = c->next) {
		int px = maxwidth - ui_strwidth(c->name);
		con_printf("%s"CON_STYLE_PADPX"%d : \"%s\"\n", c->name, px, c->string);
	}
}

void aliaslist_f(void)
{
	struct alias *a;
	int maxwidth = 0;

	for(a = aliaslist; a != NULL; a = a->next) {
		int w = ui_strwidth(a->name);
		if(w > maxwidth) {
			maxwidth = w;
		}
	}

	for(a = aliaslist; a != NULL; a = a->next) {
		int px = maxwidth - ui_strwidth(a->name);
		con_printf("%s"CON_STYLE_PADPX"%d : \"%s\"\n", a->name, px, a->cmd);
	}
}

void quit_f(void)
{
	cl.done = true;
}

void alias_f(void)
{
	char *name, *cmd;
	struct alias *a;

	if(cmd_argc() >= 3) {
		name = cmd_argv(1);
		cmd = cmd_args(2, cmd_argc());
		alias_register(name, cmd);
	} else if(cmd_argc() == 2) {
		name = cmd_argv(1);
		a = alias_find(name);
		if(a == NULL) {
			con_printf("alias doesn't exist\n");
		} else {
			con_printf("\"%s\" = \"%s\"\n", name, a->cmd);
		}
	} else {
		con_printf("usage: %s <name> [<command>]\n", cmd_argv(0));
	}
}

void unalias_f(void)
{
	if(cmd_argc() != 2) {
		con_printf("usage: %s <name>\n", cmd_argv(0));
		return;
	}

	alias_remove(cmd_argv(1));
}

void echo_f(void)
{
	con_printf("%s\n", cmd_args(1, cmd_argc()));
}

void exec_f(void)
{
	char *line, *data;
	char file[512] = {0}; // fixme buf

	if(cmd_argc() != 2) {
		con_printf("usage: %s <filename>\n", cmd_argv(0));
		return;
	}

	strlcpy(file, cmd_argv(1), sizeof(file));
	strlcat(file, ".cfg", sizeof(file));

	data = (char *) SDL_LoadFile(file, NULL);

	if(data == NULL)
		return;

	for(line = strtok(data, "\n"); line != NULL; line = strtok(NULL, "\n"))
		cmd_exec(line);

	SDL_free(data);
}

void cfg_save_f(void)
{
	FILE *f;
	char *name = "config.cfg";
	const char *keyname;
	struct alias *alias;
	cvar *cvar;
	int i;

	if(cmd_argc() >= 2)
		name = cmd_argv(1);

	f = fopen(name, "w+");
	if(!f) {
		con_printf("failed to open '%s'\n", name);
		return;
	}

	// aliases
	for(alias = aliaslist; alias != NULL; alias = alias->next) {
		fputs("alias \"", f);
		fputs(alias->name, f);
		fputs("\" \"", f);
		fputs(alias->cmd, f);
		fputs("\"\n", f);
	}
	fputs("\n", f);

	// binds
	for(i = 0; i < KEY_NUM; i++) {
		if(input_keys[i].binding) {
			keyname = name_from_key(i);
			if(keyname == NULL) {
				con_printf("cfg_save: null keyname? key=%d\n", i);
				continue;
			}
			fputs("bind \"", f);
			fputs(keyname, f);
			fputs("\" \"", f);
			fputs(input_keys[i].binding, f);
			fputs("\"\n", f);
		}
	}
	fputs("\n", f);

	// cvars
	for(cvar = cvarlist; cvar != NULL; cvar = cvar->next) {
		if(strcmp(cvar->default_value, cvar->string) == 0)
			continue; // default value, dont save

		fputs(cvar->name, f);
		fputs(" \"", f);
		fputs(cvar->string, f);
		fputs("\"\n", f);
	}
	fputs("\n", f);

	fclose(f);

	con_printf("wrote %s\n", name);
}

void cmds_register(void)
{
	cmd_register("cmdlist", cmdlist_f);
	cmd_register("cvarlist", cvarlist_f);
	cmd_register("quit", quit_f);
	cmd_register("echo", echo_f);
	cmd_register("exec", exec_f);
	cmd_register("cfg_save", cfg_save_f);

	cmd_register("alias", alias_f);
	cmd_register("unalias", unalias_f);
	cmd_register("aliaslist", aliaslist_f);
}


