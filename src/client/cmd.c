#include <ctype.h>
#include "console.h"
#include "common.h"
#include "vid/ui.h"
#include "cvar.h"
#include <bsd/string.h>

struct cmd *cmdlist = NULL;
struct alias *aliaslist = NULL;

static char *_cmd_argv[64]; // fixme buf
static int _cmd_argc = 0;

static char *emptystr = "";

// cmds.c
void cmds_register(void);
errcode cmd_init(void)
{
	cmds_register();
    return ERR_OK;
}

char *cmd_argv(int i)
{
	if(i < 0 || i >= _cmd_argc)
		return emptystr;
	return _cmd_argv[i];
}

int cmd_argc(void)
{
	return _cmd_argc;
}

char *cmd_args(int from, int to)
{
	static char buf[4096]; // fixme buf
	int i;

	if(from < 0)
		from = 0;
	if(to > cmd_argc())
		to = cmd_argc();

	memset(buf, 0, sizeof(buf));
	for (i = from; i < to; i++) {
		strlcat(buf, cmd_argv(i), sizeof(buf));
		if(i != to - 1) {
			strlcat(buf, " ", sizeof(buf));
		}
	}

	return buf;
}

void cmd_register(const char *name, void (*func)(void))
{
	struct cmd *c;

	if(!name)
		return;
	if(!func)
		return;

	c = mem_alloc(sizeof(*c));

	c->name = mem_alloc(strlen(name) + 1);
	strcpy(c->name, name);
	c->func = func;

	c->next = cmdlist;
	cmdlist = c;
}

void alias_register(const char *name, const char *command)
{
	struct alias *a;

	if(!name)
		return;
	if(!command)
		return;

	if(alias_find(name))
		alias_remove(name);

	a = mem_alloc(sizeof(*a));

	a->name = mem_alloc(strlen(name) + 1);
	strcpy(a->name, name);

	a->cmd = mem_alloc(strlen(command) + 1);
	strcpy(a->cmd, command);

	a->next = aliaslist;
	aliaslist = a;
}

void alias_remove(const char *name)
{
	struct alias *a = NULL, *prev = NULL;

	if(name == NULL)
		return;

	for(a = aliaslist; a != NULL; prev = a, a = a->next)
		if(strcmp(a->name, name) == 0)
			break;

	if(a == NULL)
		return;

	if(prev != NULL)
		prev->next = a->next;
	else
		aliaslist = a->next;

	mem_free(a->name);
	mem_free(a->cmd);
	mem_free(a);
}

struct cmd *cmd_find(const char *name)
{
	struct cmd *cmd = NULL;

	for(cmd = cmdlist; cmd != NULL; cmd = cmd->next)
		if(strcmp(name, cmd->name) == 0)
			return cmd;

	return NULL;
}

struct alias *alias_find(const char *name)
{
	struct alias *a = NULL;

	for(a = aliaslist; a != NULL; a = a->next)
		if(strcmp(name, a->name) == 0)
			return a;

	return NULL;
}

static char *copystr(char *s)
{
	char *s2;
	size_t len = strlen(s);
	s2 = mem_alloc(len + 1);
	s2[len] = 0;
	memcpy(s2, s, len);
	return s2;
}

static void cmd_cvar(void)
{
	char *name = cmd_argv(0);
	cvar *c = cvar_find(name);

	if(c == NULL) {
		con_printf("cmd_cvar: WTF?!\n");
		return;
	}

	if(cmd_argc() == 1) {
		// display info
		con_printf("%s : "CON_STYLE_PADPX"%ddefault value is \"%s\"\n", name, ui_strwidth("current") - ui_strwidth("default"), c->string);
		con_printf(CON_STYLE_INVISIBLE "%s : " CON_STYLE_WHITE "current value is \"%s\"\n", name, c->string);
	} else {
		// set
		cvar_set(name, cmd_argv(1));
	}
}

// kind of like strtok, but customized
char *tokenize(char *text)
{
	static char *totok;
	static char *p, *start;
	static char oldchar;
	static bool done;

	bool quoted = false;
	bool scanning = false;

	if(text) {
		totok = text;
		p = totok;
		start = p;
		oldchar = 0;
		done = false;
	}

	if(oldchar != 0) {
		*p = oldchar;
		oldchar = 0;
		start = ++p;
		scanning = false;
	}

	while(*p) {
		if(isspace(*p)) {
			if(scanning && !quoted) {
				oldchar = *p;
				*p = 0;
				return start;
			} else {
				p++;
				if(!quoted)
					start = p;
				continue;
			}
		} else if(*p == '"') {
			if(quoted) {
				oldchar = *p;
				*p = 0;
				return start;
			} else {
				if(scanning) {
					oldchar = *p;
					*p = 0;
					return start;
				} else {
					quoted = true;
					start = ++p;
					continue;
				}
			}
		} else {
			scanning = true;
			p++;
		}
	}

	if(done) {
		return NULL;
	} else {
		done = true;
		if(*start == 0)
			return NULL;
		return start;
	}
}

void cmd_exec_impl(char *text);
void cmd_exec(char *text)
{
	bool quoted = false;
	char buf[2048] = {0}; // fixme buf
	int len = 0;

	while(*text) {
		if(*text == '"') {
			quoted = !quoted;
		} else if(*text == ';') {
			if(!quoted) {
				// dont bother executing if its empty
				if(buf[0] != 0)
					cmd_exec_impl(buf);
				memset(buf, 0, sizeof(buf));
				len = 0;
				text++;
				continue;
			}
		}
		buf[len++] = *text++;
	}

	if(buf[0] != 0) {
		cmd_exec_impl(buf);
	}
}

static bool is_color_code(char c)
{
	return isdigit(c) || (c >= 'a' && c <= 'f') || c == 'p' || c == '-' || c == 'i';
}

static void replace_color_codes(char *text)
{
	int i;
	int len = strlen(text);
	for(i = 0; i < len - 1; i++) {
		if(text[i] == '&' && is_color_code(text[i+1])) {
			if(i == 0 || (i > 0 && text[i-1] != '\\')) {
				text[i] = CON_STYLE_PREFIX_CHAR;
			} else if((i > 0 && text[i-1] == '\\')) {
				memmove(&text[i-1], &text[i], len - i);
				text[len - 1] = 0;
			}
		}
	}
}

void cmd_exec_impl(char *text)
{
	char *tok;
	char *totok;
	struct cmd *cmd;
	cvar *cvar;
	struct alias *alias;
	int i;
	char buf[4096] = {0}; // fixme buf
	static int runaway_protection = 0;

// reset old args
	for(i = 0; i < _cmd_argc; i++)
		mem_free(_cmd_argv[i]);
	_cmd_argc = 0;

// make new args
	// new var because strtok modifies the string
	totok = mem_alloc(4096);
	memset(totok, 0, 4096);
	strlcpy(totok, text, 4096);
	replace_color_codes(totok);
	for(tok = tokenize(totok); tok != NULL; tok = tokenize(NULL))
		_cmd_argv[_cmd_argc++] = copystr(tok);
	mem_free(totok);

	if(_cmd_argc == 0)
		return;

// try to find cmd
	cmd = cmd_find(_cmd_argv[0]);
	if(cmd != NULL) {
		cmd->func();
		return;
	}

// not found, so alias
	alias = alias_find(_cmd_argv[0]);
	if(alias != NULL) {
		if(runaway_protection < 1000) {
			if(_cmd_argc > 1) {
				snprintf(buf, sizeof(buf), "%s %s", alias->cmd, cmd_args(1, cmd_argc()));
				cmd_exec(buf);
			} else {
				cmd_exec(alias->cmd);
			}
			runaway_protection++;
		} else {
			con_printf("recursing alias detected - stopping execution\n");
			runaway_protection = 0;
		}
		return;
	}

// not found, so cvar
	cvar = cvar_find(_cmd_argv[0]);
	if(cvar != NULL) {
		cmd_cvar();
		return;
	}

// nothing was found
	con_printf("unknown command \"%s\"\n", _cmd_argv[0]);
}
