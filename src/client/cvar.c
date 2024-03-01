#include "cvar.h"

cvar *cvarlist = NULL;

void cvar_register(cvar *c)
{
	if(c->name == NULL)
		return;
	if(c->default_value == NULL)
		return;

	c->next = cvarlist;
	cvarlist = c;
	c->string = NULL;

	cvar_set(c->name, c->default_value);
}

cvar *cvar_find(const char *name)
{
	cvar *cvar;

	for(cvar = cvarlist; cvar != NULL; cvar = cvar->next)
		if(strcmp(name, cvar->name) == 0)
			return cvar;

	return NULL;
}

bool cvar_set_silent(const char *name, const char *value)
{
	cvar *cvar;
	size_t len;

	if (name == NULL)
		return false;
	if (value == NULL)
		return false;
	if (!(cvar = cvar_find(name)))
		return false;

	len = strlen(value);

	mem_free(cvar->string);
	cvar->string = mem_alloc(len + 1);
	memcpy(cvar->string, value, len);
	cvar->string[len] = 0;

	cvar->value = strtof(value, NULL);
	cvar->integer = (int) strtol(value, NULL, 10);

	return true;
}

bool cvar_set(const char *name, const char *value)
{
	cvar *cvar;

	if(!cvar_set_silent(name, value))
		return false;

	cvar = cvar_find(name);
	if(cvar->onchange)
		cvar->onchange();

	return true;
}

void recalculate_projection_matrix(void); // in world_renderer.c
void onchange_block_render_modes(void);   // in block.c
void onchange_ui_scale(void);             // in ui.c

cvar vid_width = {"vid_width", "854"};
cvar vid_height = {"vid_height", "480"};
cvar fov = {"fov", "90", recalculate_projection_matrix};
cvar r_zfar = {"r_zfar", "256", recalculate_projection_matrix};
cvar r_znear = {"r_znear", "0.1", recalculate_projection_matrix};
cvar r_max_remeshes = {"r_max_remeshes", "2"};
cvar r_fancyleaves = {"r_fancyleaves", "1", onchange_block_render_modes};
cvar r_smartleaves = {"r_smartleaves", "0", onchange_block_render_modes};
cvar gl_polygon_mode = {"gl_polygon_mode", "GL_FILL"};
cvar developer = {"developer", "0"};
cvar cvar_name = {"name", "player"};
cvar cl_2b2tmode = {"cl_2b2tmode", "0"};
cvar ui_scale = {"ui_scale", "2", onchange_ui_scale};
cvar sensitivity = {"sensitivity", "2.5"};
cvar cl_smoothstep = {"cl_smoothstep", "1"};

errcode cvar_init(void)
{
	cvar_register(&vid_width);
	cvar_register(&vid_height);
	cvar_register(&fov);
	cvar_register(&r_zfar);
	cvar_register(&r_znear);
	cvar_register(&r_max_remeshes);
	cvar_register(&r_fancyleaves);
	cvar_register(&r_smartleaves);
	cvar_register(&gl_polygon_mode);
	cvar_register(&developer);
	cvar_register(&cvar_name);
	cvar_register(&cl_2b2tmode);
	cvar_register(&ui_scale);
	cvar_register(&sensitivity);
	cvar_register(&cl_smoothstep);

    return ERR_OK;
}
