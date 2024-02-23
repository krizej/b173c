#include "vid.h"
#include "mathlib.h"
#include "common.h"
#include "client/client.h"
#include "glad/glad.h"
#include "game/world.h"
#include "game/entity.h"
#include "client/console.h"
#include "hashmap.c/hashmap.h"
#include "meshbuilder.h"
#include "assets.h"

uint gl_vao, gl_vbo;
uint gl_uniform_model, gl_uniform_view, gl_uniform_projection;
extern struct gl_state gl;
extern mat4 view_mat, proj_mat;

errcode entity_renderer_init(void)
{
    vec3 entity_model_verts[] = {
            vec3_from(0,0,0),
            vec3_from(1,0,0),
            vec3_from(0,1,0)
    };

    glGenVertexArrays(1, &gl_vao);
    glGenBuffers(1, &gl_vbo);

    glBindVertexArray(gl_vao);
        glBindBuffer(GL_ARRAY_BUFFER, gl_vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(entity_model_verts), entity_model_verts, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), 0);
        glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    gl_uniform_model = glGetUniformLocation(gl.shader_model, "MODEL");
    gl_uniform_view = glGetUniformLocation(gl.shader_model, "VIEW");
    gl_uniform_projection = glGetUniformLocation(gl.shader_model, "PROJECTION");

    if(glGetError() != GL_NO_ERROR)
        return ERR_FATAL;
    return ERR_OK;
}

void entity_renderer_shutdown(void)
{
    glDeleteVertexArrays(1, &gl_vao);
    glDeleteBuffers(1, &gl_vbo);
}

void entity_renderer_render(void)
{
    size_t i;
    void *it;

    glUniformMatrix4fv(gl_uniform_view, 1, GL_FALSE, (const GLfloat *) view_mat);
    glUniformMatrix4fv(gl_uniform_projection, 1, GL_FALSE, (const GLfloat *) proj_mat);

    glBindVertexArray(gl_vao);

    glDisable(GL_CULL_FACE);
    i = 0;
    while(hashmap_iter(world_entity_map, &i, &it)) {
        entity *ent = it;
        mat4 model, t, r;

        mat4_identity(t);
        mat4_identity(r);
        mat4_translation(t, ent->position);
        mat4_rotation(r, ent->rotation);
        mat4_multiply(model, r, t);

        glUniformMatrix4fv(gl_uniform_model, 1, GL_FALSE, (const GLfloat *) model);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 3);
    }
    glEnable(GL_CULL_FACE);

    glBindVertexArray(0);
}
