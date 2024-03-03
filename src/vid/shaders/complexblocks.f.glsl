#version 430 core

in vec3 COLORMOD;
in vec2 UV_COORD;

out vec4 COLOR;

uniform vec3 CHUNK_POS;
uniform sampler2D TEXTURE;

void main()
{
    // hack for block selection box
    if (CHUNK_POS == vec3(0) && COLORMOD == vec3(0) && UV_COORD == vec2(0)) {
         COLOR = vec4(0.0, 0.0, 0.0, 0.4);
         return;
    }

    COLOR = texture(TEXTURE, UV_COORD) * vec4(COLORMOD, 1.0);
    if (COLOR.a < 0.01) {
        discard;
    }
}