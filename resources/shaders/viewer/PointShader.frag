#version 330 core

in vec3 v_pos;

uniform vec3 u_color;

out vec4 FragColor;

void main()
{
    // circular points
    vec2 uv = gl_PointCoord * 2.0 - 1.0;
    if (dot(uv, uv) > 1.0)
        discard;

    FragColor = vec4(u_color, 1.0);
}