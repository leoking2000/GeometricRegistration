#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;

uniform mat4 u_mvp;
uniform mat4 u_model;

out vec3 v_worldPos;
out vec3 v_worldNormal;

void main()
{
    vec4 worldPos = u_model * vec4(a_position, 1.0);
    v_worldPos = worldPos.xyz;

    v_worldNormal = normalize(mat3(transpose(inverse(u_model))) * a_normal);

    gl_Position = u_mvp * vec4(a_position, 1.0);
}