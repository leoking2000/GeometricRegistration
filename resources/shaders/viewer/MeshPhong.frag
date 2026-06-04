#version 330 core

in vec3 v_worldPos;
in vec3 v_worldNormal;

uniform vec3 u_color;
uniform vec3 u_lightDir;
uniform vec3 u_cameraPos;

out vec4 FragColor;

void main()
{
    vec3 N = normalize(v_worldNormal);

    vec3 L = normalize(-u_lightDir);

    vec3 V = normalize(u_cameraPos - v_worldPos);

    vec3 R = reflect(-L, N);

    // Ambient
    float ambient = 0.2;

    // Diffuse
    float diffuse = max(dot(N, L), 0.0);

    // Specular
    float specular = pow(max(dot(R, V), 0.0), 32.0);

    vec3 lighting = min(vec3(1.0), vec3(ambient + diffuse + 0.4 * specular));

    FragColor = vec4(u_color * lighting, 1.0);
	//FragColor = vec4(u_color, 1.0);
	//FragColor = vec4(normalize(v_worldNormal) * 0.5 + 0.5, 1.0);
}