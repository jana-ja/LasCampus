#version 330 core

uniform vec3 light_color;
uniform vec3 light_pos;
uniform vec3 camera_pos;
uniform sampler2D ortho_texture;

in vec3 v2f_color;
in vec3 v2f_normal;
in vec3 v2f_pos;
in vec2 v2f_tex_coord;

out vec4 f_color;


void main(){
    float ambient_strength = 0.4;//0.1;
    vec3 ambient_color = ambient_strength * light_color;

    vec3 normal = normalize(v2f_normal);
    vec3 light_dir = normalize(light_pos - v2f_pos);
    float diffuse = max(dot(normal, light_dir), 0.0);
    vec3 diffuse_color = diffuse * light_color;

    vec3 view_dir = normalize(camera_pos - v2f_pos);
    vec3 reflect_dir = reflect(-light_dir, normal);
    float specular_strength = 0.5;
    float specular = pow(max(dot(view_dir, reflect_dir), 0.0), 32);
    vec3 specular_color = specular_strength * specular * light_color;

    vec3 result_color = (ambient_color + diffuse_color + specular_color) * v2f_color;
    f_color = texture(ortho_texture, v2f_tex_coord);//vec4(result_color, 1.0);
}