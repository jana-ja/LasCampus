#version 330 core
layout(location = 0) in vec3 v_world_pos;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec3 v_color;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform float point_size;
uniform vec3 camera_pos;

out vec3 v2f_color;
out vec3 v2f_normal;
out vec3 v2f_pos;

void main(){
    gl_Position = projection_matrix * view_matrix * vec4(v_world_pos, 1.0);
    gl_PointSize = point_size * (100 / distance(camera_pos, v_world_pos.xyz) / 10);

    v2f_normal = v_normal;
    v2f_pos = v_world_pos;
    v2f_color = v_color;
}