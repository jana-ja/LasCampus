#version 330 core
layout(location = 0) in vec3 v_world_pos;
layout(location = 1) in vec3 v_color;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

out vec3 v2f_color;

void main(){
    gl_Position = projection_matrix * view_matrix * model_matrix * vec4(v_world_pos, 1.0);
    v2f_color = v_color;
}