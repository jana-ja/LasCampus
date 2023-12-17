#version 330 core
layout(location = 0) in vec3 v_world_pos;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec3 v_color;
layout(location = 3) in float v_radius;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform float point_size;
uniform vec3 camera_pos;
uniform vec3 light_dir;
uniform float size_const;

out vec3 v2f_color;
out vec3 v2f_normal;
out vec3 v2f_center;
out float v2f_radius;
out vec3 v2f_light_dir;

void main(){

    // float v_radius = 0.1;
    float size_fac = 2.2;


    v2f_normal = (view_matrix * vec4(v_normal, 0.0)).xyz;
    v2f_center = (view_matrix * vec4(v_world_pos, 1.0)).xyz;
    v2f_color = v_color;
    v2f_radius = v_radius * size_fac;
    v2f_light_dir = (view_matrix * vec4(light_dir, 0.0)).xyz;


    gl_Position = projection_matrix * view_matrix * vec4(v_world_pos, 1.0);
    gl_PointSize = (v_radius / -v2f_center.z * size_const * size_fac);

    // backface culling
        if (dot(v2f_normal, v2f_center) > 0.0) gl_Position.w = 0.0;

    // TODO blending, sharp corners, phong splatting(?), real radius sizes
}