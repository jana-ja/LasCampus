#version 330 core
layout(location = 0) in vec3 v_world_pos;
layout(location = 2) in vec3 v_color;
layout(location = 4) in vec3 v_tangent0;
layout(location = 5) in vec3 v_tangent1;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform float point_size;
uniform vec3 light_dir;
uniform float size_const;
uniform bool backface_culling;

out vec3 v2f_color;
out vec3 v2f_center;
out vec3  v2f_tangent0;
out vec3  v2f_tangent1;
out vec3 v2f_light_dir;

void main(){


    // float v_radius = 0.1;
    float size_fac = 1.8;

    //v2f_normal = (view_matrix * vec4(v_normal, 0.0)).xyz;

    v2f_tangent0 = (view_matrix * vec4((v_tangent0 / size_fac), 0.0)).xyz;
    v2f_tangent1 = (view_matrix * vec4((v_tangent1 / size_fac), 0.0)).xyz;
    v2f_center = (view_matrix * vec4(v_world_pos, 1.0)).xyz;
    v2f_color = v_color;
    //v2f_radius = v_radius * size_fac;
    v2f_light_dir = (view_matrix * vec4(light_dir, 0.0)).xyz;


    gl_Position = projection_matrix * view_matrix * vec4(v_world_pos, 1.0);
    float radius = 1.0 / min(length(v_tangent0), length(v_tangent1));
    gl_PointSize = (radius / -v2f_center.z * size_const * size_fac);
if (v_tangent0 == vec3(0,0,0)) {
        gl_PointSize = 0;
    }
    // backface culling
    if (backface_culling){
        vec3 n = cross(v2f_tangent0, v2f_tangent1);
        if (dot(n, v2f_center) > 0.0) gl_Position.w = 0.0;
    }
}