#version 330 core
layout(location = 0) in vec3 v_world_pos;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec3 v_color;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform float point_size;
uniform vec3 camera_pos;
uniform vec3 light_dir;

out vec3 v2f_color;
out vec3 v2f_normal;
out vec3 v2f_center;
out float v2f_radius; // TODO als nächstes diesen radius genau anschauen, wegen dem wird glaube ich bei mir zu viel discarded im fragment shader
out vec3 v2f_light_dir;

void main(){
    gl_Position = projection_matrix * view_matrix * vec4(v_world_pos, 1.0);
    gl_PointSize = point_size * (100 / distance(camera_pos, v_world_pos.xyz) / 10); // TODO hier berechnen die auch dinge

    v2f_normal = (view_matrix * vec4(v_normal, 0.0)).xyz; // TODO alles in view space, brauche ich hier normal matrix? hab ja kein scaling
    //v2f_center = v_world_pos;
    v2f_center = (view_matrix * vec4(v_world_pos, 0.0)).xyz; // schauen was die da geben, TODO die machen modelview matrix, model habe ich ja nicht, also aber mit view matrix wohl?
    v2f_color = v_color;
    v2f_radius = gl_PointSize * 1000; // TODO die berechnen dinge
    v2f_light_dir = (view_matrix * vec4(light_dir, 0.0)).xyz;

    // TODO backface culling
}