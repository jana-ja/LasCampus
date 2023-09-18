#version 330 core
layout(location = 0) in vec3 worldPos;
layout(location = 1) in vec3 vertColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform float pointSize;

out vec3 vertexColor;

void main(){
    gl_Position = projection * view * model * vec4(worldPos, 1.0);
    vertexColor = vertColor;
}