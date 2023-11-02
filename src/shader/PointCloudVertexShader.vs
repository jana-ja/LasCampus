#version 330 core
layout(location = 0) in vec3 worldPos;

uniform mat4 view;
uniform mat4 projection;
uniform float pointSize;
uniform vec3 cameraPos;

void main(){
    gl_Position = projection * view * vec4(worldPos, 1.0);
    gl_PointSize = pointSize * (100 / distance(cameraPos, worldPos.xyz) / 10);
}