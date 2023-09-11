#version 330 core
layout(location = 0) in vec3 worldPos;

//uniform mat4 model; point cloud already is in world space
uniform mat4 view;
uniform mat4 projection;
uniform float pointsize;
uniform vec3 cameraeye;

void main(){
    gl_Position = projection * view * vec4(worldPos, 1.0); //projection * view * model * vec4(worldPos, 1.0);
    gl_PointSize = pointsize - (distance(cameraeye, worldPos.xyz) * 20 / pointsize);
}