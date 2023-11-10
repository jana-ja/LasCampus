#version 330 core
layout(location = 0) in vec3 worldPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 view;
uniform mat4 projection;
uniform float pointSize;
uniform vec3 cameraPos;

out vec3 normal;
out vec3 fragPos;

void main(){
    gl_Position = projection * view * vec4(worldPos, 1.0);
    gl_PointSize = pointSize * (100 / distance(cameraPos, worldPos.xyz) / 10);
    normal = aNormal;
    fragPos = worldPos;
}