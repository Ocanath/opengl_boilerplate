#version 460 core
layout(location = 0) in vec3 aPos;
uniform mat4 view;
uniform mat4 projection;
uniform float pointSize;
out float vDist;
void main() {
    gl_Position = projection * view * vec4(aPos, 1.0);
    gl_PointSize = pointSize;
    vDist = length(aPos);
}
