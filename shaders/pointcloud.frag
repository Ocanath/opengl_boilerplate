#version 460 core
out vec4 FragColor;
uniform vec3 color;
void main() {
    // Circular point sprite — discard corners
    vec2 coord = gl_PointCoord * 2.0 - 1.0;
    if (dot(coord, coord) > 1.0) discard;
    FragColor = vec4(color, 1.0);
}
