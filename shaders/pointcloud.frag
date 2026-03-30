#version 460 core
in float vDist;
out vec4 FragColor;
uniform float maxDist;
void main() {
    vec2 coord = gl_PointCoord * 2.0 - 1.0;
    if (dot(coord, coord) > 1.0) discard;
    float t = clamp(vDist / maxDist, 0.0, 1.0);
    // near=blue, mid=green, far=red
    vec3 color = mix(mix(vec3(0.0, 0.0, 1.0), vec3(0.0, 1.0, 0.0), t * 2.0),
                     mix(vec3(0.0, 1.0, 0.0), vec3(1.0, 0.0, 0.0), t * 2.0 - 1.0),
                     step(0.5, t));
    FragColor = vec4(color, 1.0);
}
