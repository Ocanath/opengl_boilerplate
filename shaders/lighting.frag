#version 460 core

in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D gPosition;
uniform sampler2D gNormal;
uniform sampler2D gAlbedo;
uniform vec3      viewPos;
uniform int       numLights;

struct LightData {
    vec3  position;   // bytes 0-11
    float intensity;  // bytes 12-15
    vec3  color;      // bytes 16-27
    float radius;     // bytes 28-31
};
layout(std430, binding = 0) readonly buffer LightBuffer { LightData lights[]; };

void main()
{
    vec4 posSample = texture(gPosition, TexCoord);

    // Discard sky pixels (no geometry written here — w stays 0)
    if (posSample.w < 0.5) discard;

    vec3 fragPos = posSample.xyz;
    vec3 normal  = normalize(texture(gNormal, TexCoord).xyz);
    vec3 albedo  = texture(gAlbedo, TexCoord).rgb;

    vec3 viewDir = normalize(viewPos - fragPos);
    vec3 result  = 0.05 * albedo; // ambient

    for (int i = 0; i < numLights; ++i) {
        float dist = length(lights[i].position - fragPos);
        if (dist > lights[i].radius) continue; // radius cull

        float att  = lights[i].intensity / (1.0 + 0.09 * dist + 0.032 * dist * dist);
        vec3  ldir = normalize(lights[i].position - fragPos);

        float diff  = max(dot(normal, ldir), 0.0);
        vec3  half_ = normalize(ldir + viewDir);
        float spec  = pow(max(dot(normal, half_), 0.0), 64.0);

        result += (diff * albedo + vec3(spec)) * lights[i].color * att;
    }

    FragColor = vec4(result, 1.0);
}
