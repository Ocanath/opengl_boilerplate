#version 460 core

#define MAX_LIGHTS 16

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoord;

uniform int   numLights;
uniform vec3  lightPositions[MAX_LIGHTS];
uniform vec3  lightColors[MAX_LIGHTS];
uniform float lightIntensities[MAX_LIGHTS];
uniform vec3  viewPos;
uniform vec3  objectColor;
uniform bool  unlit;

out vec4 FragColor;

void main()
{
    if (unlit) {
        FragColor = vec4(objectColor, 1.0);
        return;
    }
    vec3 norm     = normalize(Normal);
    vec3 viewDir  = normalize(viewPos - FragPos);

    // Ambient
    vec3 ambient = 0.05 * objectColor;

    vec3 result = ambient;

    for (int i = 0; i < numLights && i < MAX_LIGHTS; ++i)
    {
        vec3  lightDir   = normalize(lightPositions[i] - FragPos);
        float dist       = length(lightPositions[i] - FragPos);
        float attenuation = lightIntensities[i] / (1.0 + 0.09 * dist + 0.032 * dist * dist);

        // Diffuse
        float diff   = max(dot(norm, lightDir), 0.0);
        vec3  diffuse = diff * lightColors[i] * objectColor * attenuation;

        // Specular (Blinn-Phong)
        vec3  halfDir = normalize(lightDir + viewDir);
        float spec    = pow(max(dot(norm, halfDir), 0.0), 64.0);
        vec3  specular = spec * lightColors[i] * attenuation;

        result += diffuse + specular;
    }

    FragColor = vec4(result, 1.0);
}
