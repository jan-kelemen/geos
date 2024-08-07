#version 450

layout(location = 0) in vec3 inColor;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inFragmentPosition;

layout(binding = 0) uniform Transform {
    mat4 view;
    mat4 projection;
    vec3 camera;
    vec3 lightPosition;
    vec3 lightColor;
} transform;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 lightColor = transform.lightColor;
    vec3 lightPosition = transform.lightPosition;

    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;

    vec3 norm = normalize(inNormal);
    vec3 lightDirection = normalize(lightPosition - inFragmentPosition);  

    float diff = max(dot(norm, lightDirection), 0.0);
    vec3 diffuse = diff * lightColor;

    float specularStrength = 0.5;
    vec3 viewDir = normalize(transform.camera - inFragmentPosition);
    vec3 reflectDirection = reflect(-lightDirection, norm);  
    float spec = pow(max(dot(viewDir, reflectDirection), 0.0), 32);
    vec3 specular = specularStrength * spec * lightColor;  

    vec3 result = (ambient + diffuse + specular) * inColor;

    outColor = vec4(result, 1.);
}
