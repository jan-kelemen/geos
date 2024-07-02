#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;

layout(push_constant) uniform PushConsts {
    uint storageIndex;
} pushConsts;

layout(binding = 0) uniform Transform {
    mat4 view;
    mat4 projection;
    vec3 camera;
    vec3 lightPosition;
    vec3 lightColor;
} transform;

struct Storage {
    mat4 model;
};

layout(std140, binding = 1) readonly buffer StorageBuffer {
    Storage storage[];
} storageBuffer;

layout(location = 0) out vec3 outColor;
layout(location = 1) out vec3 outNormal;
layout(location = 2) out vec3 outFragmentPosition;

void main() {
    mat4 model = storageBuffer.storage[pushConsts.storageIndex].model;
    gl_Position = transform.projection * transform.view * model * vec4(inPosition, 1.0);

    outColor = inColor;
    outNormal = vec3(model * vec4(inNormal.xyz, 1.0));
    outFragmentPosition = vec3(model * vec4(inPosition.xyz, 1.0));
}
