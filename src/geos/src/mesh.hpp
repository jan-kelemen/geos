#ifndef GEOS_MESH_INCLUDED
#define GEOS_MESH_INCLUDED

#include <cppext_pragma_warning.hpp>

#include <vulkan_buffer.hpp>

#include <glm/fwd.hpp>
#include <glm/vec3.hpp> // IWYU pragma: keep

#include <vulkan/vulkan_core.h>

#include <cstdint>
#include <vector>

namespace geos
{
    struct [[nodiscard]] buffer_part final
    {
        int32_t vertex_offset;
        uint32_t vertex_count;
        int32_t index_offset;
        uint32_t index_count;
    };

    struct [[nodiscard]] gpu_mesh final
    {
        vkrndr::vulkan_buffer vert_index_buffer;
        VkDeviceSize vertex_offset{};
        VkDeviceSize index_offset{};
        std::vector<buffer_part> submeshes;
    };

    struct [[nodiscard]] mesh_component final
    {
        gpu_mesh mesh;
    };

    DISABLE_WARNING_PUSH
    DISABLE_WARNING_STRUCTURE_WAS_PADDED_DUE_TO_ALIGNMENT_SPECIFIER

    struct [[nodiscard]] vertex final
    {
        alignas(16) glm::fvec3 position;
        alignas(16) glm::fvec3 color;
        alignas(16) glm::fvec3 normal;
    };

    DISABLE_WARNING_POP

} // namespace geos

#endif
