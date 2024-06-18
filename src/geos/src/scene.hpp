#ifndef GEOS_SCENE_INLCUDED
#define GEOS_SCENE_INLCUDED

#include <cppext_cyclic_stack.hpp>

#include <vulkan_buffer.hpp>
#include <vulkan_image.hpp>
#include <vulkan_memory.hpp>
#include <vulkan_scene.hpp>

#include <glm/fwd.hpp>
#include <glm/vec3.hpp> // IWYU pragma: keep

#include <vulkan/vulkan_core.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace vkrndr
{
    struct vulkan_device;
    struct vulkan_pipeline;
    class vulkan_renderer;
} // namespace vkrndr

namespace geos
{
    class camera;
    struct gpu_mesh;
} // namespace geos

namespace geos
{
    class [[nodiscard]] scene final
    {
    public: // Construction
        scene();

        scene(scene const&) = delete;

        scene(scene&&) noexcept = delete;

    public: // Destruction
        ~scene();

    public: // Interface
        void attach_renderer(vkrndr::vulkan_device* device,
            vkrndr::vulkan_renderer* renderer);

        void detach_renderer(vkrndr::vulkan_device* device,
            vkrndr::vulkan_renderer* renderer);

        void begin_frame();

        void end_frame();

        void update(camera const& camera, glm::fvec3 const& translate);

    public:
        [[nodiscard]] VkClearValue clear_color();

        [[nodiscard]] VkClearValue clear_depth();

        [[nodiscard]] vkrndr::vulkan_image* depth_image();

        void resize(VkExtent2D extent);

        void draw(gpu_mesh const& mesh,
            VkCommandBuffer command_buffer,
            VkExtent2D extent);

    public: // Operators
        scene& operator=(scene const&) = delete;

        scene& operator=(scene&&) noexcept = delete;

    private: // Types
        struct [[nodiscard]] frame_data final
        {
            VkDescriptorSet descriptor_set_{VK_NULL_HANDLE};
            vkrndr::memory_region uniform_buffer_region_;
        };

    private: // Data
        vkrndr::vulkan_device* vulkan_device_{};
        vkrndr::vulkan_renderer* vulkan_renderer_{};

        vkrndr::vulkan_buffer vertex_uniform_buffer_;
        vkrndr::vulkan_image depth_buffer_;

        VkDescriptorSetLayout descriptor_set_layout_{VK_NULL_HANDLE};
        std::unique_ptr<vkrndr::vulkan_pipeline> pipeline_;

        cppext::cyclic_stack<frame_data> frame_data_;
    };
} // namespace geos

#endif
