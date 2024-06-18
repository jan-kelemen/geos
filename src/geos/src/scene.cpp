#include <scene.hpp>

#include <camera.hpp>
#include <mesh.hpp>

#include <vulkan_buffer.hpp>
#include <vulkan_depth_buffer.hpp>
#include <vulkan_descriptors.hpp>
#include <vulkan_device.hpp>
#include <vulkan_image.hpp>
#include <vulkan_memory.hpp>
#include <vulkan_pipeline.hpp>
#include <vulkan_renderer.hpp>
#include <vulkan_utility.hpp>

#include <cppext_numeric.hpp>
#include <cppext_pragma_warning.hpp>

#include <glm/fwd.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>
#include <glm/vec3.hpp>

#include <vulkan/vulkan_core.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <span>

// IWYU pragma: no_include <filesystem>

namespace
{
    struct [[nodiscard]] transform final
    {
        glm::fmat4 model;
        glm::fmat4 view;
        glm::fmat4 projection;
    };

    consteval auto binding_description()
    {
        constexpr std::array descriptions{
            VkVertexInputBindingDescription{.binding = 0,
                .stride = sizeof(geos::vertex),
                .inputRate = VK_VERTEX_INPUT_RATE_VERTEX}};

        return descriptions;
    }

    consteval auto attribute_descriptions()
    {
        constexpr std::array descriptions{
            VkVertexInputAttributeDescription{.location = 0,
                .binding = 0,
                .format = VK_FORMAT_R32G32B32_SFLOAT,
                .offset = offsetof(geos::vertex, position)},
            VkVertexInputAttributeDescription{.location = 1,
                .binding = 0,
                .format = VK_FORMAT_R32G32B32_SFLOAT,
                .offset = offsetof(geos::vertex, color)}};

        return descriptions;
    }

    [[nodiscard]] VkDescriptorSetLayout create_descriptor_set_layout(
        vkrndr::vulkan_device const* const device)
    {
        VkDescriptorSetLayoutBinding vertex_uniform_binding{};
        vertex_uniform_binding.binding = 0;
        vertex_uniform_binding.descriptorType =
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        vertex_uniform_binding.descriptorCount = 1;
        vertex_uniform_binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

        std::array const bindings{vertex_uniform_binding};

        VkDescriptorSetLayoutCreateInfo layout_info{};
        layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        layout_info.bindingCount = vkrndr::count_cast(bindings.size());
        layout_info.pBindings = bindings.data();

        VkDescriptorSetLayout rv; // NOLINT
        vkrndr::check_result(vkCreateDescriptorSetLayout(device->logical,
            &layout_info,
            nullptr,
            &rv));

        return rv;
    }

    void bind_descriptor_set(vkrndr::vulkan_device const* const device,
        VkDescriptorSet const& descriptor_set,
        VkDescriptorBufferInfo const vertex_buffer_info)
    {
        VkWriteDescriptorSet vertex_descriptor_write{};
        vertex_descriptor_write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        vertex_descriptor_write.dstSet = descriptor_set;
        vertex_descriptor_write.dstBinding = 0;
        vertex_descriptor_write.dstArrayElement = 0;
        vertex_descriptor_write.descriptorType =
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        vertex_descriptor_write.descriptorCount = 1;
        vertex_descriptor_write.pBufferInfo = &vertex_buffer_info;

        std::array const descriptor_writes{vertex_descriptor_write};

        vkUpdateDescriptorSets(device->logical,
            vkrndr::count_cast(descriptor_writes.size()),
            descriptor_writes.data(),
            0,
            nullptr);
    }
} // namespace

geos::scene::scene() = default;

geos::scene::~scene() = default;

void geos::scene::attach_renderer(vkrndr::vulkan_device* device,
    vkrndr::vulkan_renderer* renderer)
{
    vulkan_device_ = device;
    vulkan_renderer_ = renderer;

    descriptor_set_layout_ = create_descriptor_set_layout(vulkan_device_);

    depth_buffer_ =
        vkrndr::create_depth_buffer(device, renderer->extent(), false);

    pipeline_ = std::make_unique<vkrndr::vulkan_pipeline>(
        vkrndr::vulkan_pipeline_builder{vulkan_device_,
            vkrndr::vulkan_pipeline_layout_builder{vulkan_device_}
                .add_descriptor_set_layout(descriptor_set_layout_)
                .build(),
            renderer->image_format()}
            .add_shader(VK_SHADER_STAGE_VERTEX_BIT, "scene.vert.spv", "main")
            .add_shader(VK_SHADER_STAGE_FRAGMENT_BIT, "scene.frag.spv", "main")
            .with_rasterization_samples(vulkan_device_->max_msaa_samples)
            .add_vertex_input(binding_description(), attribute_descriptions())
            .with_culling(VK_CULL_MODE_BACK_BIT, VK_FRONT_FACE_CLOCKWISE)
            .with_depth_test(depth_buffer_.format)
            .build());

    auto const uniform_buffer_size{sizeof(transform)};
    vertex_uniform_buffer_ = create_buffer(vulkan_device_,
        uniform_buffer_size * renderer->image_count(),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    frame_data_ = cppext::cyclic_stack<frame_data>{renderer->image_count(),
        renderer->image_count()};
    for (auto const& [index, data] :
        std::views::enumerate(frame_data_.as_span()))
    {
        data.uniform_buffer_region_ = {
            .offset = cppext::narrow<VkDeviceSize>(index) * uniform_buffer_size,
            .size = uniform_buffer_size};

        create_descriptor_sets(vulkan_device_,
            descriptor_set_layout_,
            renderer->descriptor_pool(),
            std::span{&data.descriptor_set_, 1});

        bind_descriptor_set(vulkan_device_,
            data.descriptor_set_,
            VkDescriptorBufferInfo{.buffer = vertex_uniform_buffer_.buffer,
                .offset = data.uniform_buffer_region_.offset,
                .range = data.uniform_buffer_region_.size});
    }
}

void geos::scene::detach_renderer(vkrndr::vulkan_device* device,
    vkrndr::vulkan_renderer* renderer)
{
    if (device)
    {
        for (auto& data : frame_data_.as_span())
        {
            vkFreeDescriptorSets(device->logical,
                renderer->descriptor_pool(),
                1,
                &data.descriptor_set_);
        }

        destroy(device, pipeline_.get());
        pipeline_.reset();

        vkDestroyDescriptorSetLayout(device->logical,
            descriptor_set_layout_,
            nullptr);

        destroy(device, &depth_buffer_);

        destroy(device, &vertex_uniform_buffer_);
    }
    vulkan_device_ = nullptr;
}

void geos::scene::begin_frame() { frame_data_.cycle(); }

void geos::scene::end_frame() { }

void geos::scene::update(camera const& camera, glm::fvec3 const& translate)
{
    {
        static auto start_time{std::chrono::high_resolution_clock::now()};
        auto const current_time{std::chrono::high_resolution_clock::now()};
        float const time{
            std::chrono::duration<float, std::chrono::seconds::period>(
                current_time - start_time)
                .count()};

        vkrndr::mapped_memory uniform_map{vkrndr::map_memory(vulkan_device_,
            vertex_uniform_buffer_.memory,
            frame_data_.top().uniform_buffer_region_)};

        *uniform_map.as<transform>() = {
            .model = glm::translate(glm::fmat4{1.0f}, translate),
            .view = camera.view_matrix(),
            .projection = camera.projection_matrix()};

        unmap_memory(vulkan_device_, &uniform_map);
    }
}

VkClearValue geos::scene::clear_color() { return {{{1.f, .5f, .3f, 1.f}}}; }

VkClearValue geos::scene::clear_depth() { return {.depthStencil = {1.0f, 0}}; }

vkrndr::vulkan_image* geos::scene::depth_image() { return &depth_buffer_; }

void geos::scene::resize(VkExtent2D const extent)
{
    destroy(vulkan_device_, &depth_buffer_);
    depth_buffer_ = vkrndr::create_depth_buffer(vulkan_device_, extent, false);
}

void geos::scene::draw(gpu_mesh const& mesh,
    VkCommandBuffer command_buffer,
    VkExtent2D const extent)
{
    vkCmdBindVertexBuffers(command_buffer,
        0,
        1,
        &mesh.vert_index_buffer.buffer,
        &mesh.vertex_offset);

    vkCmdBindIndexBuffer(command_buffer,
        mesh.vert_index_buffer.buffer,
        mesh.index_offset,
        VK_INDEX_TYPE_UINT32);

    VkViewport const viewport{.x = 0.0f,
        .y = 0.0f,
        .width = cppext::as_fp(extent.width),
        .height = cppext::as_fp(extent.height),
        .minDepth = 0.0f,
        .maxDepth = 1.0f};
    vkCmdSetViewport(command_buffer, 0, 1, &viewport);

    VkRect2D const scissor{{0, 0}, extent};
    vkCmdSetScissor(command_buffer, 0, 1, &scissor);

    vkrndr::bind_pipeline(command_buffer,
        *pipeline_,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        0,

        std::span<VkDescriptorSet const>{&frame_data_.top().descriptor_set_,
            1});

    for (auto const submesh : mesh.submeshes)
    {
        vkCmdDrawIndexed(command_buffer,
            submesh.index_count,
            1,
            vkrndr::count_cast(submesh.index_offset),
            submesh.vertex_offset,
            0);
    }
}
