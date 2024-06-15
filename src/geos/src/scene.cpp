#include <scene.hpp>

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

#include <glm/fwd.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>
#include <glm/vec3.hpp>

#include <vulkan/vulkan_core.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <span>

// IWYU pragma: no_include <filesystem>

namespace
{
    struct [[nodiscard]] vertex final
    {
        glm::fvec3 position;
    };

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
                .stride = sizeof(vertex),
                .inputRate = VK_VERTEX_INPUT_RATE_VERTEX},
        };

        return descriptions;
    }

    consteval auto attribute_descriptions()
    {
        constexpr std::array descriptions{
            VkVertexInputAttributeDescription{.location = 0,
                .binding = 0,
                .format = VK_FORMAT_R32G32B32_SFLOAT,
                .offset = offsetof(vertex, position)}};

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
            renderer->image_format()}
            .add_shader(VK_SHADER_STAGE_VERTEX_BIT, "scene.vert.spv", "main")
            .add_shader(VK_SHADER_STAGE_FRAGMENT_BIT, "scene.frag.spv", "main")
            .with_rasterization_samples(vulkan_device_->max_msaa_samples)
            .add_vertex_input(binding_description(), attribute_descriptions())
            .add_descriptor_set_layout(descriptor_set_layout_)
            .with_depth_test(depth_buffer_.format)
            .build());

    load_vertices();

    auto const uniform_buffer_size{sizeof(transform)};
    vertex_uniform_buffer_ = create_buffer(vulkan_device_,
        uniform_buffer_size * renderer->image_count(),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    frame_data_.resize(renderer->image_count());

    for (auto const& [index, data] : std::views::enumerate(frame_data_))
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

void geos::scene::detach_renderer()
{
    if (vulkan_device_)
    {
        for (auto& data : frame_data_)
        {
            vkFreeDescriptorSets(vulkan_device_->logical,
                vulkan_renderer_->descriptor_pool(),
                1,
                &data.descriptor_set_);
        }

        destroy(vulkan_device_, pipeline_.get());
        pipeline_.reset();

        vkDestroyDescriptorSetLayout(vulkan_device_->logical,
            descriptor_set_layout_,
            nullptr);

        destroy(vulkan_device_, &depth_buffer_);

        destroy(vulkan_device_, &vertex_uniform_buffer_);

        destroy(vulkan_device_, &vert_index_buffer_);
    }
    vulkan_device_ = nullptr;
}

void geos::scene::begin_frame()
{
    current_frame_ = (current_frame_ + 1) % vulkan_renderer_->image_count();
}

void geos::scene::end_frame() { }

void geos::scene::update()
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
            frame_data_[current_frame_].uniform_buffer_region_)};

        transform uniform{.model = glm::rotate(glm::mat4(1.0f),
                              time * glm::radians(90.0f),
                              glm::fvec3(0.0f, 0.0f, 1.0f)),
            .view = glm::lookAt(glm::fvec3(2.0f, 2.0f, 2.0f),
                glm::fvec3(0.0f, 0.0f, 0.0f),
                glm::fvec3(0.0f, 0.0f, 1.0f)),
            .projection =
                glm::perspective(glm::radians(45.0f), 1.f, 0.1f, 10.0f)};

        uniform.projection[1][1] *= -1;

        *uniform_map.as<transform>() = uniform;

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

void geos::scene::draw(VkCommandBuffer command_buffer, VkExtent2D const extent)
{
    vkCmdBindPipeline(command_buffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        pipeline_->pipeline);

    size_t const index_offset{8 * sizeof(vertex)};
    VkDeviceSize const zero_offsets{0};
    vkCmdBindVertexBuffers(command_buffer,
        0,
        1,
        &vert_index_buffer_.buffer,
        &zero_offsets);
    vkCmdBindIndexBuffer(command_buffer,
        vert_index_buffer_.buffer,
        index_offset,
        VK_INDEX_TYPE_UINT16);

    VkViewport const viewport{.x = 0.0f,
        .y = 0.0f,
        .width = cppext::as_fp(extent.width),
        .height = cppext::as_fp(extent.height),
        .minDepth = 0.0f,
        .maxDepth = 1.0f};
    vkCmdSetViewport(command_buffer, 0, 1, &viewport);

    VkRect2D const scissor{{0, 0}, extent};
    vkCmdSetScissor(command_buffer, 0, 1, &scissor);

    vkCmdBindDescriptorSets(command_buffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        pipeline_->pipeline_layout,
        0,
        1,
        &frame_data_[current_frame_].descriptor_set_,
        0,
        nullptr);

    vkCmdDrawIndexed(command_buffer, 12, 1, 0, 0, 0);
}

void geos::scene::draw_imgui() { }

void geos::scene::load_vertices()
{
    size_t const vertices_size{8 * sizeof(vertex)};
    size_t const indices_size{12 * sizeof(uint16_t)};

    vkrndr::vulkan_buffer staging_buffer{create_buffer(vulkan_device_,
        vertices_size + indices_size,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)};
    {
        vkrndr::mapped_memory vert_index_map{vkrndr::map_memory(vulkan_device_,
            staging_buffer.memory,
            vkrndr::memory_region{.offset = 0,
                .size = vertices_size + indices_size})};

        vertex* const vertices{vert_index_map.as<vertex>(0)};
        uint16_t* const indices{vert_index_map.as<uint16_t>(vertices_size)};

        vertices[0] = {{-0.5f, -0.5f, 0}};
        vertices[1] = {{0.5f, -0.5f, 0}};
        vertices[2] = {{0.5f, 0.5f, 0}};
        vertices[3] = {{-0.5f, 0.5f, 0}};

        vertices[4] = {{-0.5f, -0.5f, -0.5f}};
        vertices[5] = {{0.5f, -0.5f, -0.5f}};
        vertices[6] = {{0.5f, 0.5f, -0.5f}};
        vertices[7] = {{-0.5f, 0.5f, -0.5f}};

        indices[0] = 0;
        indices[1] = 1;
        indices[2] = 2;
        indices[3] = 2;
        indices[4] = 3;
        indices[5] = 0;

        indices[6] = 4;
        indices[7] = 5;
        indices[8] = 6;
        indices[9] = 6;
        indices[10] = 7;
        indices[11] = 4;

        unmap_memory(vulkan_device_, &vert_index_map);
    }

    vert_index_buffer_ = create_buffer(vulkan_device_,
        vertices_size + indices_size,
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vulkan_renderer_->transfer_buffer(staging_buffer, vert_index_buffer_);
    destroy(vulkan_device_, &staging_buffer);
}
