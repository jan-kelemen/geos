#include <application.hpp>

#include <mesh.hpp>

#include <cppext_numeric.hpp>

#include <gltf_manager.hpp>
#include <vulkan_buffer.hpp>
#include <vulkan_renderer.hpp>
#include <vulkan_utility.hpp>

geos::application::application()
    : camera_{glm::fvec3{3.0f, 3.0f, 3.0f},
          glm::fvec3{0.0f, 0.0f, 0.0f},
          45.0f,
          1.0f,
          0.1f,
          10.f}
{
}

geos::application::~application() { }

void geos::application::handle_event(SDL_Event const& event)
{
    if (event.type == SDL_WINDOWEVENT)
    {
        auto const& window{event.window};
        if (window.event == SDL_WINDOWEVENT_RESIZED ||
            window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
        {
            camera_.resize(cppext::narrow<uint32_t>(window.data1),
                cppext::narrow<uint32_t>(window.data2));
        }
    }
}

void geos::application::begin_frame() { scene_.begin_frame(); }

void geos::application::end_frame() { scene_.end_frame(); }

void geos::application::update([[maybe_unused]] float delta_time)
{
    camera_.update();
    scene_.update(camera_);
}

void geos::application::attach_renderer(vkrndr::vulkan_device* const device,
    vkrndr::vulkan_renderer* const renderer)
{
    load_meshes(device, renderer);
    scene_.attach_renderer(device, renderer);
}

void geos::application::detach_renderer(vkrndr::vulkan_device* const device,
    vkrndr::vulkan_renderer* renderer)
{
    for (auto entity : registry_.view<mesh_component>())
    {
        destroy(device,
            &registry_.get<mesh_component>(entity).mesh.vert_index_buffer);
    }
    registry_.clear<mesh_component>();

    scene_.detach_renderer(device, renderer);
}

VkClearValue geos::application::clear_color() { return scene_.clear_color(); }

VkClearValue geos::application::clear_depth() { return scene_.clear_depth(); }

vkrndr::vulkan_image* geos::application::depth_image()
{
    return scene_.depth_image();
}

void geos::application::resize(VkExtent2D extent)
{
    camera_.resize(extent.width, extent.height);
    scene_.resize(extent);
}

void geos::application::draw(VkCommandBuffer command_buffer, VkExtent2D extent)
{
    for (auto entity : registry_.view<mesh_component>())
    {
        scene_.draw(registry_.get<mesh_component>(entity).mesh,
            command_buffer,
            extent);
    }
}

void geos::application::draw_imgui() { camera_.debug(); }

void geos::application::load_meshes(vkrndr::vulkan_device* const device,
    vkrndr::vulkan_renderer* const renderer)
{
    std::unique_ptr<vkrndr::gltf_model> cube{renderer->load_model("cube.gltf")};

    auto const& cube_primitive{cube->nodes[0].mesh->primitives[0]};

    buffer_part cube_submesh{.vertex_offset = 0,
        .vertex_count = vkrndr::count_cast(cube_primitive.vertices.size()),
        .index_offset = 0,
        .index_count = vkrndr::count_cast(cube_primitive.indices.size())};

    size_t const vertices_size{cube_submesh.vertex_count * sizeof(vertex)};
    size_t const indices_size{cube_submesh.index_count * sizeof(uint32_t)};

    vkrndr::vulkan_buffer staging_buffer{vkrndr::create_buffer(device,
        vertices_size + indices_size,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)};
    {
        vkrndr::mapped_memory vert_index_map{vkrndr::map_memory(device,
            staging_buffer.memory,
            vkrndr::memory_region{.offset = 0,
                .size = vertices_size + indices_size})};

        vertex* const vertices{vert_index_map.as<vertex>(0)};
        uint32_t* const indices{vert_index_map.as<uint32_t>(vertices_size)};

        float color{-1.0f / 36};
        std::ranges::transform(cube_primitive.vertices,
            vertices,
            [&](vkrndr::gltf_vertex const& vert)
            {
                color += 1.0f / 36;
                return vertex{.position = vert.position,
                    .color = glm::fvec3(color, color, color)};
            });

        std::ranges::copy(cube_primitive.indices, indices);

        unmap_memory(device, &vert_index_map);
    }

    auto const cube_entity{registry_.create()};

    vkrndr::vulkan_buffer vert_index_buffer = create_buffer(device,
        staging_buffer.size,
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    renderer->transfer_buffer(staging_buffer, vert_index_buffer);
    destroy(device, &staging_buffer);

    gpu_mesh mesh{.vert_index_buffer = vert_index_buffer,
        .vertex_offset = 0,
        .index_offset = vertices_size,
        .submeshes = {cube_submesh}};

    registry_.emplace<mesh_component>(cube_entity, mesh);
}
