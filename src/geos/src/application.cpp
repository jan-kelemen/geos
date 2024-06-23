#include <application.hpp>

#include <camera.hpp>
#include <glm/ext/matrix_projection.hpp>
#include <mesh.hpp>
#include <physics.hpp>
#include <scene.hpp>

#include <cppext_numeric.hpp>

#include <gltf_manager.hpp>
#include <vulkan_buffer.hpp>
#include <vulkan_memory.hpp>
#include <vulkan_renderer.hpp>
#include <vulkan_utility.hpp>

#include <fmt/format.h>

#include <glm/fwd.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>

#include <SDL_events.h>
#include <SDL_video.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <vector>

// IWYU pragma: no_include <filesystem>

geos::application::application(uint32_t width, uint32_t height)
    : camera_{glm::fvec3{10.0f, 10.0f, 10.0f},
          glm::fvec3{2.0f, -5.0f, 0.0f},
          width,
          height,
          45.0f,
          1.0f,
          0.1f,
          100.f}
{
}

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
    else if (event.type == SDL_MOUSEMOTION)
    {
        auto const& motion{event.motion};
        auto const& [near, far] = camera_.raycast(motion.x, motion.y);
        auto const* const hit{
            physics_simulation_.raycast({near.x, near.y, near.z},
                {far.x, far.y, far.z})};
        if (hit)
        {
            fmt::println("hit {}", hit->getUserIndex());
        }
    }
}

void geos::application::begin_frame() { scene_.begin_frame(); }

void geos::application::end_frame() { scene_.end_frame(); }

void geos::application::update(float const delta_time)
{
    physics_simulation_.update(delta_time);

    camera_.update();

    scene_.update(camera_);
}

void geos::application::attach_renderer(vkrndr::vulkan_device* const device,
    vkrndr::vulkan_renderer* const renderer)
{
    load_meshes(device, renderer);
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
    glm::fmat4x4 gpu_transform;
    for (auto entity : registry_.view<physics_component, mesh_component>())
    {
        btTransform const transform{
            registry_.get<physics_component>(entity).position()};
        transform.getOpenGLMatrix(glm::value_ptr(gpu_transform));

        scene_.draw(registry_.get<mesh_component>(entity).mesh,
            gpu_transform,
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

    static std::mt19937 generator{std::random_device{}()};
    static int entity_count{1};

    auto const& phyisics_component{registry_.emplace<physics_component>(
        cube_entity,
        physics_simulation_.add_rigid_body(
            std::make_unique<btBoxShape>(btVector3{1, 1, 1}),
            1.0f,
            btVector3{cppext::as_fp(
                          std::uniform_int_distribution<int>(-5, 5)(generator)),
                10,
                cppext::as_fp(
                    std::uniform_int_distribution<int>(-5, 5)(generator))}))};

    phyisics_component.rigid_body_->setUserIndex(++entity_count);
}
