#include <application.hpp>

#include <camera.hpp>
#include <mesh.hpp>
#include <physics.hpp>
#include <scene.hpp>

#include <cppext_numeric.hpp>

#include <gltf_manager.hpp>
#include <vulkan_buffer.hpp>
#include <vulkan_memory.hpp>
#include <vulkan_renderer.hpp>
#include <vulkan_utility.hpp>

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>

#include <fmt/core.h>

#include <glm/fwd.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec3.hpp>

#include <SDL2/SDL_events.h>
#include <SDL2/SDL_mouse.h>
#include <SDL2/SDL_scancode.h>
#include <SDL2/SDL_stdinc.h>
#include <SDL2/SDL_video.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <ranges>
#include <vector>

// IWYU pragma: no_include <filesystem>

geos::application::application(uint32_t const width,
    uint32_t const height,
    bool const capture_mouse)
    : capture_mouse_{capture_mouse}
    , camera_{glm::fvec3{10.0f, 2.0f, 10.0f},
          width,
          height,
          45.0f,
          1.0f,
          0.1f,
          100.f,
          -135.0f,
          0.0f}
{
    if (capture_mouse_)
    {
        SDL_SetRelativeMouseMode(SDL_TRUE);
    }

    auto* const terrain_box{physics_simulation_.add_rigid_body(
        std::make_unique<btBoxShape>(btVector3{50.0f, 1.0f, 50.0f}),
        0.0f,
        btVector3{0.0f, -1.0f, 0.0f})};
    terrain_box->setUserIndex(1);
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
        auto const& [near, far] =
            camera_.raycast(cppext::narrow<uint32_t>(motion.x),
                cppext::narrow<uint32_t>(motion.y));
        auto const* const hit{
            physics_simulation_.raycast({near.x, near.y, near.z},
                {far.x, far.y, far.z})};
        if (hit)
        {
            fmt::println("hit {}", hit->getUserIndex());
        }
    }
    else if (event.type == SDL_KEYDOWN)
    {
        auto const& keyboard{event.key};
        if (keyboard.keysym.scancode == SDL_SCANCODE_F3)
        {
            capture_mouse_ ^= true;
            if (capture_mouse_)
            {
                SDL_SetRelativeMouseMode(SDL_TRUE);
                SDL_GetRelativeMouseState(nullptr, nullptr);
            }
            else
            {
                SDL_SetRelativeMouseMode(SDL_FALSE);
            }
        }
    }
}

void geos::application::begin_frame() { scene_.begin_frame(); }

void geos::application::end_frame() { scene_.end_frame(); }

void geos::application::fixed_update([[maybe_unused]] float const delta_time)
{
    if (capture_mouse_)
    {
        int x; // NOLINT
        int y; // NOLINT
        SDL_GetRelativeMouseState(&x, &y);

        camera_.mouse_movement(-cppext::as_fp(x), cppext::as_fp(y));
    }
}

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
    scene_.attach_renderer(device, renderer);
}

void geos::application::detach_renderer(vkrndr::vulkan_device* const device,
    vkrndr::vulkan_renderer* renderer)
{
    scene_.detach_renderer(device, renderer);

    registry_.clear<mesh_component>();
    destroy(device, &model_mesh_);
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
    std::unique_ptr<vkrndr::gltf_model> model{
        renderer->load_model("geos.gltf")};

    std::vector<vkrndr::gltf_mesh const*> load_meshes;
    std::vector<std::pair<buffer_part, glm::fmat4>> parts;
    std::vector<std::unique_ptr<btTriangleMesh>> collision_meshes;

    int32_t vertex_count{};
    int32_t index_count{};
    for (auto const& node : model->nodes)
    {
        auto const& model_mesh{node.mesh};
        load_meshes.push_back(&(*model_mesh));

        auto const& current_mesh{parts.emplace_back(
            buffer_part{vertex_count,
                vkrndr::count_cast(model_mesh->primitives[0].vertices.size()),
                index_count,
                vkrndr::count_cast(model_mesh->primitives[0].indices.size())},
            local_matrix(node))};

        vertex_count += current_mesh.first.vertex_count;
        index_count += current_mesh.first.index_count;
    }

    size_t const vertices_size{vertex_count * sizeof(vertex)};
    size_t const indices_size{index_count * sizeof(uint32_t)};

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

        vertex* vertices{vert_index_map.as<vertex>(0)};
        uint32_t* indices{vert_index_map.as<uint32_t>(vertices_size)};

        for (size_t i{}; i != load_meshes.size(); ++i)
        {
            auto const& mesh{load_meshes[i]};
            auto const& part{parts[i].first};

            auto const& gltf_vertices{mesh->primitives[0].vertices};
            auto const& gltf_indices{mesh->primitives[0].indices};

            float color{-1.0f / part.vertex_count};
            vertices = std::ranges::transform(gltf_vertices,
                vertices,
                [&](vkrndr::gltf_vertex const& vert)
                {
                    color += 1.0f / part.vertex_count;
                    return vertex{.position = glm::fvec4(vert.position, 0.0f),
                        .color = glm::fvec3{color, color, color}};
                }).out;

            indices =
                std::ranges::copy(mesh->primitives[0].indices, indices).out;

            auto const gltf_to_bt = [](vkrndr::gltf_vertex const& vert) {
                return btVector3{vert.position.x,
                    vert.position.y,
                    vert.position.z};
            };

            collision_meshes.push_back(
                std::make_unique<btTriangleMesh>(true, false));

            for (uint32_t i{}; i != part.index_count; i += 3)
            {
                collision_meshes.back()->addTriangle(
                    gltf_to_bt(gltf_vertices[gltf_indices[i]]),
                    gltf_to_bt(gltf_vertices[gltf_indices[i + 1]]),
                    gltf_to_bt(gltf_vertices[gltf_indices[i + 2]]));
            }
        }

        unmap_memory(device, &vert_index_map);
    }

    model_mesh_ = create_buffer(device,
        staging_buffer.size,
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    renderer->transfer_buffer(staging_buffer, model_mesh_);
    destroy(device, &staging_buffer);

    for (size_t i{}; i != load_meshes.size(); ++i)
    {
        auto const& [part, transform] = parts[i];

        auto const entity{registry_.create()};

        gpu_mesh mesh{.vert_index_buffer = model_mesh_,
            .vertex_offset = 0,
            .index_offset = vertices_size,
            .submeshes = {part}};

        registry_.emplace<mesh_component>(entity, mesh);

        btVector3 const origin{transform[3][0],
            10.0f, // transform[3][1],
            transform[3][2]};
        auto collision{std::make_unique<btGImpactMeshShape>(
            collision_meshes[i].release())};
        collision->updateBound();

        auto const& physics{registry_.emplace<physics_component>(entity,
            physics_simulation_.add_rigid_body(std::move(collision),
                1.0f,
                origin))};

        physics.rigid_body_->setUserIndex(part.vertex_offset);
    }
}
