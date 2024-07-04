#include <application.hpp>

#include <camera.hpp>
#include <mesh.hpp>
#include <mouse.hpp>
#include <physics.hpp>
#include <scene.hpp>

#include <cppext_numeric.hpp>

#include <gltf_manager.hpp>
#include <vulkan_buffer.hpp>
#include <vulkan_memory.hpp>
#include <vulkan_renderer.hpp>
#include <vulkan_utility.hpp>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>

#include <glm/fwd.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp> // IWYU pragma: keep
#include <glm/vec3.hpp> // IWYU pragma: keep

#include <SDL2/SDL_events.h>
#include <SDL2/SDL_mouse.h>
#include <SDL2/SDL_scancode.h>
#include <SDL2/SDL_video.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// IWYU pragma: no_include <filesystem>

namespace
{
    [[nodiscard]] std::unique_ptr<btCollisionShape> make_collision_shape(
        vkrndr::gltf_node const& node,
        vkrndr::gltf_primitive const& primitive)
    {
        auto const gltf_to_bt = [](vkrndr::gltf_vertex const& vert) {
            return btVector3{vert.position.x, vert.position.y, vert.position.z};
        };

        auto const& node_name{node.name};

        if (node_name == "Sorter" || node_name == "Arch")
        {
            auto mesh{std::make_unique<btTriangleMesh>(true, false)};

            auto const& gltf_vertices{primitive.vertices};
            auto const& gltf_indices{primitive.indices};

            for (size_t i{}; i != primitive.indices.size(); i += 3)
            {
                mesh->addTriangle(gltf_to_bt(gltf_vertices[gltf_indices[i]]),
                    gltf_to_bt(gltf_vertices[gltf_indices[i + 1]]),
                    gltf_to_bt(gltf_vertices[gltf_indices[i + 2]]));
            }

            auto shape{std::make_unique<btGImpactMeshShape>(mesh.release())};
            shape->updateBound();

            return shape;
        }

        if (node_name == "Triangle" || node_name == "Semicircle")
        {
            auto shape{std::make_unique<btConvexHullShape>()};

            auto const& gltf_vertices{primitive.vertices};
            for (uint32_t const index : primitive.indices)
            {
                shape->addPoint(gltf_to_bt(gltf_vertices[index]));
            }

            return shape;
        }

        if (node_name == "Cylinder")
        {
            if (node.axis_aligned_bounding_box)
            {
                glm::fvec3 extents{node.axis_aligned_bounding_box->max -
                    node.axis_aligned_bounding_box->min};
                extents /= 2;

                return std::make_unique<btCylinderShape>(
                    btVector3{extents.x, extents.y, extents.z});
            }
        }

        if (node_name == "Thin Rectangle" || node_name == "Rectangle" ||
            node_name == "Square")
        {
            if (node.axis_aligned_bounding_box)
            {
                glm::fvec3 extents{node.axis_aligned_bounding_box->max -
                    node.axis_aligned_bounding_box->min};
                extents /= 2;

                return std::make_unique<btBoxShape>(
                    btVector3{extents.x, extents.y, extents.z});
            }
        }

        return nullptr;
    }
} // namespace

geos::application::application(uint32_t const width,
    uint32_t const height,
    bool const capture_mouse)
    : camera_{glm::fvec3{0.0f, 50.0f, 0.0f},
          width,
          height,
          45.0f,
          1.0f,
          0.1f,
          100.f,
          -90.0f,
          -90.0f}
    , mouse_{&camera_, &physics_simulation_}
{
    mouse_.set_capture(capture_mouse);

    btTransform transform;
    transform.setIdentity();
    transform.setOrigin({0.0f, -1.0f, 0.0f});

    physics_simulation_.add_rigid_body(
        std::make_unique<btBoxShape>(btVector3{50.0f, 1.0f, 50.0f}),
        0.0f,
        transform);
}

geos::application::~application() = default;

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
    else if (event.type == SDL_KEYDOWN)
    {
        auto const& keyboard{event.key};
        if (keyboard.keysym.scancode == SDL_SCANCODE_F3)
        {
            mouse_.toggle_capture();
        }
    }

    mouse_.handle_event(event);
}

void geos::application::begin_frame() { scene_.begin_frame(); }

void geos::application::end_frame() { scene_.end_frame(); }

void geos::application::fixed_update([[maybe_unused]] float const delta_time)
{
    if (mouse_.capture())
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

    for (auto entity : registry_.view<physics_component>())
    {
        auto physics{registry_.get<physics_component>(entity)};
        physics_simulation_.remove_rigid_body(physics.rigid_body_);
    }

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

void geos::application::draw_imgui()
{
    camera_.debug();
    scene_.debug();
}

void geos::application::load_meshes(vkrndr::vulkan_device* const device,
    vkrndr::vulkan_renderer* const renderer)
{
    std::unique_ptr<vkrndr::gltf_model> model{
        renderer->load_model("geos.gltf")};

    std::vector<vkrndr::gltf_mesh const*> load_meshes;
    std::vector<std::tuple<buffer_part, glm::fmat4, btScalar, glm::fvec3>>
        parts;
    std::vector<std::unique_ptr<btCollisionShape>> collision_shapes;

    uint32_t vertex_count{};
    uint32_t index_count{};
    for (auto const& node : model->nodes)
    {
        auto const& model_mesh{node.mesh};
        load_meshes.push_back(&(*model_mesh));

        auto const& current_mesh{parts.emplace_back(
            buffer_part{cppext::narrow<int32_t>(vertex_count),
                vkrndr::count_cast(model_mesh->primitives[0].vertices.size()),
                cppext::narrow<int32_t>(index_count),
                vkrndr::count_cast(model_mesh->primitives[0].indices.size())},
            local_matrix(node),
            node.name != "Sorter" ? 100.0f : 0.0f,
            node.name != "Sorter" ? glm::fvec3{0.8f, 0.0f, 0.0f}
                                  : glm::fvec3{0.8f, 0.8f, 0.8f})};

        vertex_count += std::get<0>(current_mesh).vertex_count;
        index_count += std::get<0>(current_mesh).index_count;

        collision_shapes.push_back(
            make_collision_shape(node, model_mesh->primitives[0]));
    }

    size_t const vertices_size{vertex_count * sizeof(vertex)};
    size_t const indices_size{index_count * sizeof(uint32_t)};

    vkrndr::vulkan_buffer staging_buffer{vkrndr::create_buffer(device,
        vertices_size + indices_size,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)};
    {
        vkrndr::mapped_memory vert_index_map{
            vkrndr::map_memory(device, staging_buffer.allocation)};

        vertex* vertices{vert_index_map.as<vertex>(0)};
        uint32_t* indices{vert_index_map.as<uint32_t>(vertices_size)};

        for (size_t i{}; i != load_meshes.size(); ++i)
        {
            auto const& mesh{load_meshes[i]};
            auto const& part{std::get<0>(parts[i])};
            auto const& mass{std::get<2>(parts[i])};
            auto const& color{std::get<3>(parts[i])};

            auto const& gltf_vertices{mesh->primitives[0].vertices};

            float color_offset{-1.0f / cppext::as_fp(part.vertex_count)};
            vertices = std::ranges::transform(gltf_vertices,
                vertices,
                [&](vkrndr::gltf_vertex const& vert)
                {
                    color_offset += 1.0f / cppext::as_fp(part.vertex_count);
                    return vertex{.position = glm::fvec4{vert.position, 0.0f},
                        .color = mass == 0.0f
                            ? color
                            : color * glm::fvec3{color_offset},
                        .normal = vert.normal};
                }).out;

            indices =
                std::ranges::copy(mesh->primitives[0].indices, indices).out;
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
        auto const& [part, transform, mass, color] = parts[i];

        auto const entity{registry_.create()};

        gpu_mesh mesh{.vert_index_buffer = model_mesh_,
            .vertex_offset = 0,
            .index_offset = vertices_size,
            .submeshes = {part}};

        registry_.emplace<mesh_component>(entity, mesh);

        btTransform physics_transform;
        physics_transform.setFromOpenGLMatrix(glm::value_ptr(transform));

        if (mass != 0.0f) // Move nonstatic bodies up
        {
            btVector3 const& origin{physics_transform.getOrigin()};
            physics_transform.setOrigin({origin.x(), 100.0f, origin.y()});
        }

        auto const& physics{registry_.emplace<physics_component>(entity,
            physics_simulation_.add_rigid_body(std::move(collision_shapes[i]),
                mass,
                physics_transform),
            physics_transform)};

        physics.rigid_body_->setUserIndex(part.vertex_offset);
    }
}
