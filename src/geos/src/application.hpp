#ifndef GEOS_APPLICATION_INCLUDED
#define GEOS_APPLICATION_INCLUDED

#include <camera.hpp>
#include <physics.hpp>
#include <scene.hpp>

#include <vulkan_scene.hpp>

#include <entt/entt.hpp>

#include <SDL2/SDL_events.h>

#include <memory>

namespace vkrndr
{
    struct vulkan_device;
    class vulkan_renderer;
    class vulkan_scene;
} // namespace vkrndr

namespace geos
{
    class [[nodiscard]] application final : public vkrndr::vulkan_scene
    {
    public:
        application();

        application(application const&) = delete;

        application(application&&) noexcept = delete;

    public:
        ~application() override;

    public:
        void handle_event(SDL_Event const& event);

        void begin_frame();

        void end_frame();

        void update(float delta_time);

    public:
        void attach_renderer(vkrndr::vulkan_device* device,
            vkrndr::vulkan_renderer* renderer);

        void detach_renderer(vkrndr::vulkan_device* device,
            vkrndr::vulkan_renderer* renderer);

    public: // vulkan_scene overrides
        [[nodiscard]] VkClearValue clear_color() override;

        [[nodiscard]] VkClearValue clear_depth() override;

        [[nodiscard]] vkrndr::vulkan_image* depth_image() override;

        void resize(VkExtent2D extent) override;

        void draw(VkCommandBuffer command_buffer, VkExtent2D extent) override;

        void draw_imgui() override;

    public:
        application& operator=(application const&) = delete;

        application& operator=(application&&) = delete;

    private:
        void load_meshes(vkrndr::vulkan_device* device,
            vkrndr::vulkan_renderer* renderer);

    private:
        entt::registry registry_;

        physics_simulation physics_simulation_;

        camera camera_;
        scene scene_;
    };
} // namespace geos

#endif
