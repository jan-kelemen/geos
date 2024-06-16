#include <application.hpp>

#include <cppext_numeric.hpp>

geos::application::application()
    : camera_{glm::fvec3(3.0f, 3.0f, 3.0f),
          glm::fvec3(0.0f, 0.0f, 0.0f),
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
    scene_.attach_renderer(device, renderer);
}

void geos::application::detach_renderer(vkrndr::vulkan_device* const device,
    vkrndr::vulkan_renderer* renderer)
{
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
    return scene_.resize(extent);
}

void geos::application::draw(VkCommandBuffer command_buffer, VkExtent2D extent)
{
    return scene_.draw(command_buffer, extent);
}

void geos::application::draw_imgui()
{
    camera_.debug();
    scene_.draw_imgui();
}
