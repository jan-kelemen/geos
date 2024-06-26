#include <camera.hpp>

#include <cppext_numeric.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>

#include <imgui.h>

#include <fmt/format.h>

geos::camera::camera(glm::fvec3 const& eye,
    uint32_t const width,
    uint32_t const height,
    float const fov,
    float const aspect_ratio,
    float const near_plane,
    float const far_plane,
    float const yaw,
    float const pitch)
    : eye_{eye}
    , width_{width}
    , height_{height}
    , fov_{fov}
    , aspect_ratio_{aspect_ratio}
    , near_plane_{near_plane}
    , far_plane_{far_plane}
    , yaw_{yaw}
    , pitch_{pitch}
{
    update();
}

void geos::camera::update()
{
    constexpr glm::fvec3 world_up{0.0f, -1.0f, 0.0f};

    glm::vec3 front;
    front.x = cos(glm::radians(yaw_)) * cos(glm::radians(pitch_));
    front.y = sin(glm::radians(pitch_));
    front.z = sin(glm::radians(yaw_)) * cos(glm::radians(pitch_));
    front_ = glm::normalize(front);

    right_ = glm::normalize(glm::cross(front_, world_up));
    up_ = glm::normalize(glm::cross(right_, front_));

    view_matrix_ = calculate_view_matrix();
    projection_matrix_ = calculate_projection_matrix();
    view_projection_matrix_ = calculate_view_projection_matrix();
}

void geos::camera::mouse_movement(float const relative_x,
    float const relative_y)
{
    yaw_ += relative_x;
    pitch_ += relative_y;
}

std::pair<glm::fvec3, glm::fvec3> geos::camera::raycast_center() const
{
    return raycast(width_ / 2, height_ / 2);
}

std::pair<glm::fvec3, glm::fvec3> geos::camera::raycast(
    uint32_t const x_position,
    uint32_t const y_position) const
{
    glm::fvec3 const window{cppext::as_fp(x_position),
        cppext::as_fp(y_position),
        0};

    glm::fvec4 const viewport{0, 0, width_, height_};

    auto const near{glm::unProjectZO(glm::fvec3{window.x, window.y, 0},
        view_matrix_,
        projection_matrix_,
        viewport)};

    auto const far{glm::unProjectZO(glm::fvec3{window.x, window.y, 1},
        view_matrix_,
        projection_matrix_,
        viewport)};

    return std::make_pair(near, far);
}

void geos::camera::resize(uint32_t width, uint32_t height)
{
    width_ = width;
    height_ = height;
    aspect_ratio_ = cppext::as_fp(width) / cppext::as_fp(height);
}

void geos::camera::debug()
{
    ImGui::Begin("Camera");
    ImGui::SliderFloat3("Eye", glm::value_ptr(eye_), -10.f, 10.f);
    ImGui::SliderFloat("FOV", &fov_, 0.0f, 360.0f);
    ImGui::SliderFloat("Aspect ratio", &aspect_ratio_, 0.0f, 2.0f);
    ImGui::SliderFloat("Near plane", &near_plane_, -10.f, 100.f);
    ImGui::SliderFloat("Far plane", &far_plane_, -10.f, 100.f);
    ImGui::End();
}

glm::fmat4 geos::camera::calculate_view_matrix() const
{
    return glm::lookAtRH(eye_, eye_ + front_, up_);
}

glm::fmat4 geos::camera::calculate_projection_matrix() const
{
    return glm::perspectiveRH_ZO(glm::radians(fov_),
        aspect_ratio_,
        near_plane_,
        far_plane_);
}

glm::fmat4 geos::camera::calculate_view_projection_matrix() const
{
    return view_matrix_ * projection_matrix_;
}
