#include <camera.hpp>

#include <cppext_numeric.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>
#include <glm/vec3.hpp>

#include <imgui.h>

geos::camera::camera(glm::fvec3 const& eye,
    glm::fvec3 const& center,
    float const fov,
    float const aspect_ratio,
    float const near_plane,
    float const far_plane)
    : eye_{eye}
    , center_{center}
    , up_direction_{0.0f, 0.0f, -1.0f}
    , fov_{fov}
    , aspect_ratio_{aspect_ratio}
    , near_plane_{near_plane}
    , far_plane_{far_plane}
    , view_matrix_{calculate_view_matrix()}
    , projection_matrix_{calculate_projection_matrix()}
    , view_projection_matrix_{calculate_view_projection_matrix()}
{
}

void geos::camera::update()
{
    view_matrix_ = calculate_view_matrix();
    projection_matrix_ = calculate_projection_matrix();
    view_projection_matrix_ = calculate_view_projection_matrix();
}

void geos::camera::resize(uint32_t width, uint32_t height)
{
    aspect_ratio_ = cppext::as_fp(width) / cppext::as_fp(height);
}

void geos::camera::debug()
{
    ImGui::Begin("Camera");
    ImGui::SliderFloat3("Eye", glm::value_ptr(eye_), -10.f, 10.f);
    ImGui::SliderFloat3("Center", glm::value_ptr(center_), -10.f, 10.f);
    ImGui::SliderFloat("FOV", &fov_, 0.0f, 360.0f);
    ImGui::SliderFloat("Aspect ratio", &aspect_ratio_, 0.0f, 2.0f);
    ImGui::SliderFloat("Near plane", &near_plane_, -10.f, 100.f);
    ImGui::SliderFloat("Far plane", &far_plane_, -10.f, 100.f);
    ImGui::End();
}

glm::fmat4 geos::camera::calculate_view_matrix() const
{
    return glm::lookAt(eye_, center_, up_direction_);
}

glm::fmat4 geos::camera::calculate_projection_matrix() const
{
    return glm::perspective(glm::radians(fov_),
        aspect_ratio_,
        near_plane_,
        far_plane_);
}

glm::fmat4 geos::camera::calculate_view_projection_matrix() const
{
    return view_matrix_ * projection_matrix_;
}
