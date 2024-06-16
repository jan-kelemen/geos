#include <camera.hpp>

#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>

geos::camera::camera(glm::fvec3 const& eye,
    glm::fvec3 const& center,
    float fov,
    float aspect_ratio,
    float near_plane,
    float far_plane)
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

void geos::camera::debug()
{
    ImGui::Begin("Camera");
    ImGui::SliderFloat3("Eye", glm::value_ptr(eye_), -10.f, 10.f);
    ImGui::SliderFloat3("Center", glm::value_ptr(center_), -10.f, 10.f);
    ImGui::SliderAngle("FOV", &fov_);
    ImGui::SliderFloat("Aspect ratio", &aspect_ratio_, 0.0f, 2.0f);
    ImGui::SliderFloat("Near plane", &near_plane_, -10.f, 100.f);
    ImGui::SliderFloat("Far plane", &far_plane_, -10.f, 100.f);
    ImGui::End();
}

glm::fmat4 geos::camera::calculate_view_matrix()
{
    return glm::lookAt(eye_, center_, up_direction_);
}

glm::fmat4 geos::camera::calculate_projection_matrix()
{
    return glm::perspective(glm::radians(fov_),
        aspect_ratio_,
        near_plane_,
        far_plane_);
}

glm::fmat4 geos::camera::calculate_view_projection_matrix()
{
    return view_matrix_ * projection_matrix_;
}
