#ifndef GEOS_CAMERA_INCLUDED
#define GEOS_CAMERA_INCLUDED

#include <glm/fwd.hpp>
#include <glm/mat4x4.hpp> // IWYU pragma: keep
#include <glm/vec3.hpp> // IWYU pragma: keep

#include <cstdint>
#include <utility>

namespace geos
{
    class [[nodiscard]] camera final
    {
    public:
        camera(glm::fvec3 const& eye,
            uint32_t width,
            uint32_t height,
            float fov,
            float aspect_ratio,
            float near_plane,
            float far_plane,
            float yaw,
            float pitch);

        camera(camera const&) = default;

        camera(camera&&) noexcept = default;

    public:
        ~camera() = default;

    public:
        [[nodiscard]] float fov() const { return fov_; }

        [[nodiscard]] float aspect_ratio() const { return aspect_ratio_; }

        [[nodiscard]] glm::fvec3 const& position() const { return eye_; }

        [[nodiscard]] glm::fmat4 const& view_matrix() const
        {
            return view_matrix_;
        }

        [[nodiscard]] glm::fmat4 const& projection_matrix() const
        {
            return projection_matrix_;
        }

        [[nodiscard]] glm::fmat4 const& view_projection_matrix() const
        {
            return view_projection_matrix_;
        }

    public:
        void update();

        void mouse_movement(float relative_x, float relative_y);

        [[nodiscard]] std::pair<glm::fvec3, glm::fvec3> raycast_center() const;

        [[nodiscard]] std::pair<glm::fvec3, glm::fvec3>
        raycast(uint32_t x_position, uint32_t y_position) const;

        void resize(uint32_t width, uint32_t height);

        void debug();

    public:
        camera& operator=(camera const&) = default;

        camera& operator=(camera&&) noexcept = default;

    private:
        [[nodiscard]] glm::fmat4 calculate_view_matrix() const;

        [[nodiscard]] glm::fmat4 calculate_projection_matrix() const;

        [[nodiscard]] glm::fmat4 calculate_view_projection_matrix() const;

    private:
        glm::fvec3 eye_;

        glm::fvec3 right_;
        glm::fvec3 front_;
        glm::fvec3 up_;

        uint32_t width_;
        uint32_t height_;

        float fov_;
        float aspect_ratio_;
        float near_plane_;
        float far_plane_;

        float yaw_;
        float pitch_;

        float mouse_sensitivity_{0.15f};

        glm::fmat4 view_matrix_;
        glm::fmat4 projection_matrix_;
        glm::fmat4 view_projection_matrix_;
    };
} // namespace geos

#endif
