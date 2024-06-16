#ifndef GEOS_CAMERA_INCLUDED
#define GEOS_CAMERA_INCLUDED

#include <glm/fwd.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace geos
{
    class [[nodiscard]] camera final
    {
    public:
        camera(glm::fvec3 const& eye,
            glm::fvec3 const& center,
            float fov,
            float aspect_ratio,
            float near_plane,
            float far_plane);

        camera(camera const&) = default;

        camera(camera&&) noexcept = default;

    public:
        ~camera() = default;

    public:
        [[nodiscard]] float fov() const { return fov_; }

        [[nodiscard]] float aspect_ratio() const { return aspect_ratio_; }

        [[nodiscard]] glm::fmat4 view_matrix() const { return view_matrix_; }

        [[nodiscard]] glm::fmat4 projection_matrix() const
        {
            return projection_matrix_;
        }

        [[nodiscard]] glm::fmat4 view_projection_matrix() const
        {
            return view_projection_matrix_;
        }

    public:
        void update();

        void debug();

    public:
        camera& operator=(camera const&) = default;

        camera& operator=(camera&&) noexcept = default;

    private:
        glm::fmat4 calculate_view_matrix();

        glm::fmat4 calculate_projection_matrix();

        glm::fmat4 calculate_view_projection_matrix();

    private:
        glm::fvec3 eye_;
        glm::fvec3 center_;
        glm::fvec3 up_direction_;

        float fov_;
        float aspect_ratio_;
        float near_plane_;
        float far_plane_;

        glm::fmat4 view_matrix_;
        glm::fmat4 projection_matrix_;
        glm::fmat4 view_projection_matrix_;
    };
} // namespace geos

#endif
