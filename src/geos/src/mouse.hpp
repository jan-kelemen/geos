#ifndef GEOS_MOUSE_INCLUDED
#define GEOS_MOUSE_INCLUDED

#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>

#include <glm/fwd.hpp>
#include <glm/vec3.hpp>

#include <SDL2/SDL_events.h>

#include <memory>
#include <utility>

namespace geos
{
    class camera;
    class physics_simulation;
} // namespace geos

namespace geos
{
    class [[nodiscard]] mouse final
    {
    public:
        mouse(camera* camera, physics_simulation* physics_simulation);

        mouse(mouse const&) = delete;

        mouse(mouse&&) noexcept = delete;

    public:
        ~mouse() = default;

    public:
        [[nodiscard]] bool capture() const { return capture_; }

        void toggle_capture() { set_capture(!capture_); }

        void set_capture(bool value);

        bool handle_event(SDL_Event const& event);

    public:
        mouse& operator=(mouse const&) = delete;

        mouse& operator=(mouse&&) noexcept = delete;

    private:
        std::pair<glm::fvec3, glm::fvec3> cast_to_world() const;

    private:
        camera* camera_;
        physics_simulation* physics_simulation_;

        bool capture_{false};

        btCollisionObject const* picked_body_{nullptr};
        std::unique_ptr<btPoint2PointConstraint> pick_constraint_{};
        btScalar pick_distance_{};
    };
} // namespace geos

#endif // GEOS_MOUSE_INCLUDED
