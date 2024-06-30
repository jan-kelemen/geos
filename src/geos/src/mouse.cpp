#include <mouse.hpp>

#include <camera.hpp>
#include <physics.hpp>

#include <cppext_numeric.hpp>

#include <memory>

geos::mouse::mouse(camera* camera, physics_simulation* physics_simulation)
    : camera_{camera}
    , physics_simulation_{physics_simulation}
{
    set_capture(capture_);
}

void geos::mouse::set_capture(bool value)
{
    capture_ = value;
    if (capture_)
    {
        SDL_SetRelativeMouseMode(SDL_TRUE);
        SDL_GetRelativeMouseState(nullptr, nullptr);
    }
    else
    {
        SDL_SetRelativeMouseMode(SDL_FALSE);
    }
}

bool geos::mouse::handle_event(SDL_Event const& event)
{
    if (event.type == SDL_MOUSEBUTTONDOWN)
    {
        auto const& [near, far] = cast_to_world();

        auto [body, point] =
            physics_simulation_->raycast({near.x, near.y, near.z},
                {far.x, far.y, far.z});
        if (!body)
        {
            return false;
        }

        if (auto rigid_body{
                btRigidBody::upcast(const_cast<btCollisionObject*>(body))})
        {
            if (rigid_body->isStaticObject() || rigid_body->isKinematicObject())
            {
                return false;
            }

            picked_body_ = rigid_body;
            picked_body_->setActivationState(DISABLE_DEACTIVATION);

            auto const local_pivot{
                rigid_body->getCenterOfMassTransform().inverse() * point};
            pick_constraint_ =
                std::make_unique<btPoint2PointConstraint>(*rigid_body,
                    local_pivot);
            physics_simulation_->add_constraint(pick_constraint_.get());
            pick_constraint_->m_setting.m_impulseClamp = 100.0f;
            pick_constraint_->m_setting.m_tau = 0.001f;
            pick_distance_ =
                (point - btVector3{near.x, near.y, near.z}).length();

            return true;
        }
    }
    else if (event.type == SDL_MOUSEMOTION || event.type == SDL_MOUSEWHEEL)
    {
        if (!picked_body_ || !pick_constraint_)
        {
            return false;
        }

        if (event.type == SDL_MOUSEWHEEL)
        {
            auto const& wheel{event.wheel};
            pick_distance_ += wheel.preciseY;
        }

        auto const& [near, far] = cast_to_world();
        auto const direction{glm::normalize(far - near) * pick_distance_};
        auto const new_pivot{near + direction};
        pick_constraint_->setPivotB({new_pivot.x, new_pivot.y, new_pivot.z});

        return true;
    }
    else if (event.type == SDL_MOUSEBUTTONUP)
    {
        if (pick_constraint_)
        {
            picked_body_->forceActivationState(ACTIVE_TAG);
            picked_body_->activate();

            physics_simulation_->remove_constraint(pick_constraint_.get());

            pick_constraint_.reset();
            picked_body_ = nullptr;

            return true;
        }
    }

    return false;
}

std::pair<glm::fvec3, glm::fvec3> geos::mouse::cast_to_world() const
{
    if (capture_)
    {
        return camera_->raycast_center();
    }

    int x; // NOLINT
    int y; // NOLINT
    SDL_GetMouseState(&x, &y);
    return camera_->raycast(cppext::narrow<uint32_t>(x),
        cppext::narrow<uint32_t>(y));
}
