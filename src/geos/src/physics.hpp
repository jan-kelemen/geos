#ifndef GEOS_PHYSICS_INCLUDED
#define GEOS_PHYSICS_INCLUDED

#include <btBulletDynamicsCommon.h>

#include <memory>
#include <vector>

namespace geos
{
    class [[nodiscard]] physics_simulation final
    {
    public:
        physics_simulation();

        physics_simulation(physics_simulation const&) = delete;

        physics_simulation(physics_simulation&&) noexcept = delete;

    public:
        ~physics_simulation();

    public:
        btRigidBody* add_rigid_body();

        void update(float delta_time);

    public:
        physics_simulation& operator=(physics_simulation const&) = delete;

        physics_simulation& operator=(physics_simulation&&) noexcept = delete;

    private:
        std::unique_ptr<btDefaultCollisionConfiguration>
            collision_configuration_;
        std::unique_ptr<btCollisionDispatcher> dispatcher_;
        std::unique_ptr<btBroadphaseInterface> overlapping_pair_cache_;
        std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;
        std::unique_ptr<btDiscreteDynamicsWorld> world_;
        btAlignedObjectArray<btCollisionShape*> collision_shapes_;
    };

    struct [[nodiscard]] physics_component final
    {
        btRigidBody* rigid_body_;

        [[nodiscard]] btTransform position() const;
    };
} // namespace geos

#endif
