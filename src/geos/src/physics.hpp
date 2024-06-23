#ifndef GEOS_PHYSICS_INCLUDED
#define GEOS_PHYSICS_INCLUDED

#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btTransform.h>

#include <memory>

class btBroadphaseInterface;
class btCollisionDispatcher;
class btCollisionObject;
class btCollisionShape;
class btDefaultCollisionConfiguration;
class btDiscreteDynamicsWorld;
class btRigidBody;
class btSequentialImpulseConstraintSolver;

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
        btRigidBody* add_rigid_body(std::unique_ptr<btCollisionShape> shape,
            float mass,
            btVector3 const& origin);

        void update(float delta_time);

        [[nodiscard]] btCollisionObject const* raycast(btVector3 const& from,
            btVector3 const& to);

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
