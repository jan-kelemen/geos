#include <physics.hpp>

#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btAlignedAllocator.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btMotionState.h>
#include <LinearMath/btVector3.h>

geos::physics_simulation::physics_simulation()
    : collision_configuration_{std::make_unique<
          btDefaultCollisionConfiguration>()}
    , dispatcher_{std::make_unique<btCollisionDispatcher>(
          collision_configuration_.get())}
    , overlapping_pair_cache_{std::make_unique<btDbvtBroadphase>()}
    , solver_{std::make_unique<btSequentialImpulseConstraintSolver>()}
    , world_{std::make_unique<btDiscreteDynamicsWorld>(dispatcher_.get(),
          overlapping_pair_cache_.get(),
          solver_.get(),
          collision_configuration_.get())}
{
    world_->setGravity(btVector3(0, -10, 0));

    // TODO: TEMPORARY
    {
        btCollisionShape* ground_shape{new btBoxShape{
            btVector3{btScalar{50.0f}, btScalar{50.0f}, btScalar{50.0f}}}};
        collision_shapes_.push_back(ground_shape);

        btTransform ground_transform;
        ground_transform.setIdentity();
        ground_transform.setOrigin(btVector3{0, -56, 0});

        btScalar const mass{0.0f};

        btVector3 local_inertia{0, 0, 0};

        // NOLINTBEGIN(cppcoreguidelines-owning-memory)
        btDefaultMotionState* const motion_state{
            new btDefaultMotionState{ground_transform}};
        btRigidBody::btRigidBodyConstructionInfo const rigid_body_info{mass,
            motion_state,
            ground_shape,
            local_inertia};
        btRigidBody* const body{new btRigidBody{rigid_body_info}};
        body->setUserIndex(1);
        // NOLINTEND(cppcoreguidelines-owning-memory)

        // add the body to the dynamics world
        world_->addRigidBody(body);
    }
}

geos::physics_simulation::~physics_simulation()
{
    // remove the rigidbodies from the dynamics world and delete them
    for (int i{world_->getNumCollisionObjects() - 1}; i >= 0; --i)
    {
        btCollisionObject* obj{world_->getCollisionObjectArray()[i]};
        btRigidBody* body{btRigidBody::upcast(obj)};
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        world_->removeCollisionObject(obj);
        delete obj;
    }

    // delete collision shapes
    for (int j{}; j < collision_shapes_.size(); ++j)
    {
        btCollisionShape* shape{collision_shapes_[j]};
        collision_shapes_[j] = 0;
        delete shape;
    }
}

btRigidBody* geos::physics_simulation::add_rigid_body(
    std::unique_ptr<btCollisionShape> shape,
    float const mass,
    btVector3 const& origin)
{
    btVector3 local_inertia{0, 0, 0};
    if (mass != 0.0f)
    {
        shape->calculateLocalInertia(mass, local_inertia);
    }
    collision_shapes_.push_back(shape.release());

    btTransform start_transform;
    start_transform.setIdentity();
    start_transform.setOrigin(origin);

    // NOLINTBEGIN(cppcoreguidelines-owning-memory)
    btDefaultMotionState* const motion_state{
        new btDefaultMotionState{start_transform}};
    btRigidBody::btRigidBodyConstructionInfo const rigid_body_info{mass,
        motion_state,
        collision_shapes_[collision_shapes_.size() - 1],
        local_inertia};
    btRigidBody* const body{new btRigidBody{rigid_body_info}};
    body->setUserIndex(2);
    // NOLINTEND(cppcoreguidelines-owning-memory)

    // add the body to the dynamics world
    world_->addRigidBody(body);

    return body;
}

void geos::physics_simulation::update(float const delta_time)
{
    world_->stepSimulation(delta_time, 10);
}

btCollisionObject const*
geos::physics_simulation::raycast(btVector3 const& from, btVector3 const& to)
{
    btCollisionWorld::ClosestRayResultCallback callback{from, to};
    world_->rayTest(from, to, callback);
    if (callback.hasHit())
    {
        return callback.m_collisionObject;
    }

    return nullptr;
}

btTransform geos::physics_component::position() const
{
    btTransform rv;

    if (rigid_body_ && rigid_body_->getMotionState())
    {
        rigid_body_->getMotionState()->getWorldTransform(rv);
    }
    else
    {
        rv = rigid_body_->getWorldTransform();
    }

    return rv;
}
