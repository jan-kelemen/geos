#include <physics.hpp>

#include <cstdio>

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
        btCollisionShape* groundShape = new btBoxShape(
            btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

        collision_shapes_.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -56, 0));

        btScalar mass(0.);

        // rigidbody is dynamic if and only if mass is non zero, otherwise
        // static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass, localInertia);

        // using motionstate is optional, it provides interpolation
        // capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState =
            new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,
            myMotionState,
            groundShape,
            localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        // add the body to the dynamics world
        world_->addRigidBody(body);
    }
}

geos::physics_simulation::~physics_simulation()
{
    // remove the rigidbodies from the dynamics world and delete them
    for (int i = world_->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = world_->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        world_->removeCollisionObject(obj);
        delete obj;
    }

    // delete collision shapes
    for (int j = 0; j < collision_shapes_.size(); j++)
    {
        btCollisionShape* shape = collision_shapes_[j];
        collision_shapes_[j] = 0;
        delete shape;
    }
}

btRigidBody* geos::physics_simulation::add_rigid_body()
{
    // create a dynamic rigidbody
    btCollisionShape* colShape = new btBoxShape(btVector3(1, 1, 1));
    collision_shapes_.push_back(colShape);

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    btScalar mass(1.f);

    // rigidbody is dynamic if and only if mass is non zero, otherwise
    // static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        colShape->calculateLocalInertia(mass, localInertia);

    startTransform.setOrigin(btVector3(2, 10, 0));

    // using motionstate is recommended, it provides interpolation
    // capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState =
        new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,
        myMotionState,
        colShape,
        localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    world_->addRigidBody(body);

    return body;
}

void geos::physics_simulation::update(float delta_time)
{
    world_->stepSimulation(delta_time, 10);
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
