/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <random>

#include "ManualMesh.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct MT_ManualMeshxExample : public CommonRigidBodyBase
{
    MT_ManualMeshxExample(struct GUIHelperInterface* helper)
        : CommonRigidBodyBase(helper)
    {
    }
    ~MT_ManualMeshxExample() override = default;
    void initPhysics() override;
    void renderScene() override;
    void resetCamera()
    {
        float dist = 41;
        float pitch = -35;
        float yaw = 52;
        float targetPos[3] = {0, 0.46, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }

    void stepSimulation(float deltaTime) override
    {
        CommonRigidBodyBase::stepSimulation(deltaTime);

        const btTransform tr = m_lastBody->getCenterOfMassTransform();
        const btVector3& o = tr.getOrigin();
        const btMatrix3x3& r = tr.getBasis();
        b3Printf("dt = %f", deltaTime);
        b3Printf(
            "pose of last obj (r0): %+.3f  %+.3f  %+.3f  %+.3f",
            r.getRow(0).x(), r.getRow(0).y(), r.getRow(0).z(), o.x()
        );
        b3Printf(
            "pose of last obj (r1): %+.3f  %+.3f  %+.3f  %+.3f",
            r.getRow(1).x(), r.getRow(1).y(), r.getRow(1).z(), o.y()
        );
        b3Printf(
            "pose of last obj (r2): %+.3f  %+.3f  %+.3f  %+.3f",
            r.getRow(2).x(), r.getRow(2).y(), r.getRow(2).z(),  o.z()
        );
    }

    btRigidBody* m_lastBody = nullptr;
};

void MT_ManualMeshxExample::initPhysics()
{
    m_guiHelper->setUpAxis(1);

    createEmptyDynamicsWorld();

    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    if (m_dynamicsWorld->getDebugDrawer())
    {
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
    }

    ///create a few basic rigid bodies
    btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
    m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -50, 0));
    {
        btScalar mass(0.);
        createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
    }

    std::mt19937 g{std::random_device{}()};
    std::uniform_real_distribution<float> d{-1, 1};

    for (int shapeidx = 0; shapeidx < 10; ++ shapeidx)
    {
        //create a few dynamic rigidbodies
        // Re-using the same collision is better for memory usage and performance
        btConvexHullShape* colShape = new btConvexHullShape;

        for (int i = 0; i < 10; ++i)
        {
            colShape->addPoint({d(g), d(g), d(g)});
        }

        m_collisionShapes.push_back(colShape);

        /// Create Dynamic Objects
        btTransform startTransform;
        startTransform.setIdentity();

        btScalar mass(1.f);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
        {
            colShape->calculateLocalInertia(mass, localInertia);
        }

        startTransform.setOrigin(btVector3(
                                     btScalar(0),
                                     btScalar(3 * (shapeidx + 1)),
                                     btScalar(0)));
        m_lastBody = createRigidBody(mass, startTransform, colShape);
    }

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void MT_ManualMeshxExample::renderScene()
{
    CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* MT_ManualMesh(CommonExampleOptions& options)
{
    return new MT_ManualMeshxExample(options.m_guiHelper);
}
