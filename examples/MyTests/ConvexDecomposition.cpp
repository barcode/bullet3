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

#include "../Extras/ConvexDecomposition/ConvexDecomposition.h"
//#include "../Extras/ConvexDecomposition/ConvexBuilder.h"
#include "../Extras/ConvexDecomposition/cd_wavefront.h"

class ConvexDecompositionToBtCompoundShape : public ConvexDecomposition::ConvexDecompInterface
{
public:
    ConvexDecompositionToBtCompoundShape(btCompoundShape* c) : compound{c} {}

    void ConvexDecompResult(ConvexDecomposition::ConvexResult& result) override
    {
        ++i;
        b3Printf("ConvexDecompositionToBtCompoundShape::ConvexDecompResult %d", i);
        btConvexHullShape* colShape = new btConvexHullShape;

        for (unsigned int i = 0; i < result.mHullVcount; i++)
        {
            const auto* p = result.mHullVertices + i * 3;
            colShape->addPoint({p[0], p[1], p[2]});
        }

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(0, 0, 0));

        compound->addChildShape(transform, colShape);
    }
    btCompoundShape* compound;
    int i = 0;
};


class ConvexDecompositionToBtConvexHullShape : public ConvexDecomposition::ConvexDecompInterface
{
public:
    ConvexDecompositionToBtConvexHullShape(CommonRigidBodyBase* c) : container{c} {}

    void ConvexDecompResult(ConvexDecomposition::ConvexResult& result) override
    {
        ++i;
        b3Printf("ConvexDecompositionToBtConvexHullShape::ConvexDecompResult %d", i);
        btConvexHullShape* colShape = new btConvexHullShape;

        for (unsigned int i = 0; i < result.mHullVcount; i++)
        {
            const auto* p = result.mHullVertices + i * 3;
            colShape->addPoint({p[0], p[1], p[2]});
        }

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(xOffset, i * 3, 0));
                
        {
            btScalar mass(1.f);
    
            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = (mass != 0.f);
            btVector3 localInertia(0, 0, 0);
            if (isDynamic)
            {
                colShape->calculateLocalInertia(mass, localInertia);
            }
        }
        
        container->createRigidBody(1.0, transform, colShape);

    }
    CommonRigidBodyBase* container;
    double xOffset = 2.5;
    int i = 0;
};

struct MT_ConvexDecompositionExample : public CommonRigidBodyBase
{
    MT_ConvexDecompositionExample(struct GUIHelperInterface* helper)
        : CommonRigidBodyBase(helper)
    {}
    void initPhysics() override;
    void renderScene() override;
    void resetCamera() override
    {
        float dist = 41;
        float pitch = -35;
        float yaw = 52;
        float targetPos[3] = {0, 0.46, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
};

void MT_ConvexDecompositionExample::initPhysics()
{
    m_guiHelper->setUpAxis(1);

    createEmptyDynamicsWorld();

    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    if (m_dynamicsWorld->getDebugDrawer())
    {
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
    }

    btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
    m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -50, 0));
    {
        btScalar mass(0.);
        createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
    }

    //single box
    {
        //create a few dynamic rigidbodies
        // Re-using the same collision is better for memory usage and performance
        btBoxShape* colShape = createBoxShape(btVector3(1, 1, 1));

        m_collisionShapes.push_back(colShape);

        /// Create Dynamic Objects
        btTransform startTransform;
        startTransform.setIdentity();

        
        btScalar mass(1.f);
        {
    
            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = (mass != 0.f);
            btVector3 localInertia(0, 0, 0);
            if (isDynamic)
            {
                colShape->calculateLocalInertia(mass, localInertia);
            }
        }

        startTransform.setOrigin(btVector3(
                                     btScalar(0),
                                     btScalar(10),
                                     btScalar(0)));
        createRigidBody(mass, startTransform, colShape);
    }

    //compounds
    {
        const auto add = [&](auto offset, const std::string & fname)
        {
            btCompoundShape* compoundShape = new btCompoundShape();

            {
                ConvexDecomposition::WavefrontObj wo;
                wo.loadObj(fname.c_str());


                ConvexDecomposition::DecompDesc d;
                d.mVcount       =   wo.mVertexCount;
                d.mVertices     = wo.mVertices;
                d.mTcount       = wo.mTriCount;
                d.mIndices      = (unsigned int*)wo.mIndices;
                d.mDepth        = 5;
                d.mCpercent     = 1;
                d.mPpercent     = 1;
                d.mMaxVertices  = 256;
                d.mSkinWidth    = 0;

                {
                    ConvexDecompositionToBtCompoundShape callback{compoundShape};
                    d.mCallback     = &callback;
                    //ConvexDecomposition::ConvexBuilder cb(d.mCallback);
                    //cb.process(d);
                    ConvexDecomposition::performConvexDecomposition(d);
                }

                {
                    ConvexDecompositionToBtConvexHullShape callback{this};
                    callback.xOffset = offset;
                    d.mCallback     = &callback;
                    //ConvexDecomposition::ConvexBuilder cb(d.mCallback);
                    //cb.process(d);
                    ConvexDecomposition::performConvexDecomposition(d);
                }

            }

            btScalar masses[3] = {1, 1, 1};
            btTransform principal;
            btVector3 inertia;
            compoundShape->calculatePrincipalAxisTransform(masses, principal, inertia);

            // new compund shape to store
            btCompoundShape* compound2 = new btCompoundShape();
            m_collisionShapes.push_back(compound2);

            // recompute the shift to make sure the compound shape is re-aligned
            for (int i = 0; i < compoundShape->getNumChildShapes(); i++)
                compound2->addChildShape(compoundShape->getChildTransform(i) * principal.inverse(),
                                         compoundShape->getChildShape(i));

            delete compoundShape;

            //other setup
            btTransform transform;
            transform.setIdentity();
            transform.setOrigin(btVector3(0, offset, 0));
            createRigidBody(1.0, transform, compound2);
        };

        add(5.0, "/home/raphael/repos/bullet3/worktree2/models/saladbowl.obj");
        add(10.0, "/home/raphael/repos/bullet3/worktree2/models/court_part.obj");
    }

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void MT_ConvexDecompositionExample::renderScene()
{
    CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* MT_ConvexDecomposition(CommonExampleOptions& options)
{
    return new MT_ConvexDecompositionExample(options.m_guiHelper);
}
