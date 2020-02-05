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

#include "CompoundShapeCollision.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct MT_CompoundShapeCollisionxExample : public CommonRigidBodyBase
{
	MT_CompoundShapeCollisionxExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~MT_CompoundShapeCollisionxExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 41;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0.46, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void MT_CompoundShapeCollisionxExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

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

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(0),
			btScalar(10),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShape);
    }
    
    //compound box
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btBoxShape* colShape = createBoxShape(btVector3(2, 1, 1));
        
        
        btCompoundShape* compoundShape = new btCompoundShape();
        
        btTransform transform;
        //b1
        {
            transform.setIdentity();
            transform.setOrigin(btVector3(0, 0, 0));
            compoundShape->addChildShape(transform, colShape);
        }
        
        //b2
        {
            transform.setIdentity();
            transform.setOrigin(btVector3(3.5, -1, 0));
            compoundShape->addChildShape(transform, colShape);
        }
        
        //b3
        {
            transform.setIdentity();
            transform.setOrigin(btVector3(-3.5, -1, 0));
            compoundShape->addChildShape(transform, colShape);
        }
        
        //other setup
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

		transform.setIdentity();
		transform.setOrigin(btVector3(0, 20, 0));
		createRigidBody(1.0, transform, compound2);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void MT_CompoundShapeCollisionxExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* MT_CompoundShapeCollision(CommonExampleOptions& options)
{
	return new MT_CompoundShapeCollisionxExample(options.m_guiHelper);
}
