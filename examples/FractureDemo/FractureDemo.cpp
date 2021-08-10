/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2011 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///FractureDemo shows how to break objects.
///It assumes a btCompoundShaps (where the childshapes are the pre-fractured pieces)
///The btFractureBody is a class derived from btRigidBody, dealing with the collision impacts.
///Press the F key to toggle between fracture and glue mode
///This is preliminary work

#define CUBE_HALF_EXTENTS 1.f
#define EXTRA_HEIGHT 1.f
///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "FractureDemo.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h>  //printf debugging

int sFrameNumber = 0;

#include "btFractureBody.h"
#include "btFractureDynamicsWorld.h"

#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"


#include <vector>
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../Utils/b3BulletDefaultFileIO.h"


///FractureDemo shows basic breaking and glueing of objects
class FractureDemo : public CommonRigidBodyBase
{
public:
	FractureDemo(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~FractureDemo()
	{
	}
	void initPhysics();

	void exitPhysics();

	// helpers
	btCompoundShape* LoadHACDOBJ(const char* objFileName);

	btBvhTriangleMeshShape* LoadStaticConcaveObj(const char* objFileName);



	virtual void stepSimulation(float deltaTime)
	{
		CommonRigidBodyBase::stepSimulation(deltaTime);

		{
			BT_PROFILE("recreate graphics");
			//@todo: make this graphics re-creation better
			//right now: brute force remove all graphics objects, and re-create them every frame
			m_guiHelper->getRenderInterface()->removeAllInstances();
			for (int i = 0; i < m_dynamicsWorld->getNumCollisionObjects(); i++)
			{
				btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
				colObj->getCollisionShape()->setUserIndex(-1);
				colObj->setUserIndex(-1);
			}
			m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
		}
	}

	virtual bool keyboardCallback(int key, int state);

	void resetCamera()
	{
		float dist = 41;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0.46, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void FractureDemo::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	//m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	btFractureDynamicsWorld* fractureWorld = new btFractureDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld = fractureWorld;

	m_dynamicsWorld->setGravity(btVector3{ 2.0, -1.0, 0.0 });

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	//m_splitImpulse removes the penetration resolution from the applied impulse, otherwise objects might fracture due to deep penetrations.
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;

	{
		///create a few basic rigid bodies
		//btCollisionShape* groundShape = new btBoxShape(btVector3(500, 1, 500));
		btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),0);
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, 0, 0));
		createRigidBody(0.f, groundTransform, groundShape);
	}
	{
		///create a few basic rigid bodies
		//btCollisionShape* groundShape = new btBoxShape(btVector3(500, 1, 500));
		btCollisionShape* sphereShape = new btCylinderShape({ 5.0, 5.0, 2.0});
		
		m_collisionShapes.push_back(sphereShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, 10, 30));
		groundTransform.setRotation(btQuaternion{ { 1.0, 0.0, 0.0 }, 3.14159 / 2 });
		//createRigidBody(0.f, groundTransform, sphereShape);
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		btRigidBody* sphereBody = createRigidBody(5.0f, groundTransform, sphereShape, btVector4{ 1, 1, 0, 1 });
		//btVector3 localInertia(0, 0, 0);
		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		//btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);

		//btRigidBody::btRigidBodyConstructionInfo cInfo(10.0f, myMotionState, sphereShape, localInertia);


		//btRigidBody* body = new btRigidBody(cInfo);
		//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

		//body->setUserIndex(-1);
		//m_dynamicsWorld->addRigidBody(body);

	}

	// add tooth #1
	{
		const char* compoundFileName = "D:/dev/bullet3/testdata/tooth_acd.obj";
		btCompoundShape* compoundShape = LoadHACDOBJ(compoundFileName);

		m_collisionShapes.push_back(compoundShape);
		btTransform compoundTransform;
		compoundTransform.setIdentity();
		compoundTransform.setOrigin(btVector3(-5, 0, 0));
		btScalar angle = 3.14159 / 2;
		//compoundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), angle));
		btRigidBody* tooth = createRigidBody(static_cast<btScalar>(compoundShape->getNumChildShapes() * 20), compoundTransform, compoundShape);
	}

	// add tooth #2
	{
		const char* compoundFileName = "D:/dev/bullet3/testdata/tooth_acd.obj";
		btCompoundShape* compoundShape = LoadHACDOBJ(compoundFileName);

		m_collisionShapes.push_back(compoundShape);
		btTransform compoundTransform;
		compoundTransform.setIdentity();
		compoundTransform.setOrigin(btVector3(-25.305878, 5.395162, 0));
		btScalar angle = 3.14159 / 2;
		//compoundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), angle));
		btRigidBody* tooth = createRigidBody(0.f, compoundTransform, compoundShape);
	}

	// add walls
	{
		const char* curveWallFileName = "D:/dev/bullet3/testdata/curve_wall_thick.obj";
		auto* curveShape = LoadStaticConcaveObj(curveWallFileName);

		m_collisionShapes.push_back(curveShape);
		btTransform compoundTransform;
		compoundTransform.setIdentity();
		compoundTransform.setOrigin(btVector3(0, 0, 0));
		btScalar angle = 3.14159 / 2;
		//compoundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), angle));
		btRigidBody* curveWall = createRigidBody(0.0f, compoundTransform, curveShape);
	}


	{
		int gNumObjects = 0;


		for (int i = 0; i < gNumObjects; i++)
		{
			//create a few dynamic rigidbodies

			btCollisionShape* colShape = new btBoxShape(btVector3(SCALING * 1, SCALING * (i * 0.5 + 1), SCALING * 1));
			//btCollisionShape* colShape = new btCapsuleShape(SCALING*0.4,SCALING*1);
			//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
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


			btTransform trans;
			trans.setIdentity();

			btVector3 pos(i * 2 * CUBE_HALF_EXTENTS, 20, 0);
			trans.setOrigin(pos);
			
			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
			btFractureBody* body = new btFractureBody(rbInfo, m_dynamicsWorld);
			body->setLinearVelocity(btVector3(0, -5, 0));

			m_dynamicsWorld->addRigidBody(body);
		}
	}

	fractureWorld->stepSimulation(1. / 60., 0);
	fractureWorld->glueCallback();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	m_guiHelper->getAppInterface()->setMouseWheelMultiplier(0.2);
	m_guiHelper->getAppInterface()->setMouseMoveMultiplier(0.2);
	
}





#if 0
void FractureDemo::showMessage()
{
	if((getDebugMode() & btIDebugDraw::DBG_DrawText))
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];

		int lineWidth=380;
		int xStart = m_glutScreenWidth - lineWidth;
		int yStart = 20;

		btFractureDynamicsWorld* world = (btFractureDynamicsWorld*)m_dynamicsWorld;
		if (world->getFractureMode())
		{
			sprintf(buf,"Fracture mode");
		} else
		{
			sprintf(buf,"Glue mode");
		}
		GLDebugDrawString(xStart,yStart,buf);
		sprintf(buf,"f to toggle fracture/glue mode");		
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		sprintf(buf,"space to restart, mouse to pick/shoot");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);

		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}

}
#endif

#if 0
void FractureDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	showMessage();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}
#endif

bool FractureDemo::keyboardCallback(int key, int state)
{
	if (key == 'f' && (state == 0))
	{
		btFractureDynamicsWorld* world = (btFractureDynamicsWorld*)m_dynamicsWorld;
		world->setFractureMode(!world->getFractureMode());
		if (world->getFractureMode())
		{
			b3Printf("Fracturing mode");
		}
		else
		{
			b3Printf("Gluing mode");
		}
		return true;
	}

	return false;
}

#if 0
void FractureDemo::keyboardUpCallback(unsigned char key, int x, int y)
{
	if (key=='f')
	{
		btFractureDynamicsWorld* world = (btFractureDynamicsWorld*)m_dynamicsWorld;
		world->setFractureMode(!world->getFractureMode());
	}

	PlatformDemoApplication::keyboardUpCallback(key,x,y);

}
#endif

#if 0
void	FractureDemo::shootBox(const btVector3& destination)
{

	if (m_dynamicsWorld)
	{
		btScalar mass = 1.f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);

		setShootBoxShape ();

		btAssert((!m_shootBoxShape || m_shootBoxShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			m_shootBoxShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btFractureBody* body = new btFractureBody(mass,0,m_shootBoxShape,localInertia,&mass,1,m_dynamicsWorld);

		body->setWorldTransform(startTransform);

		m_dynamicsWorld->addRigidBody(body);


		body->setLinearFactor(btVector3(1,1,1));
		//body->setRestitution(1);

		btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=m_ShootBoxInitialSpeed;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
		body->setCcdMotionThreshold(1.);
		body->setCcdSweptSphereRadius(0.2f);

	}
}
#endif

void FractureDemo::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	m_dynamicsWorld = 0;

	delete m_solver;
	m_solver = 0;

	delete m_broadphase;
	m_broadphase = 0;

	delete m_dispatcher;
	m_dispatcher = 0;

	delete m_collisionConfiguration;
	m_collisionConfiguration = 0;
}

btCompoundShape* FractureDemo::LoadHACDOBJ(const char* objFileName)
{
	// use tinyobj to load fractures;
	b3BulletDefaultFileIO fileIO;
	std::vector<tinyobj::shape_t> shapes;
	tinyobj::attrib_t attribute;
	{
		B3_PROFILE("tinyobj::LoadObj2");
		std::string err = LoadFromCachedOrFromObj(attribute, shapes, objFileName, "", &fileIO);
	}

	btCompoundShape* ret = new btCompoundShape{};

	// for each shape
	for (const auto& shape : shapes)
	{
		// create btCollisionShapes from fractures;
		btConvexHullShape* convHullShape = new btConvexHullShape();
		btScalar scaling(1);
		convHullShape->setLocalScaling(btVector3{ scaling, scaling, scaling });

		// add points;
		for (const auto idx : shape.mesh.indices)
		{
			btVector3 v{ attribute.vertices[idx.vertex_index * 3],
						 attribute.vertices[idx.vertex_index * 3 + 1],
						 attribute.vertices[idx.vertex_index * 3 + 2] };

			convHullShape->addPoint(v);
		}

		convHullShape->initializePolyhedralFeatures();
		float mass = 1.f;
		btVector3 localInertia{ 0.0, 0.0, 0.0 };
		convHullShape->calculateLocalInertia(mass, localInertia);
		ret->addChildShape(btTransform::getIdentity(), convHullShape);
	}

	return ret;
}

btBvhTriangleMeshShape* FractureDemo::LoadStaticConcaveObj(const char* objFileName)
{
	// use tinyobj to load fractures;
	b3BulletDefaultFileIO fileIO;
	std::vector<tinyobj::shape_t> shapes;
	tinyobj::attrib_t attribute;
	{
		B3_PROFILE("tinyobj::LoadObj2");
		std::string err = LoadFromCachedOrFromObj(attribute, shapes, objFileName, "", &fileIO);
	}
	btAssert(shapes.size() == 1 && "Multiple objects in OBJ file. use LoadHACDObj() instead.");
	btTriangleMesh* meshInterface = new btTriangleMesh(true, false);

	const auto& tris = shapes[0];
	for (size_t i = 0; i < tris.mesh.indices.size() / 3; ++i)
	{
		const auto i0 = tris.mesh.indices[i * 3];
		const auto i1 = tris.mesh.indices[i * 3 + 1];
		const auto i2 = tris.mesh.indices[i * 3 + 2];
		const float* v0 = &attribute.vertices[i0.vertex_index * 3];
		const float* v1 = &attribute.vertices[i1.vertex_index * 3];
		const float* v2 = &attribute.vertices[i2.vertex_index * 3];
		meshInterface->addTriangle(
			btVector3(v0[0], v0[1], v0[2]),
			btVector3(v1[0], v1[1], v1[2]),
			btVector3(v2[0], v2[1], v2[2]), false);
	}
	btBvhTriangleMeshShape* ret = new btBvhTriangleMeshShape(meshInterface, true, true);

	return ret;
}

class CommonExampleInterface* FractureDemoCreateFunc(struct CommonExampleOptions& options)
{
	return new FractureDemo(options.m_guiHelper);
}
