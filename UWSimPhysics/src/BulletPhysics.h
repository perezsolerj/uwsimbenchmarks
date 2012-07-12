
#ifndef BULLETPHYSICS_H_
#define BULLETPHYSICS_H_

#include "SimulatorConfig.h"
#include "UWSimUtils.h"


#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>

#include <btBulletDynamicsCommon.h>
#include <iostream>

#include <osgOcean/OceanScene>

//#include <osgbCollision/GLDebugDrawer.h>



#define UWSIM_DEFAULT_GRAVITY	btVector3(0,0,-1.0)

// Define filter groups
enum CollisionTypes {
    COL_NOTHING = 0x00000000,
    COL_OBJECTS = 0x00000001,
    COL_VEHICLE = 0x00000010,
    COL_EVERYTHING = 0x11111111,
};



/*class NodeDataType : public osg::Referenced{
    public:
       NodeDataType(btRigidBody * rigidBody,int catcha){ catchable=catcha; rb=rigidBody;}; 
       int catchable;
       btRigidBody * rb;
       
};*/

class CollisionDataType : public osg::Referenced{
    public:
       CollisionDataType(std::string nam,std::string vehName,int isVehi){vehicleName=vehName;name=nam;isVehicle=isVehi;};
       std::string getObjectName(){if(isVehicle) return vehicleName; else return name;};
       std::string name, vehicleName;
       int isVehicle;
       
};

class BulletPhysics: public osg::Referenced {

public:
	typedef enum {SHAPE_BOX, SHAPE_TRIMESH,SHAPE_COMPOUND_TRIMESH,SHAPE_COMPOUND_BOX} collisionShapeType_t;

	btDynamicsWorld * dynamicsWorld;
	//osgbCollision::GLDebugDrawer debugDrawer;

	BulletPhysics(double configGravity[3],osgOcean::OceanTechnique* oceanSurf);

	void setGravity(btVector3 g) {dynamicsWorld->setGravity( g );}

	btRigidBody* addDynamicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data,osg::Node * colShape = NULL );
	btRigidBody* addKinematicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data,osg::Node * colShape= NULL);

	btRigidBody* addFloatingObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data ,osg::Node * colShape= NULL);

	void stepSimulation(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep );
	void printManifolds();

	int getNumCollisions();

	btPersistentManifold * getCollision(int i);

	~BulletPhysics() {};

private:
	btDefaultCollisionConfiguration * collisionConfiguration;
	btCollisionDispatcher * dispatcher;
	btConstraintSolver * solver;
	btBroadphaseInterface * inter;

	std::vector<btRigidBody *> floatingBodies;
	std::vector<double> floatForces;
	osgOcean::OceanTechnique* oceanSurface;

	void processFloatingObjects();
	void cleanManifolds();
	btCollisionShape* GetCSFromOSG(osg::Node * node, collisionShapeType_t ctype);
	btRigidBody* addObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data,osg::Node * colShape= NULL );

	
};


#endif

