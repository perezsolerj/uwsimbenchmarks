#include "BulletPhysics.h"

// Define filter masks
unsigned int vehicleCollidesWith( COL_OBJECTS);
unsigned int objectsCollidesWith( COL_EVERYTHING  );

class MyMotionState : public btMotionState {
public:

  MyMotionState(osg::Node * obj, osg::MatrixTransform *root){
    transf=root;
    object=obj;
  }

  void setNode(osg::Node *node) {
        object = node;
  }

  virtual void getWorldTransform(btTransform &worldTrans) const {
        worldTrans = osgbCollision::asBtTransform(*getWorldCoords(object));
  }

  virtual void setWorldTransform(const btTransform &worldTrans){
    //Object initial position
    osg::Matrixd * mat= getWorldCoords(transf->getParent(0));

    //Get object position in matrixd
    osg::Matrixd worldMatrix;
    btQuaternion rot = worldTrans.getRotation();
    btVector3 pos = worldTrans.getOrigin();
    worldMatrix.setTrans(osg::Vec3d(pos.x(),pos.y(),pos.z()));
    worldMatrix.setRotate(osg::Quat(rot.x(),rot.y(),rot.z(),rot.w()));
    
    //matrix transform from object initial position to final position
    osg::Matrixd rootmat=worldMatrix*(mat->inverse(*mat));

    //Apply transform to object matrix
    rootmat.setTrans(rootmat.getTrans());
    rootmat.setRotate(rootmat.getRotate());
    transf->setMatrix(rootmat);

  }

protected:
  osg::Node * object;
  osg::MatrixTransform *transf;

};

void BulletPhysics::stepSimulation(btScalar timeStep, int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.) ) {
  //dynamicsWorld->debugDrawWorld();
  //printManifolds();
  cleanManifolds();
  processFloatingObjects();
  dynamicsWorld->stepSimulation( timeStep, maxSubSteps, fixedTimeStep);
}

void BulletPhysics::printManifolds(){
  std::cout<<dispatcher->getNumManifolds()<<std::endl;
  for(int i=0;i<dispatcher->getNumManifolds();i++){
    btCollisionObject* colObj0 =(btCollisionObject*) dispatcher->getManifoldByIndexInternal(i)->getBody0();
    btCollisionObject* colObj1 = (btCollisionObject*)dispatcher->getManifoldByIndexInternal(i)->getBody1();
    CollisionDataType * nombre=(CollisionDataType *)colObj0->getUserPointer();
    CollisionDataType * nombre2=(CollisionDataType *)colObj1->getUserPointer();
    double min=999999;
    for(int j=0;j<dispatcher->getManifoldByIndexInternal(i)->getNumContacts();j++)
	if(dispatcher->getManifoldByIndexInternal(i)->getContactPoint(j).getDistance() < min)
	  min=dispatcher->getManifoldByIndexInternal(i)->getContactPoint(j).getDistance();
    if(min<999999)
    std::cout<<i<<" "<<nombre->name<<" "<<" "<<nombre2->name<<" "<<min<<std::endl;
  }
}

BulletPhysics::BulletPhysics(double configGravity[3],osgOcean::OceanTechnique* oceanSurf) {
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher( collisionConfiguration );
    solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    btVector3 gravity(configGravity[0],configGravity[1],configGravity[2]);
    if(configGravity[0]==0 && configGravity[1]==0 && configGravity[2]==0){
      gravity=UWSIM_DEFAULT_GRAVITY;
    }

    dynamicsWorld->setGravity( gravity);
    floatingBodies.clear();
    floatForces.clear();
    oceanSurface=oceanSurf;
    /*debugDrawer.setDebugMode(btIDebugDraw::DBG_DrawContactPoints|| btIDebugDraw::DBG_DrawWireframe || btIDebugDraw::DBG_DrawText);
    dynamicsWorld->setDebugDrawer(&debugDrawer);
    debugDrawer.BeginDraw();
    debugDrawer.setEnabled(true);*/
}

btCollisionShape* BulletPhysics::GetCSFromOSG(osg::Node * node, collisionShapeType_t ctype){
    btCollisionShape* cs=NULL;

    if (ctype==SHAPE_BOX)
	cs= osgbCollision::btBoxCollisionShapeFromOSG(node);
    else if (ctype==SHAPE_COMPOUND_TRIMESH)
	cs= osgbCollision::btCompoundShapeFromOSGGeodes(node,CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE);
    else if (ctype==SHAPE_COMPOUND_BOX)
	cs= osgbCollision::btCompoundShapeFromOSGGeodes(node,BOX_SHAPE_PROXYTYPE);
    else if (ctype==SHAPE_TRIMESH)
	cs= osgbCollision::btTriMeshCollisionShapeFromOSG(node);

    return cs;
}

btRigidBody* BulletPhysics::addObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data,osg::Node * colShape) {
    

   btCollisionShape* cs;
   if(colShape==NULL)
     cs=GetCSFromOSG( node, ctype);
   else
     cs=GetCSFromOSG( colShape, ctype);

    MyMotionState* motion = new MyMotionState(node,root);
    cs->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, cs, inertia );
    btRigidBody* body = new btRigidBody( rb );
    body->setUserPointer(data);

    //addRigidBody adds its own collision masks, changing after object creation do not update masks so objects are removed and readded in order to update masks to improve collisions performance.
    dynamicsWorld->addRigidBody( body);
    if(data->isVehicle){
      dynamicsWorld->btCollisionWorld::removeCollisionObject(body);
      dynamicsWorld->addCollisionObject(body,short( COL_VEHICLE),short(vehicleCollidesWith));
    }
    else{
      dynamicsWorld->btCollisionWorld::removeCollisionObject(body);
      dynamicsWorld->addCollisionObject(body,short( COL_OBJECTS),short(objectsCollidesWith));
    }

 

    return( body );
}

btRigidBody* BulletPhysics::addFloatingObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data,osg::Node * colShape ) {

  btRigidBody* floating = addObject(root, node, mass,inertia,ctype,data,colShape);
  btVector3 min,max;
  floating->getAabb(min,max);
  max=max-min;
  double volume=max.x()*max.y()*max.z();

  floatingBodies.push_back(floating);
  floatForces.push_back(volume*1027);
  //std::cout<<"VOLUME: "<<volume<<" Empuje: "<<volume*1027<<std::endl;

  return floating;
}

btRigidBody* BulletPhysics::addDynamicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data,osg::Node * colShape ) {
	return addObject(root, node, mass,inertia,ctype,data,colShape);
}

btRigidBody* BulletPhysics::addKinematicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data,osg::Node * colShape) {
	btRigidBody *b=addObject(root,node, mass,inertia,ctype,data,colShape);
	if (b!=NULL) {
	  b->setCollisionFlags( b->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
          b->setActivationState( DISABLE_DEACTIVATION );
	}  
	return b;
}


int BulletPhysics::getNumCollisions(){
  return dispatcher->getNumManifolds();
}

btPersistentManifold * BulletPhysics::getCollision(int i){
  return dispatcher->getManifoldByIndexInternal(i);
}


void BulletPhysics::cleanManifolds(){  //it removes contact points with long lifetime
  //std::cout<<dispatcher->getNumManifolds()<<"aa"<<std::endl;
  for(int i=0;i<dispatcher->getNumManifolds();i++){
    btPersistentManifold * col = dispatcher->getManifoldByIndexInternal(i);
    //std::cout<<col->getNumContacts()<<std::endl;
    for(int j=0;j < col->getNumContacts();j++)
	if(col->getContactPoint(j).getLifeTime() > 300)
	  col->removeContactPoint(j);

  }	  
}

void BulletPhysics::processFloatingObjects(){

  for(unsigned int i=0;i<floatingBodies.size();i++){
    btRigidBody * floating=floatingBodies[i];
    btVector3 min,max;
    floating->getAabb(min,max);
    btVector3 velocidad=floating->getLinearVelocity();
    //std::cout<<"min:"<<min.x()<<" "<<min.y()<<" "<<min.z()<<" max: "<<max.x()<<" "<<max.y()<<" "<<max.z()<<std::endl;
    
    btVector3 med=(max+min)/2.0;
    btVector3 alt=max-min;
    double Adn= alt.x()*alt.y(); //Area de la superficie normal a la direccion de movimiento
    double Ads= floatForces[i]/1027; //Volumen del objeto
    double Dn= pow(Adn/M_PI,0.5)*2,Ds=pow(Ads/M_PI*3/4.0,1/3.0)*2;
    double seaWaterViscosity=0.001792 *1000;
    double froz=3*M_PI*seaWaterViscosity*velocidad.z()*Dn*(1/3.0+(2/3.0)*(Ds/Dn));
    double fuerza=floatForces[i]*-1-froz,oceanSurf=oceanSurface->getSurfaceHeightAt(med.x(),med.y()) ;

    std::cout<<Adn<<" "<<Ads<<" "<<froz<<" "<<Dn<<" "<<Ds<<" "<<froz<<std::endl;

    //std::cout<<"altura: "<<alt.z()<<" Oceansurf: "<<oceanSurf<<" med: "<<med.z()<<std::endl;
    if(med.z()-oceanSurf < -alt.z())
	  fuerza=0;
    else if(med.z()-oceanSurf < alt.z())
	  fuerza=(med.z()-oceanSurf+alt.z())*fuerza-froz;
    //std::cout<<"fuerza: "<< fuerza<<" froz: "<<froz<<" velocidad: "<<velocidad.z()<<" Total:"<<fuerza-froz*velocidad.z()<<std::endl;
   floating->applyForce(btVector3(0,0,fuerza),btVector3(0,0,0));

    //std::cout<<oceanSurf<<" "<<max.z()<<std::endl;
  }

}
