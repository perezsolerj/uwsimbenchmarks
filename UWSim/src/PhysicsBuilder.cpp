#include "PhysicsBuilder.h"

PhysicsBuilder::PhysicsBuilder(SceneBuilder * scene_builder,ConfigFile config){
  loadPhysics(scene_builder,config);
}

void PhysicsBuilder::loadPhysics(SceneBuilder * scene_builder,ConfigFile config){

  physics = new BulletPhysics(config.gravity,scene_builder->scene->getOceanSurface());
  OSG_INFO << "Loading Physics" << std::endl;
  for (unsigned int i=0; i<scene_builder->iauvFile.size();i++){
    for(unsigned int j=0; j<scene_builder->iauvFile[i]->urdf->link.size();j++){
      osg::Node * link =scene_builder->iauvFile[i]->urdf->link[j];
      osg::Node * cs= NULL;
      //TODO: Add collision shape parsing....

      CollisionDataType * colData=new CollisionDataType(link->getName(),scene_builder->iauvFile[i]->name,1);
      physics->addKinematicObject(NULL,link,btScalar(1),btVector3(0,0,0), BulletPhysics::SHAPE_COMPOUND_BOX ,colData,cs);
      //TODO: Add node data type correctly.
      //NodeDataType * data= new NodeDataType(floorbody,0);
      //link->setUserData(data);
    }
  }
  
  for(unsigned int i=0; i<scene_builder->objects.size();i++){
    //create Matrix Transform to use it on physics
    osg::Matrix mat;
    mat.makeIdentity();
    osg::MatrixTransform * mt= new osg::MatrixTransform();
    osg::Group * parent= scene_builder->objects[i]->getParent(0);
    mt->addChild(scene_builder->objects[i]);
    parent->removeChild(scene_builder->objects[i]);
    parent->addChild(mt);

    //Add physics to object
    btRigidBody *floorbody;
    //NodeDataType * data;
    CollisionDataType * colData=new CollisionDataType(scene_builder->objects[i]->getName()," ",0);

    double mass=1, inertia[3];
    memset(inertia,0,3*sizeof(double));
    BulletPhysics::collisionShapeType_t shape=BulletPhysics::SHAPE_BOX;  
    for(std::list<Object>::iterator j=config.objects.begin();j!=config.objects.end();j++){
      if(j->name==scene_builder->objects[i]->getName() && j->physicProperties){
	mass=j->physicProperties->mass;
	inertia[0]=j->physicProperties->inertia[0];
	inertia[1]=j->physicProperties->inertia[1];
	inertia[2]=j->physicProperties->inertia[2];
	if(j->physicProperties->csType=="box")
	  shape=BulletPhysics::SHAPE_BOX;
	else if(j->physicProperties->csType=="compound box")
	  shape=BulletPhysics::SHAPE_COMPOUND_BOX;
	else if(j->physicProperties->csType=="trimesh")
	  shape=BulletPhysics::SHAPE_TRIMESH;
	else if(j->physicProperties->csType=="compound trimesh")
	  shape=BulletPhysics::SHAPE_COMPOUND_TRIMESH;
	else
	  OSG_WARN << "Object: "<< j->name<<" has an unknown collision shape type: "<<j->physicProperties->csType<<". Using default box shape. Check xml file, allowed collision shapes are 'box' 'compound box' 'trimesh' 'compound trimesh'." << std::endl;
      }

    }
    if(scene_builder->objects[i]->getName()!="terrain"){
       physics->addDynamicObject(mt,scene_builder->objects[i],btScalar(mass),btVector3(inertia[0],inertia[1],inertia[2]), shape,colData);
      //data = new NodeDataType(flotante,1);
    }
    else{
      floorbody=physics->addKinematicObject(mt,scene_builder->objects[i],btScalar(0),btVector3(0,0,0), BulletPhysics::SHAPE_TRIMESH,colData);
      //data = new NodeDataType(floorbody,0);
    }
    //wMb->setUserData(data); 
  }

  OSG_INFO << "Physics Loaded!" << std::endl;
}


