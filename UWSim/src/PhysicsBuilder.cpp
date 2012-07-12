#include "PhysicsBuilder.h"

PhysicsBuilder::PhysicsBuilder(SceneBuilder * scene_builder,ConfigFile config){

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
    if(scene_builder->objects[i]->getName()!="terrain"){
       physics->addDynamicObject(mt,scene_builder->objects[i],btScalar(0.5),btVector3(0,0,0), BulletPhysics::SHAPE_BOX,colData);
      //data = new NodeDataType(flotante,1);
    }
    else{
      floorbody=physics->addKinematicObject(mt,scene_builder->objects[i],btScalar(1),btVector3(0,0,0), BulletPhysics::SHAPE_TRIMESH,colData);
      //data = new NodeDataType(floorbody,0);
    }
    //wMb->setUserData(data); 
  }

  OSG_INFO << "Physics Loaded!" << std::endl;
}


