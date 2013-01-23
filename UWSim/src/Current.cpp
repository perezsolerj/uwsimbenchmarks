#include "Current.h"
#include <iostream>

#define MAX_ELAPSED 0.5

Current::Current(double module, double direction[], double moduleVariation, double modulePeriod, double directionVariation[], double directionPeriod[], double randomNoise){
  this->module=module;
  this->direction[0]=direction[0];
  this->direction[1]=direction[1];
  this->moduleVariation=moduleVariation;
  this->modulePeriod=modulePeriod;
  this->directionVariation[0]=directionVariation[0];
  this->directionVariation[1]=directionVariation[1];
  this->directionPeriod[0]=directionPeriod[0];
  this->directionPeriod[1]=directionPeriod[1];
  this->randomNoise=randomNoise;
  srand (ros::WallTime::now().toSec());
  last=ros::WallTime::now();
}

void Current::applyCurrent(std::vector<boost::shared_ptr<SimulatedIAUV> > iauvFile){

  double elapsedFromBeg=ros::WallTime::now().toSec();
  double elapsed=(ros::WallTime::now()-last).toSec();
  if(elapsed>MAX_ELAPSED) //We assume it's too much time (It happens in first iteration if scene is complex)
    elapsed=0;
  double random;

  //Compute current function, (a more realistic function should be created)
  //Constant+sinusoidal signal+random signal
  random=(rand()/(RAND_MAX/(randomNoise*2)))-randomNoise; //Random number in range [-randomNoise,randomNoise]
  double force=(module+moduleVariation*sin(fmod(elapsedFromBeg,modulePeriod)/modulePeriod*2*3.14))*(1+random);
  random=(rand()/(RAND_MAX/(randomNoise*2)))-randomNoise; //Random number in range [-randomNoise,randomNoise]
  double dir1=(direction[0]+directionVariation[0]*sin(fmod(elapsedFromBeg,directionPeriod[0])/directionPeriod[0]*2*3.14))*(1+random);
  random=(rand()/(RAND_MAX/(randomNoise*2)))-randomNoise; //Random number in range [-randomNoise,randomNoise]
  double dir2=(direction[1]+directionVariation[1]*sin(fmod(elapsedFromBeg,directionPeriod[1])/directionPeriod[1]*2*3.14))*(1+random);

  //Apply current to vehicles (current may be in function of vehicle position)
  for(unsigned int i=0; i<iauvFile.size();i++){

    //Get direction vector for current
    double x=sin(dir1)*cos(dir2);
    double y=sin(dir1)*sin(dir2);
    double z=cos(dir1);
    //std::cout<<x<<" "<<y<<" "<<z<<" sacado de: "<<dir1<<" "<<dir2<<std::endl;
    //Get displacement
    osg::Vec3f prueba=(osg::Vec3f(y,-x,z))*elapsed*force;

    //Apply displacement to vehicle
    osg::Matrix transf;
    transf.preMultTranslate(prueba+iauvFile[i]->baseTransform->getMatrix().getTrans());
    transf.preMultRotate(iauvFile[i]->baseTransform->getMatrix().getRotate());
    iauvFile[i]->setVehiclePosition(transf);


    //Update arrow position
    osg::Matrix mat=iauvFile[i]->arrow->getMatrix();

    transf.makeIdentity();
    transf.preMultTranslate(osg::Vec3d(force*2.5*y,force*2.5*(-x),force*2.5*z)); //Keep arrow origin stable
    transf.preMultRotate(iauvFile[i]->baseTransform->getMatrix().getRotate().inverse()); //Make arrow independient from vehicle rotation
    transf.preMultRotate(osg::Quat(dir1,osg::Vec3d(1,0,0)));
    transf.preMultRotate(osg::Quat(dir2,osg::Vec3d(0,1,0)));
    transf.preMultScale(osg::Vec3d(0.5,0.5,force*5));  //scale arrow's length
    iauvFile[i]->arrow->setMatrix(transf);


    }
  last=ros::WallTime::now();
}
