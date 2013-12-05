#include "SceneUpdater.h"
#include <std_srvs/Empty.h>


SceneUpdater::SceneUpdater(double interval){
  this->interval=interval;
  started=0;
}

void SceneUpdater::start(){
  init = ros::WallTime::now();
  started=1;
}

int SceneUpdater::needsUpdate(){
  if(started){
    update();
    ros::WallDuration t_diff = ros::WallTime::now() - init;
    return t_diff.toSec() > interval;
  }
  return 0;
}

void SceneUpdater::restartTimer(){
  init = ros::WallTime::now();
  started=0;
}

/*NULL SCENE UPDATER*/

int NullSceneUpdater::needsUpdate(){
  return 0;
}

void NullSceneUpdater::updateScene(){
  finish=1;
}

int NullSceneUpdater::finished(){
  return finish;
}

double NullSceneUpdater::getReference(){
  return 1;
}

std::string NullSceneUpdater::getName(){
  return "None";
}

/*SCENE FOG UPDATER*/

void SceneFogUpdater::updateScene(){
//std::cout<<"Updated "<<initialFog<<std::endl;
restartTimer();
initialFog+=step;

for(unsigned int i=0;i<camerasFog.size();i++)
  camerasFog[i]->setDensity(initialFog);
scene->getOceanScene()->setUnderwaterFog(initialFog, osg::Vec4f(0,0.05,0.3,1) );
}

int SceneFogUpdater::finished(){
  return initialFog>finalFog;
}

SceneFogUpdater::SceneFogUpdater(double initialFog, double finalFog, double step, double interval, std::vector<osg::Fog *>  camerasFog, osg::ref_ptr<osgOceanScene> scene): SceneUpdater(interval){
this->initialFog=initialFog;
this->finalFog=finalFog;
this->step=step;
this->camerasFog=camerasFog;
this->scene=scene;

for( unsigned int i=0;i<camerasFog.size();i++)
  camerasFog[i]->setDensity(initialFog);
scene->getOceanScene()->setUnderwaterFog(initialFog, osg::Vec4f(0,0.05,0.3,1) );
}

double SceneFogUpdater::getReference(){
  return initialFog;
}

std::string SceneFogUpdater::getName(){
  return "Fog";
}

/*Current Force Updater*/

void CurrentForceUpdater::updateScene(){
  //std::cout<<"Updated "<<std::endl;
  restartTimer();
  initialCurrent+=step;
  vehicle->setOffset(0,0,0);
  vehicle->setVehiclePosition(m);
  current->changeCurrentForce(initialCurrent,1);

  /*std_srvs::Empty::Request request, response;
  ros::service::call("Dynamics/reset_navigation",request, response);*/
}

int CurrentForceUpdater::finished(){
  return initialCurrent>finalCurrent;
}

CurrentForceUpdater::CurrentForceUpdater(double initialCurrent, double finalCurrent, double step, double interval, SimulatedIAUV *  vehicle,boost::shared_ptr<Current>  current): SceneUpdater(interval){
  this->initialCurrent=initialCurrent;
  this->finalCurrent=finalCurrent;
  this->step=step;
  this->vehicle=vehicle;
  this->current=current;
  m=vehicle->baseTransform->getMatrix();

  vehicle->setOffset(0,0,0);
}

double CurrentForceUpdater::getReference(){
  return initialCurrent;
}

std::string CurrentForceUpdater::getName(){
  return "Current";
}

/*Arm Move Updater*/

void ArmMoveUpdater::updateScene(){
  restartTimer();

  vehicle->urdf->setJointPosition(armPositions.front());
  armPositions.pop_front();
}

int ArmMoveUpdater::finished(){
  return !armPositions.size();
}

ArmMoveUpdater::ArmMoveUpdater(std::list<std::vector <double> > armPositions, double steps, double interval,SimulatedIAUV *  vehicle): SceneUpdater(interval){

  this->armPositions.push_back(armPositions.front());
  armPositions.pop_front();
  while(armPositions.size()>0){
    std::vector<double> actual=this->armPositions.back();
    for(unsigned int i=1;i<=steps;i++){ //0 steps = just move from one position to the next one
      std::vector<double> nextStep;
      for(unsigned int j=0;j<armPositions.front().size();j++){
	nextStep.push_back(actual[j]+ (armPositions.front()[j]-actual[j])/(steps+1)*i);
      }
      this->armPositions.push_back(nextStep);
    }
    this->armPositions.push_back(armPositions.front());
    armPositions.pop_front();
  }
  this->steps=1;
  this->vehicle=vehicle;
  this->armPositions.push_back(std::vector<double> ()); //Needed to check if update is over

  vehicle->urdf->setJointPosition(this->armPositions.front());
  this->armPositions.pop_front();

}

double ArmMoveUpdater::getReference(){
  return steps;
}

std::string ArmMoveUpdater::getName(){
  return "ArmPosition";
}
