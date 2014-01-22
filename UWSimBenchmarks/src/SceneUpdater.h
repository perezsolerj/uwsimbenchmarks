#ifndef SCENEUPDATER_H_
#define SCENEUPDATER_H_

#include <ros/ros.h>
#include <osg/Fog>
#include "uwsim/osgOceanScene.h"
#include "uwsim/SimulatedIAUV.h"
//#include "uwsim/uwsim/Current.h"

class SceneUpdater{
private:
  ros::WallTime init;
  double interval;
  int started;
protected:
  void restartTimer();
public:
void start();
virtual int needsUpdate();
SceneUpdater(double interval);
SceneUpdater(){};

virtual void update() =0;
virtual void updateScene() =0;
virtual int finished() =0;
virtual double getReference()=0;
virtual std::string getName()=0;
};

class NullSceneUpdater: public SceneUpdater{
private:
int finish;
public:
int needsUpdate();
void updateScene();
int finished();
NullSceneUpdater(){finish=0;};
double getReference();
std::string getName();
void update(){};

};

class SceneFogUpdater: public SceneUpdater{
private:
double initialFog,finalFog,step;
std::vector<osg::Fog *>  camerasFog;
osg::ref_ptr<osgOceanScene> scene;
public:
void updateScene();
int finished();
SceneFogUpdater(double initialFog, double finalFog, double step, double interval,std::vector<osg::Fog *>  camerasFog, osg::ref_ptr<osgOceanScene> scene);
double getReference();
std::string getName();
void update(){};
};

/*class CurrentForceUpdater: public SceneUpdater{
private:
  double initialCurrent,finalCurrent,step;
  SimulatedIAUV *  vehicle;
  osg::Matrixd m;
  boost::shared_ptr<Current> current;
public:
  void updateScene();
  int finished();
  void update(){};
  CurrentForceUpdater(double initialCurrent, double finalCurrent, double step, double interval,SimulatedIAUV *  vehicle, boost::shared_ptr<Current>  current);
  double getReference();
  std::string getName();
};*/

class ArmMoveUpdater: public SceneUpdater{
private:
  std::list<std::vector <double> > armPositions;
  double steps; //Counter for position (1= first position, 1.5= average position between first and second)
  SimulatedIAUV *  vehicle;
public:
  void updateScene();
  int finished();
  void update(){};
  ArmMoveUpdater(std::list<std::vector <double> > armPositions, double steps, double interval,SimulatedIAUV *  vehicle);
  double getReference();
  std::string getName();
};

#endif
