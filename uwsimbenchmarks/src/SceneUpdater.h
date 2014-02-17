#ifndef SCENEUPDATER_H_
#define SCENEUPDATER_H_

#include <ros/ros.h>
#include <osg/Fog>
#include "uwsim/osgOceanScene.h"
#include "uwsim/SimulatedIAUV.h"
#include "Current.h"
#include "BenchmarkXMLParser.h"

class SceneUpdater{
private:
  ros::WallTime init;
  double interval;
  int started;
protected:
  void restartTimer();
  SceneUpdater * child;
public:
void start();
virtual int needsUpdate();
virtual void tick();
SceneUpdater(double interval);
SceneUpdater(){child=NULL;};
void addSceneUpdaterChild(SceneUpdater * su){child=su;}
void getReferences(std::vector<double> &refs);
void getNames(std::vector<std::string> &names);

virtual void update() =0;
virtual int updateScene() =0;  //Returns scene updater level
virtual int finished() =0;
virtual double getReference()=0;
virtual std::string getName()=0;
virtual void restart() =0;
};

class NullSceneUpdater: public SceneUpdater{
private:
int finish;
public:
int needsUpdate();
int updateScene();
int finished();
NullSceneUpdater(){finish=0;};
double getReference();
std::string getName();
void update(){};
void restart(){};

};

class SceneFogUpdater: public SceneUpdater{
private:
double initialFog,finalFog,step,fog;
std::vector<osg::Fog *>  camerasFog;
osg::ref_ptr<osgOceanScene> scene;
public:
int updateScene();
int finished();
SceneFogUpdater(double initialFog, double finalFog, double step, double interval,std::vector<osg::Fog *>  camerasFog, osg::ref_ptr<osgOceanScene> scene);
double getReference();
std::string getName();
void update(){};
void restart();
};

class CurrentForceUpdater: public SceneUpdater{
private:
  double initialCurrent,finalCurrent,step,myCurrent;
  SimulatedIAUV *  vehicle;
  osg::Matrixd m;
  boost::shared_ptr<Current> current;
public:
  int updateScene();
  int finished();
  void update(){};
  CurrentForceUpdater(double initialCurrent, double finalCurrent, double step, double interval,SimulatedIAUV *  vehicle,CurrentInfo currentInfo);
  double getReference();
  void tick();
  std::string getName();
  void restart();
};

class ArmMoveUpdater: public SceneUpdater{
private:
  std::list<std::vector <double> > armPositions;
  double steps; //Counter for position (1= first position, 1.5= average position between first and second)
  SimulatedIAUV *  vehicle;
public:
  int updateScene();
  int finished();
  void update(){};
  ArmMoveUpdater(std::list<std::vector <double> > armPositions, double steps, double interval,SimulatedIAUV *  vehicle);
  double getReference();
  std::string getName();
  void restart();
};

#endif
