#ifndef SCENEUPDATER_H_
#define SCENEUPDATER_H_

#include <ros/ros.h>
#include <osg/Fog>
#include "osgOceanScene.h"

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
};

#endif
