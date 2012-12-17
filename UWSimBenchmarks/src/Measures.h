#ifndef MEASURES_H_
#define MEASURES_H_

#define MAX_COLLISION_SPEED 0.5

#include <ros/ros.h>
#include <iostream>
#include "Trigger.h"
#include "BulletPhysics.h"
#include <osg/ComputeBoundsVisitor>


//Abstract class for holding Benchmarking measures
class Measures{
public:
  std::string name;
  virtual double getMeasure(void)=0;
  virtual int isOn()=0;
  virtual void start()=0;
  virtual void stop()=0;
  virtual void update()=0;
  virtual int error()=0;
  void reset();
  void setTriggers(Trigger * start, Trigger * stop);
  void setName(std::string name);
private:

protected:
  Trigger * startOn;
  Trigger * stopOn;
};


class Time: public Measures{
private:
  ros::WallTime init;
  double timeElapsed;

public:
  void start(void);
  void stop(void);
  void update(void);
  Time();
  double getMeasure(void);
  int isOn();
  void reset();
  int error();

};

class Collisions: public Measures{
private:
  BulletPhysics * physics;
  std::string target;
  int ticks_collision,ticks;
  double maxSpeedCollision,speedCollisionSum;

public:
  void start(void);
  void stop(void);
  void update(void);
  Collisions(BulletPhysics * physics,std::string targ);
  double getMeasure(void);
  int isOn();
  void reset();
  int error();
};

class PositionError: public Measures{
private:
  osg::Node * target;
  osg::Vec3f position;
  double distance;

public:
  void start(void);
  void stop(void);
  void update(void);
  PositionError(osg::Node * targ,double pos[3]);
  double getMeasure(void);
  int isOn();
  void reset();
  int error();

};

class Distance: public Measures{
private:
  osg::Node * target;

  osg::Vec3f last;
  double distance;

public:
  void start(void);
  void stop(void);
  void update(void);
  Distance(osg::Node * targ);
  double getMeasure(void);
  int isOn();
  void reset();
  int error();

};

class EuclideanNorm: public Measures{
public:

  class GT{
    public:
      virtual std::vector<double> getGT()=0;
  };

  class ConstantGT: public GT{
    public:
      std::vector<double> groundTruth;
      std::vector<double> getGT(){return groundTruth;};
      ConstantGT(std::vector<double> groundT){groundTruth=groundT;};
  };

  class ObjectCornersInCam: public GT{
    private:
      osg::Camera * cam;
      osg::Node * target;
    public: 
      std::vector<double> getGT();
      ObjectCornersInCam(osg::Camera * cam,osg::Node * target);
  };

  class ObjectCentroidInCam: public GT{
    private:
      osg::Camera * cam;
      osg::Node * target;
    public: 
      std::vector<double> getGT();
      ObjectCentroidInCam(osg::Camera * cam,osg::Node * target);
  };

private:
  std::vector<double> groundTruth;
  ROSArrayToEuclideanNorm * topic;
  double norm;
  GT * gt;

public:
  void start(void);
  void stop(void);
  void update(void);
  EuclideanNorm(GT * groundT, std::string topic);
  double getMeasure(void);
  int isOn();
  void reset();
  int error();



};

#endif
