#ifndef MEASURES_H_
#define MEASURES_H_

#define MAX_COLLISION_SPEED 0.5

#include <ros/ros.h>
#include <iostream>
#include "Trigger.h"
#include "BulletPhysics.h"


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
private:
  std::vector<double> groundTruth;
  int nVals;
  ROSArrayToEuclideanNorm * topic;
  double norm;

public:
  void start(void);
  void stop(void);
  void update(void);
  EuclideanNorm(std::vector<double> groundT, int nVal, std::string topic);
  double getMeasure(void);
  int isOn();
  void reset();
  int error();

};

#endif
