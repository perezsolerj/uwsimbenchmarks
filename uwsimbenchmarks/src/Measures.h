#ifndef MEASURES_H_
#define MEASURES_H_

#define MAX_COLLISION_SPEED 0.5

#include <ros/ros.h>
#include <iostream>
#include "Trigger.h"
#include "uwsim/BulletPhysics.h"
#include <osg/ComputeBoundsVisitor>

#include <uwsimbenchmarks/GTpublish.h>


//Abstract class for holding Benchmarking measures
class Measures{
public:
  std::string name;
  bool addToGlobal;
  double log;
  virtual double getMeasure(void)=0;
  virtual int isOn()=0;
  virtual void start()=0;
  virtual void stop()=0;
  virtual void update()=0;
  virtual int error()=0;
  virtual std::vector<double> getMeasureDetails(void);
  virtual std::vector<std::string> getNameDetails(void);
  virtual void reset()=0;
  void setTriggers(Trigger * start, Trigger * stop);
  void setName(std::string name);
  void setLog(double log);
  void setAddToGlobal(bool addToGlobal);
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
  osg::Vec3f distanceDetails;
  ROSPoseToPositionError * sub;
  int valid; 
  osg::Quat originalRot;

public:
  void start(void);
  void stop(void);
  void update(void);
  PositionError(osg::Node * targ,double pos[3],std::string topic);
  double getMeasure(void);
  std::vector<double> getMeasureDetails(void);
  std::vector<std::string> getNameDetails(void);
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

  class RelativeLocation: public GT{
    private:
      osg::Node * from, *to;
    public: 
      std::vector<double> getGT();
      RelativeLocation(osg::Node * from,osg::Node * to);
  };

private:
  std::vector<double> groundTruth;
  ROSArrayToEuclideanNorm * topic;
  double norm;
  GT * gt;
  std::string  publishOn;

public:
  void start(void);
  void stop(void);
  void update(void);
  EuclideanNorm(GT * groundT, std::string topic, std::string publishOn);
  double getMeasure(void);
  std::vector<double> getMeasureDetails(void);
  std::vector<std::string> getNameDetails(void);
  int isOn();
  void reset();
  int error();
};

class ObjectCenteredOnCam: public Measures{
private:
  osg::Camera * cam;
  osg::Node * target;
public:
  void start(void);
  void stop(void);
  void update(void);
  ObjectCenteredOnCam(osg::Camera * cam,osg::Node * target);
  double getMeasure(void);
  std::vector<double> getMeasureDetails(void);
  std::vector<std::string> getNameDetails(void);
  int isOn();
  void reset();
  int error();
};

class Reconstruction3D: public Measures{
private:
  ROSPointCloudTo3DReconstruction * topic;
  osg::BoundingBox box;
  osg::Matrix TcF;
  double meanError, errorVariance, resolution[3];
  int npoints, noutliers;
  bool ***occupancy;
  int occupancyDim[3];
  int gridPoints, gridOccupiedPoints;
  osg::Node * target;
  osg::ref_ptr<osg::Group> drawPoints;
  boost::shared_ptr<osg::Matrixd> WcT;
public:
  void start(void);
  void stop(void);
  void update(void);
  Reconstruction3D( std::string topic ,osg::Node * target, double resolution, osg::Node * from);
  double getMeasure(void);
  std::vector<double> getMeasureDetails(void);
  std::vector<std::string> getNameDetails(void);
  int isOn();
  void reset();
  int error();
  void processPoints(void);
};

class PathFollowing: public Measures{
private:
  ROSPathToPathFollowing * topic;
  ROSIntToPathFollowing * wptTopic;
  std::vector<osg::Vec3f> path;
  int currentPoint;
  osg::Node * from, *target;
  int wptFromTopic;
  double lastError;
  double ise;
  ros::WallTime lastTick;
public:
  void start(void);
  void stop(void);
  void update(void);
  PathFollowing( std::string topic ,osg::Node * target, osg::Node * from);
  double getMeasure(void);
  std::vector<double> getMeasureDetails(void);
  std::vector<std::string> getNameDetails(void);
  int isOn();
  void reset();
  int error();

  void drawPath();
};


#endif
