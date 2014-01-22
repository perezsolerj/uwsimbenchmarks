#ifndef TRIGGER_H_
#define TRIGGER_H_

#include <iostream>
class ROSTrigger;
class ROSServiceTrigger;
#include "BenchmarkROSInterfaces.h"
#include "uwsim/UWSimUtils.h"

#define POSITION_THRESHOLD 0.3 //Distance from defined position to real position threshold for position triggers.
#define MOVE_THRESHOLD 0.2  //Distance from start point to position for move and no move triggers.
#define NOMOVE_TIME_THRESHOLD 4  //Time of inactivity for no move trigger.
#define NOMOVE_NUMMEASURES 10 //Number of measures taken in nomove trigger to check if vehicle has been stopped. Increase number of measures to decrease time error (defined by NOMOVE_TIME_THRESHOLD/NOMOVE_NUMMEASURES).

class Trigger{
public:
virtual int isOn()=0;
virtual void reset()=0;
private:

};

class TopicTrigger: public Trigger{
private:
  ROSTrigger * trigger;
  int on;
public:
void start();
void stop();
TopicTrigger(std::string target);
void reset();
int isOn();

};

class AlwaysOnTrigger: public Trigger{
public:
int isOn();
AlwaysOnTrigger(){};
void reset();
};

class AlwaysOffTrigger: public Trigger{
public:
int isOn();
AlwaysOffTrigger(){};
void reset();
};

class ServiceTrigger: public Trigger{
private:
  ROSServiceTrigger * trigger;
  int on;
public:
int isOn();
ServiceTrigger(std::string target);
void start();
void reset();
};

class MoveTrigger: public Trigger{
private:
osg::Vec3f position, scale;
osg::Quat rotation, rotationScale;
osg::Node * target;
int moved;
public:
int isOn();
MoveTrigger(osg::Node * target);
void reset();
};

class NoMoveTrigger: public Trigger{
private:
list<osg::Vec3f> positions;
list<ros::WallTime> inits;
osg::Node * target;
int stopped;

public:
int isOn();
NoMoveTrigger(osg::Node * target);
void reset();
};

class PositionTrigger: public Trigger{
private:
osg::Vec3f position;
osg::Node * target;
int arrived;
public:
int isOn();
PositionTrigger(osg::Node * target,double position[3]);
void reset();
};


#endif
