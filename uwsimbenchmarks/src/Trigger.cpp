#include "Trigger.h"

/****** TOPIC TRIGGER *******/

TopicTrigger::TopicTrigger(std::string target){
   trigger=new ROSTrigger(target,this);
   on=0;
}

void TopicTrigger::start(){
  on=1;
}

void TopicTrigger::stop(){
  on=0;
}

int TopicTrigger::isOn(){
  return on;
}

void TopicTrigger::reset(){
  on=0;
}

/****** ALWAYS ON *******/

int AlwaysOnTrigger::isOn(){
  return 1;
}

void AlwaysOnTrigger::reset(){
}

/****** ALWAYS OFF *******/

int AlwaysOffTrigger::isOn(){
  return 0;
}

void AlwaysOffTrigger::reset(){
}

/****** SERVICE TRIGGER *******/

ServiceTrigger::ServiceTrigger(std::string target){
   trigger=new ROSServiceTrigger(target,this);
   on=0;
}

void ServiceTrigger::start(){
  on=1;
}

int ServiceTrigger::isOn(){
  return on;
}

void ServiceTrigger::reset(){
  on=0;
}

/****** MOVE TRIGGER *******/

MoveTrigger::MoveTrigger(osg::Node * target){
  this->target=target;
  moved=0;
  getWorldCoords(target)->decompose(position, rotation, scale, rotationScale);
  //std::cout<<position.x()<<" "<<position.y()<<" "<<position.z()<<" "<<rotation.x()<<" "<<rotation.y()<<" "<<rotation.z()<<" "<<rotation.w()<<std::endl;
}

int MoveTrigger::isOn(){
  if(!moved){
    osg::Vec3f newposition, newscale;
    osg::Quat newrotation, newrotationScale;

    getWorldCoords(target)->decompose(newposition, newrotation, newscale, newrotationScale);
    //std::cout<<newposition.x()<<" "<<newposition.y()<<" "<<newposition.z()<<std::endl;
    newposition=newposition-position;
    float posdiff=newposition.x()*newposition.x()+newposition.y()*newposition.y()+newposition.z()*newposition.z();
    //std::cout<<posdiff<<std::endl;
  
    if (posdiff>MOVE_THRESHOLD)
      moved=1;

    //TODO rotation threshold
  }
  return moved;
}

void MoveTrigger::reset(){
  moved=0;
  getWorldCoords(target)->decompose(position, rotation, scale, rotationScale);
}


/****** NO MOVE TRIGGER *******/

NoMoveTrigger::NoMoveTrigger(osg::Node * target){

  this->target=target;
  stopped=0;
}

int NoMoveTrigger::isOn(){
  if(!stopped){
    osg::Vec3f newposition,difference, newscale;
    osg::Quat newrotation, newrotationScale;
    getWorldCoords(target)->decompose(newposition, newrotation, newscale, newrotationScale);  
    ros::WallTime now = ros::WallTime::now();
    ros::WallDuration t_diff;
    //Check if new measure needed
    if(positions.size()==0){ 
      positions.push_back(newposition);
      inits.push_back(now);
    }
    t_diff=now-inits.back();
    if(t_diff.toSec()> (float)NOMOVE_TIME_THRESHOLD/NOMOVE_NUMMEASURES){
      positions.push_back(newposition);
      inits.push_back(now);
    }
	
    //Check if first measure can be discarded or vehicle stopped
    int stop=0;
    float distance;
    while(positions.size()>0 && !stop){
      difference=newposition-positions.front();
      distance=difference.x()*difference.x()+difference.y()*difference.y()+difference.z()*difference.z();
      if(distance > MOVE_THRESHOLD){ //TODO rotation threshold
        positions.pop_front();
        inits.pop_front();
      }
      else{
        t_diff=now-inits.front();
        if(t_diff.toSec() > NOMOVE_TIME_THRESHOLD)
          stopped=1;
        stop=1;
      }
    }
  }
  return stopped;
}

void NoMoveTrigger::reset(){
  stopped=0;
  positions.clear();
  inits.clear();
}

/****** POSITION TRIGGER *******/

PositionTrigger::PositionTrigger(osg::Node * target,double position[3]){
  this->target=target;
  arrived=0;
  this->position=osg::Vec3f(position[0],position[1],position[2]);
}

int PositionTrigger::isOn(){
  if(!arrived){
    osg::Vec3f newposition, newscale;
    osg::Quat newrotation, newrotationScale;

    getWorldCoords(target)->decompose(newposition, newrotation, newscale, newrotationScale);

    newposition=newposition-position;
    float posdiff=newposition.x()*newposition.x()+newposition.y()*newposition.y()+newposition.z()*newposition.z();

    if (posdiff<POSITION_THRESHOLD)
      arrived=1;

  }
  return arrived;
}

void PositionTrigger::reset(){
  arrived=0;
}

