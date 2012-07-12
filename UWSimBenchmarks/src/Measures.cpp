#include "Measures.h"

int Measures::isOn(){
  return startOn->isOn() && ! stopOn->isOn();
}

void Measures::setTriggers(Trigger * start, Trigger * stop){
  startOn=start;
  stopOn=stop;
}

void Measures::setName(std::string name){
  this->name=name;
}

void Measures::reset(){
  startOn->reset();
  stopOn->reset();
}



//******************TIME ***************

Time::Time(){
  timeElapsed=0;
  init = ros::WallTime::now();
}

void Time::start(void){
  init = ros::WallTime::now();
}

void Time::update(void){
}

void Time::stop(void){
  ros::WallDuration t_diff = ros::WallTime::now() - init;
  timeElapsed = t_diff.toSec();
}

double Time::getMeasure(void){

  return timeElapsed;
}

int Time::isOn(){
  return Measures::isOn();
}

void Time::reset(){
  Measures::reset();
  timeElapsed=0;
  init = ros::WallTime::now();
}

int Time::error(){
  return 0;
}

//******************COLLISIONS ***************

Collisions::Collisions(BulletPhysics * physics,std::string targ){
  this->physics=physics;
  target=targ;
}

void Collisions::start(void){
  ticks_collision=0;
  ticks=1; //Avoid zero division
  maxSpeedCollision=0;
  speedCollisionSum=0;
}

void Collisions::stop(void){
}

void Collisions::update(void){
  int colliding=0;
  for(int i=0;i<physics->getNumCollisions() && !colliding;i++){
    btPersistentManifold * col = physics->getCollision(i);

    //Get objects colliding
    btRigidBody* obA = static_cast<btRigidBody*>(col->getBody0());
    btRigidBody* obB = static_cast<btRigidBody*>(col->getBody1());  

    //Check if target is involved in collision
    CollisionDataType * data=(CollisionDataType *)obA->getUserPointer();  
    CollisionDataType * data2=(CollisionDataType *)obB->getUserPointer();

    if(target== data->getObjectName() || target==data2->getObjectName()){

      //Target is involved in collision, check contacts
      int numContacts= col->getNumContacts();
      double spColl=0; //Collision speed of contacts
      int realContacts=0;  //Number of contacts collisioning
      for (int j=0;j<numContacts ;j++){
	btManifoldPoint pt = col->getContactPoint(j);
	if (pt.getDistance()<0.f){ //Check contact points are near
	  btVector3 vec = obA->getVelocityInLocalPoint(pt.getPositionWorldOnA());  //velocity in contact point
	  btVector3 collision_direction=(pt.getPositionWorldOnA() - pt.getPositionWorldOnB()).normalize();
	  double colSpeed= collision_direction.dot(vec);  //speed in collision direction

 	  //std::cout<<"Velocidad A:"<<vec.length()<<" "<<vec.getX()<<" "<<vec.getY()<<" "<<vec.getZ()<<" "<<std::endl;
	  //std::cout<<"Direccion: "<<collision_direction.getX()<<" "<<collision_direction.getY()<<" "<<collision_direction.getZ()<<" "<<std::endl;
	  //std::cout<<"Velocidad choque: "<<colSpeed<<std::endl;

	  if(colSpeed>0){ //Check collision speed is positive
	    spColl+=colSpeed;
	    realContacts++;
          }
	}

      }
      if(realContacts>0){ //if collision happened update collision statistics
        colliding=1;
	double avgSpeed=spColl/realContacts;
	if(maxSpeedCollision<avgSpeed)
	  maxSpeedCollision=avgSpeed;
	speedCollisionSum+=avgSpeed;
      }
    }
  }
  ticks++;
  ticks_collision+=colliding;

}

double Collisions::getMeasure(void){

  //std::cout<<(double)ticks_collision/ticks<<" colliding maxSpeedCollision:"<<maxSpeedCollision<<" AvgCollision:"<<speedCollisionSum/ticks_collision<<std::endl;
  double score=1-speedCollisionSum/(ticks*MAX_COLLISION_SPEED)-maxSpeedCollision/MAX_COLLISION_SPEED;
  if(score>0)
    return score;
  return 0;
}

int Collisions::isOn(){
  return Measures::isOn();
}

void Collisions::reset(){
  Measures::reset();
  ticks_collision=0;
  ticks=1; //Avoid zero division
  maxSpeedCollision=0;
  speedCollisionSum=0;
}

int Collisions::error(){
  return 0;
}

//******************POSITION ERROR ***************

PositionError::PositionError(osg::Node * targ,double pos[3]){
  target=targ;
  position=osg::Vec3f(pos[0],pos[1],pos[2]);
  distance=100;
}

void PositionError::start(void){
}

void PositionError::update(void){
}

void PositionError::stop(void){
    osg::Vec3f newposition, newscale;
    osg::Quat newrotation, newrotationScale;

    getWorldCoords(target)->decompose(newposition, newrotation, newscale, newrotationScale);

    newposition=newposition-position;
    distance=newposition.length();
}

double PositionError::getMeasure(void){
  return distance;
}

int PositionError::isOn(){
  return Measures::isOn();
}

void PositionError::reset(){
  Measures::reset();
  distance=100;
}

int PositionError::error(){
  return 0;
}

//******************DISTANCE ***************

Distance::Distance(osg::Node * targ){
  target=targ;
  distance=0;

}

void Distance::start(void){
  osg::Vec3f newposition, newscale;
  osg::Quat newrotation, newrotationScale;

  getWorldCoords(target)->decompose(newposition, newrotation, newscale, newrotationScale);

  last=newposition;
}

void Distance::update(void){
  osg::Vec3f newposition, newscale;
  osg::Quat newrotation, newrotationScale;

  getWorldCoords(target)->decompose(newposition, newrotation, newscale, newrotationScale);

  distance+=(newposition-last).length();

  last=newposition;
}

void Distance::stop(void){
}

double Distance::getMeasure(void){
  return distance;
}

int Distance::isOn(){
  return Measures::isOn();
} 

void Distance::reset(){
  Measures::reset();
  distance=0;
}

int Distance::error(){
  return 0;
}

//******************EUCLIDEAN NORM ***************

EuclideanNorm::EuclideanNorm(std::vector<double> groundT, int nVal, std::string topic){
  nVals=nVal;
  groundTruth=groundT;
  this->topic=new ROSArrayToEuclideanNorm(topic);
  norm=-1; //error value
}

void EuclideanNorm::start(void){
  std::vector<double> estimated;

  norm=-1; //error value
  topic->getVector(estimated); //Clears previous measures on topic
}

void EuclideanNorm::update(void){
  std::vector<double> estimated;
  double acum=0;
  if(topic->getVector(estimated))
    if(estimated.size() == nVals){
      for(int i=0;i<estimated.size();i++)
        acum+=pow(groundTruth[i]-estimated[i],2);
      norm=pow(acum,0.5);
    }
    else{
      std::cout<<"Error: Estimated measure and groundTruth have different size on "<<name<<" measure."<<std::endl;
    }
}

void EuclideanNorm::stop(void){
}

double EuclideanNorm::getMeasure(void){
  return norm;
}

int EuclideanNorm::isOn(){
  return Measures::isOn();
}

void EuclideanNorm::reset(){
  Measures::reset();
  std::vector<double> estimated;

  norm=-1; //error value
  topic->getVector(estimated);
}

int EuclideanNorm::error(){
  if(norm==-1)
    return 1;
  return 0;
}
