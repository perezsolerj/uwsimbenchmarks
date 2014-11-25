#include "Measures.h"

#include <pcl/surface/convex_hull.h>
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

void Measures::setLog(double log){
  this->log=log;
}

void Measures::reset(){
  startOn->reset();
  stopOn->reset();
}

std::vector<double> Measures::getMeasureDetails(void){
  std::vector<double> vector;
  vector.resize(1);
  vector[0]=getMeasure();
  return vector;
}

std::vector<std::string> Measures::getNameDetails(void){
  std::vector<std::string> vector;
  vector.resize(1);
  vector[0]=name;
  return vector;
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

    #if BT_BULLET_VERSION <= 279
    //Get objects colliding
    btRigidBody* obA = static_cast<btRigidBody*>(col->getBody0());
    btRigidBody* obB = static_cast<btRigidBody*>(col->getBody1());

    //Check if target is involved in collision
    CollisionDataType * data = (CollisionDataType *)obA->getUserPointer();
    CollisionDataType * data2 = (CollisionDataType *)obB->getUserPointer();

    #else
    //Get objects colliding
    const btRigidBody* obA = btRigidBody::upcast(col->getBody0());
    const btRigidBody* obB = btRigidBody::upcast(col->getBody1());

    //Check if target is involved in collision
    CollisionDataType * data = (CollisionDataType *)col->getBody0()->getUserPointer();
    CollisionDataType * data2 = (CollisionDataType *)col->getBody0()->getUserPointer();
    #endif

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


  //-------ObjectCornersInCam------

EuclideanNorm::ObjectCornersInCam::ObjectCornersInCam(osg::Camera * cam,osg::Node * target){
  this->cam=cam;
  this->target=target;
}

std::vector<double> EuclideanNorm::ObjectCornersInCam::getGT(){
  std::vector<double> groundTruth;
  groundTruth.resize(8);

  //std::cout<<"minX: "<<minX<<" minY: "<<minY<<std::endl;
  osg::Matrix MVPW( cam->getViewMatrix() * cam->getProjectionMatrix() * cam->getViewport()->computeWindowMatrix());

  osg::ComputeBoundsVisitor cbv;
  target->accept(cbv);
  osg::BoundingBox box = cbv.getBoundingBox(); 

  boost::shared_ptr<osg::Matrixd> pos=getWorldCoords(target); 

  //std::cout<<"BOX: "<<box.xMin()<<" "<<box.xMax()<<" "<<box.yMin()<<" "<<box.yMax()<<" "<<box.zMin()<<" "<<box.zMax()<<" "<<std::endl;
  //std::cout<<"BOX position: "<<pos->getTrans().x()<<" "<<pos->getTrans().y()<<" "<<pos->getTrans().z()<<" "<<std::endl;

  //Create a convexHull with all the points in 2D.
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setDimension(2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), newcloud (new pcl::PointCloud<pcl::PointXYZ>);

  osg::Vec3 posIn2D;

  for(int i=1;i<9;i++){
    osg::Vec3 corner(box.xMin(),box.yMin(),box.zMin());
    if(i%2==0)
      corner.x()=box.xMax();
    if((i-1)%4>1)
      corner.y()=box.yMax();
    if(i>4)
      corner.z()=box.zMax();
    posIn2D = (pos->getTrans()+pos->getRotate()*corner) * MVPW;
    //std::cout<<"2D POINT("<<i<<"): "<<posIn2D.x()<<" "<<posIn2D.y()<<" "<<posIn2D.z()<<" "<<std::endl;
    cloud->push_back(pcl::PointXYZ(posIn2D.x(),posIn2D.y(),posIn2D.z()));
  }

  chull.setInputCloud(cloud);
  chull.reconstruct(*newcloud);

  //Get the minimun bounding box from convex hull (Can be improved with rotating calipers from o(n^2) to o(n), but there are only 8 points, so it shouldn't be worthy)
  //TODO: Initial min max can be improved with camera parameters...

  double minXMax=-9999,minXMin=9999,minYMax=-9999,minYMin=9999,minM=0,minArea=999999999;
  for( pcl::PointCloud<pcl::PointXYZ>::iterator i= newcloud->begin(); i!=newcloud->end();i++){
    //Considering edge i->i+1 to find minimun bounding box
    pcl::PointCloud<pcl::PointXYZ>::iterator next=(i+1);
    if(next==newcloud->end())
      next=newcloud->begin();
    //std::cout<<"CHULL "<<i->x<<" "<<i->y<<" "<<i->z<<std::endl;
    //std::cout<<"CHULL NEXT "<<next->x<<" "<<next->y<<" "<<next->z<<std::endl;
    double m=(next->y-i->y)/(next->x-i->x);
    //std::cout<<"Pendiente "<<m<<" "<<atan(m)<<std::endl;
    //Rotate points and find maxX,minX,maxY,minY
    double xMax=-9999,xMin=9999,yMax=-9999,yMin=9999;
    for( pcl::PointCloud<pcl::PointXYZ>::iterator j= newcloud->begin(); j!=newcloud->end();j++){
      double newX=(j->x*cos(atan(-m)))-(j->y*sin(atan(-m)));
      double newY=(j->x*sin(atan(-m)))+(j->y*cos(atan(-m)));
      //std::cout<<"Transformacion: "<<j->x<<" "<<j->y<<" "<<newX<<" "<<newY<<std::endl;
      if(newX>xMax)
	xMax=newX;
      if(newX<xMin)
	xMin=newX;
      if(newY>yMax)
	yMax=newY;
      if(newY<yMin)
	yMin=newY;
    }
    if(((xMax-xMin)*(yMax-yMin))<minArea){
      minXMax=xMax;
      minXMin=xMin;
      minYMax=yMax;
      minYMin=yMin;
      minM=m;
      minArea=(xMax-xMin)*(yMax-yMin);
    }
  }

  //Get corners of minimun bounding box and rotate them.
  double BBSW_x=(minXMin*cos(atan(minM)))-(minYMin*sin(atan(minM)));
  double BBSW_y=(minXMin*sin(atan(minM)))+(minYMin*cos(atan(minM)));

  double BBSE_x=(minXMax*cos(atan(minM)))-(minYMin*sin(atan(minM)));
  double BBSE_y=(minXMax*sin(atan(minM)))+(minYMin*cos(atan(minM)));

  double BBNE_x=(minXMax*cos(atan(minM)))-(minYMax*sin(atan(minM)));
  double BBNE_y=(minXMax*sin(atan(minM)))+(minYMax*cos(atan(minM)));

  double BBNW_x=(minXMin*cos(atan(minM)))-(minYMax*sin(atan(minM)));
  double BBNW_y=(minXMin*sin(atan(minM)))+(minYMax*cos(atan(minM)));

  //std::cout<<"Final: "<<BBSW_x<<" "<<BBSW_y<<" "<<BBSE_x<<" "<<BBSE_y<<" "<<BBNE_x<<" "<<BBNE_y<<" "<<BBNW_x<<" "<<BBNW_y<<std::endl;
  

  //std::cout<<"minX:"<<minX<<" maxX:"<<maxX<<"minY:"<<minY<<" maxY:"<<maxY<<std::endl;

  //Image origin is on the bottom-left looking towards up-right, whereas ROS image origin on
  //top-left looking towards bottom-right. So points are being manually changed.
 
  groundTruth[0]=BBNW_x;
  groundTruth[1]=cam->getViewport()->height()-BBNW_y;
  groundTruth[2]=BBNE_x;
  groundTruth[3]=cam->getViewport()->height()-BBNE_y;
  groundTruth[4]=BBSE_x;
  groundTruth[5]=cam->getViewport()->height()-BBSE_y;
  groundTruth[6]=BBSW_x;
  groundTruth[7]=cam->getViewport()->height()-BBSW_y;

  //std::cout<<"GT: "<< groundTruth[0]<<" "<< groundTruth[1]<<" "<< groundTruth[2]<<" "<< groundTruth[3]<<" "<< groundTruth[4]<<" "<< groundTruth[5]<<" "<< groundTruth[6]<<" "<< groundTruth[7]<<" "<<std::endl;

  return groundTruth;
}

//-------ObjectCentroidInCam------

EuclideanNorm::ObjectCentroidInCam::ObjectCentroidInCam(osg::Camera * cam,osg::Node * target){
  this->cam=cam;
  this->target=target;
}

std::vector<double> EuclideanNorm::ObjectCentroidInCam::getGT(){
  std::vector<double> groundTruth;
  groundTruth.resize(2);

  osg::Matrix MVPW( cam->getViewMatrix() * cam->getProjectionMatrix() * cam->getViewport()->computeWindowMatrix());

  osg::ComputeBoundsVisitor cbv;
  target->accept(cbv);
  osg::BoundingBox box = cbv.getBoundingBox(); 

  boost::shared_ptr<osg::Matrixd> pos=getWorldCoords(target); 

  osg::Vec3 posIn2D = pos->getTrans() * MVPW;

  //Image origin is on the bottom-left looking towards up-right, whereas ROS image origin on
  //top-left looking towards bottom-right. So points are being manually changed.
  groundTruth[0]=posIn2D.x();
  groundTruth[1]=cam->getViewport()->height()-posIn2D.y();

  //std::cout<<"GT: "<< groundTruth[0]<<" "<< groundTruth[1]<<std::endl;

  return groundTruth;
}

//-------RelativeLocation------

EuclideanNorm::RelativeLocation::RelativeLocation(osg::Node * from,osg::Node * to){
  this->from=from;
  this->to=to;
}

std::vector<double> EuclideanNorm::RelativeLocation::getGT(){
  std::vector<double> groundTruth;
  groundTruth.resize(3);

  boost::shared_ptr<osg::Matrixd> fromMat=getWorldCoords(from); 
  boost::shared_ptr<osg::Matrixd> toMat=getWorldCoords(to); 
  fromMat->invert(*fromMat);

  osg::Matrixd  res=*toMat * *fromMat;

  groundTruth[0]=res.getTrans().x();
  groundTruth[1]=res.getTrans().y();
  groundTruth[2]=res.getTrans().z();

  //std::cout<<"GT: "<< groundTruth[0]<<" "<< groundTruth[1]<<" "<< groundTruth[2]<<std::endl;

  return groundTruth;
}


//-------EuclideanNorm------

EuclideanNorm::EuclideanNorm(GT * groundT, std::string topic, std::string publishOn){
  gt=groundT;
  this->topic=new ROSArrayToEuclideanNorm(topic);
  norm=-1; //error value
  this->publishOn=publishOn;
}

void EuclideanNorm::start(void){
  std::vector<double> estimated;
  norm=-1; //error value
  topic->getVector(estimated); //Clears previous measures on topic
  if(publishOn!=""){
    estimated=gt->getGT();
    uwsimbenchmarks::GTpublish::Request req;
    req.groundTruth.clear();
    for(int i=0; i< estimated.size();i++)
      req.groundTruth.push_back(estimated[i]);
    uwsimbenchmarks::GTpublish::Response res;
    ros::service::call(publishOn,req, res);
  }
}

void EuclideanNorm::update(void){
}

void EuclideanNorm::stop(void){
}

double EuclideanNorm::getMeasure(void){
  std::vector<double> estimated, groundTruth=gt->getGT();
  double acum=0;
  if(topic->getVector(estimated)){
    if(estimated.size() == groundTruth.size()){
      for(unsigned int i=0;i<estimated.size();i++){
        acum+=pow(groundTruth[i]-estimated[i],2);
      }
      norm=pow(acum,0.5);
    }
    else{
      std::cout<<"Error: Estimated measure and groundTruth have different size on "<<name<<" measure. "<<estimated.size()<<" "<<groundTruth.size()<<std::endl;
    }
  }
  return norm;
}

std::vector<double> EuclideanNorm::getMeasureDetails(void){
  std::vector<double> estimated, groundTruth=gt->getGT();
  double acum=0;
  std::vector<double> results;
  results.resize(groundTruth.size()+1);
  if(topic->getVector(estimated)){
    if(estimated.size() == groundTruth.size()){
      for(unsigned int i=0;i<estimated.size();i++){
        results[i+1]=groundTruth[i]-estimated[i];
        acum+=pow(results[i+1],2);
      }
      norm=pow(acum,0.5);
    }
    else{
      std::cout<<"Error: Estimated measure and groundTruth have different size on "<<name<<" measure. "<<estimated.size()<<" "<<groundTruth.size()<<std::endl;
    }
  }

  results[0]=norm;
  return results;
}

std::vector<std::string> EuclideanNorm::getNameDetails(void){
  std::vector<std::string> results;
  results.resize(gt->getGT().size()+1);
  
  results[0]=name+"_total";
  for(unsigned int i=1;i<results.size();i++)
    results[i]=name+"["+"]"; //Falta añadir el número!

  return results;
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

//******************OBJECT CENTERED ON CAM ***************

ObjectCenteredOnCam::ObjectCenteredOnCam(osg::Camera * cam,osg::Node * target){
  this->cam=cam;
  this->target=target;
}

void ObjectCenteredOnCam::start(void){

}

void ObjectCenteredOnCam::update(void){
}

void ObjectCenteredOnCam::stop(void){
}

double ObjectCenteredOnCam::getMeasure(void){
  osg::Matrix MVPW( cam->getViewMatrix() * cam->getProjectionMatrix() * cam->getViewport()->computeWindowMatrix());

  osg::ComputeBoundsVisitor cbv;
  target->accept(cbv);
  osg::BoundingBox box = cbv.getBoundingBox(); 

  boost::shared_ptr<osg::Matrixd> pos=getWorldCoords(target); 

  osg::Vec3 posIn2D = pos->getTrans() * MVPW;

  double measure=sqrt((cam->getViewport()->width()/2-posIn2D.x())*(cam->getViewport()->width()/2-posIn2D.x())+(cam->getViewport()->height()/2-posIn2D.y())*(cam->getViewport()->height()/2-posIn2D.y()));

  return measure;
}

std::vector<double> ObjectCenteredOnCam::getMeasureDetails(void){
  osg::Matrix MVPW( cam->getViewMatrix() * cam->getProjectionMatrix() * cam->getViewport()->computeWindowMatrix());

  osg::ComputeBoundsVisitor cbv;
  target->accept(cbv);
  osg::BoundingBox box = cbv.getBoundingBox(); 

  boost::shared_ptr<osg::Matrixd> pos=getWorldCoords(target); 

  osg::Vec3 posIn2D = pos->getTrans() * MVPW;

  std::vector<double> results;

  results.resize(3);
  results[0]=sqrt((cam->getViewport()->width()/2-posIn2D.x())*(cam->getViewport()->width()/2-posIn2D.x())+(cam->getViewport()->height()/2-posIn2D.y())*(cam->getViewport()->height()/2-posIn2D.y()));
  results[1]=cam->getViewport()->width()/2-posIn2D.x();
  results[2]=cam->getViewport()->height()/2-posIn2D.y();

  return results;
}

std::vector<std::string> ObjectCenteredOnCam::getNameDetails(void){
  std::vector<std::string> names;

  names.resize(3);
  names[0]=name+"_total";
  names[1]=name+"_x";
  names[2]=name+"_y";

  return names;
}

int ObjectCenteredOnCam::isOn(){
  return Measures::isOn();
}

void ObjectCenteredOnCam::reset(){
  Measures::reset();
}

int ObjectCenteredOnCam::error(){ //TODO: check if target is outside the camera¿?.
  return 0;
}

//******************Reconstruction 3D ***************

Reconstruction3D::Reconstruction3D(std::string topic, osg::Node * target, double resolution){
  this->topic=new ROSPointCloudTo3DReconstruction(topic);
  this->target=target;

  osg::ComputeBoundsVisitor cbv;
  target->accept(cbv);
  box = cbv.getBoundingBox();


  //Adjust resolution to closest multiple for each axis
  this->resolution[0]=resolution + fmod((double)(box.xMax()-box.xMin()),resolution) / (double)floor((box.xMax()-box.xMin())/resolution);
  this->resolution[1]=resolution + fmod((double)(box.yMax()-box.yMin()),resolution) / (double)floor((box.yMax()-box.yMin())/resolution);
  this->resolution[2]=resolution + fmod((double)(box.zMax()-box.zMin()),resolution) / (double)floor((box.zMax()-box.zMin())/resolution);
  
  occupancyDim[0]=(box.xMax()-box.xMin())/(this->resolution[0])+1;
  occupancyDim[1]=(box.yMax()-box.yMin())/(this->resolution[1])+1;
  occupancyDim[2]=(box.zMax()-box.zMin())/(this->resolution[2])+1;

  //set up occupancy matrix
  occupancy= new bool**[occupancyDim[0]];
  for(int i=0;i<occupancyDim[0];i++){
    occupancy[i]=new bool*[occupancyDim[1]];
    for(int j=0;j<occupancyDim[1];j++){
      occupancy[i][j]=new bool[occupancyDim[2]];
      memset(occupancy[i][j], false, occupancyDim[2] * sizeof(bool));
    }
  }

  //Surface points (not taking into account ground face)
  gridPoints=occupancyDim[0]*occupancyDim[1]*2+occupancyDim[0]*occupancyDim[2]+occupancyDim[1]*occupancyDim[2]*2;

  //Substract shared faces points + corners
  gridPoints=gridPoints - (occupancyDim[0] *2) - (occupancyDim[1]*4) - (occupancyDim[2]*2) +4;

  npoints=0;
  meanError=0;
  errorVariance=0;
  gridOccupiedPoints=0;
}

void Reconstruction3D::start(void){
  std::vector<osg::Vec3f> points;
  topic->get3DPoints(points); //clear 3DPoints
}

void Reconstruction3D::update(void){
}

void Reconstruction3D::stop(void){
}

void Reconstruction3D::processPoints(void){
  //Do something
  boost::shared_ptr<osg::Matrixd> pos=getWorldCoords(target); 

  std::vector<osg::Vec3f> points;
  topic->get3DPoints(points);
  for(int i=0;i<points.size();i++){
    //Translate points from world frame to object frame
    //std::cout<< points[i].x();

    //Distance point to box
    double dx=std::max(std::max((double)box.xMin() - points[i].x(), 0.0), (double)points[i].x() - box.xMax());
    double dy=std::max(std::max((double)box.yMin() - points[i].y(), 0.0), (double)points[i].y() - box.yMax());
    double dz=std::max(std::max((double)box.zMin() - points[i].z(), 0.0), (double)points[i].z() - box.zMax());

    double distancePointObject=sqrt(dx*dx+dy*dy+dz*dz);

    double minLinearDistance=std::min( std::min(abs(box.xMin() - points[i].x()),abs(box.xMax() - points[i].x())),
                             std::min( std::min(abs(box.yMin() - points[i].y()),abs(box.yMax() - points[i].y())),
                                        std::min(abs(box.zMin() - points[i].z()),abs(box.zMax() - points[i].z()))));

    double distancePointSurface=std::max(distancePointObject,minLinearDistance);

    //Distance ground plane to Object, we guess there is a ground plane under the object, these points must be discarded as error

    //By default ground plane is on zMin

    if(abs(box.zMin() - points[i].z())>=distancePointSurface and abs(box.zMin() - points[i].z())< 0.1){  //We consider the point to further processing
      npoints++;
      double delta=distancePointSurface-meanError;
      meanError=meanError + delta/npoints;
      errorVariance= errorVariance + delta * (distancePointSurface - meanError);
    }

    //CHECK occupancy grid points
    double gridProjected[3];
    gridProjected[0]=(points[i].x()-box.xMin())/resolution[0];
    gridProjected[1]=(points[i].y()-box.yMin())/resolution[1];
    gridProjected[2]=(points[i].z()-box.zMin())/resolution[2];

    //set to true nearest points
    if(0<= floor(gridProjected[0]) and floor(gridProjected[0]) < occupancyDim[0]){
      if(0<= floor(gridProjected[1]) and floor(gridProjected[1]) < occupancyDim[1]){
        if(0<=floor(gridProjected[2]) and floor(gridProjected[2]) < occupancyDim[2]){

          if( occupancy[(int)floor(gridProjected[0])][(int)floor(gridProjected[1])][(int)floor(gridProjected[2])]==false and
                (floor(gridProjected[0])==0 or floor(gridProjected[0])== occupancyDim[0]-1 or
                floor(gridProjected[1])==0 or floor(gridProjected[1])== occupancyDim[1]-1 or
                  floor(gridProjected[2])==occupancyDim[2]-1))
            gridOccupiedPoints++;

          occupancy[(int)floor(gridProjected[0])][(int)floor(gridProjected[1])][(int)floor(gridProjected[2])]=true;
        }
        if(0<= ceil(gridProjected[2]) and ceil(gridProjected[2]) < occupancyDim[2]){

          if( occupancy[(int)floor(gridProjected[0])][(int)floor(gridProjected[1])][(int)ceil(gridProjected[2])]==false and
                (floor(gridProjected[0])==0 or floor(gridProjected[0])== occupancyDim[0]-1 or
                floor(gridProjected[1])==0 or floor(gridProjected[1])== occupancyDim[1]-1 or
                  ceil(gridProjected[2])==occupancyDim[2]-1))
            gridOccupiedPoints++;

          occupancy[(int)floor(gridProjected[0])][(int)floor(gridProjected[1])][(int)ceil(gridProjected[2])]=true;
        }
      }
      if(0<=ceil(gridProjected[1]) and ceil(gridProjected[1]) < occupancyDim[1]){
        if(0<=floor(gridProjected[2]) and floor(gridProjected[2]) < occupancyDim[2]){

          if( occupancy[(int)floor(gridProjected[0])][(int)ceil(gridProjected[1])][(int)floor(gridProjected[2])]==false and
                (floor(gridProjected[0])==0 or floor(gridProjected[0])== occupancyDim[0]-1 or
                ceil(gridProjected[1])==0 or ceil(gridProjected[1])== occupancyDim[1]-1 or
                  floor(gridProjected[2])==occupancyDim[2]-1))
            gridOccupiedPoints++;

          occupancy[(int)floor(gridProjected[0])][(int)ceil(gridProjected[1])][(int)floor(gridProjected[2])]=true;
        }
        if(0<=ceil(gridProjected[2]) and ceil(gridProjected[2])<occupancyDim[2]){

          if( occupancy[(int)floor(gridProjected[0])][(int)ceil(gridProjected[1])][(int)ceil(gridProjected[2])]==false and
                (floor(gridProjected[0])==0 or floor(gridProjected[0])== occupancyDim[0]-1 or
                ceil(gridProjected[1])==0 or ceil(gridProjected[1])== occupancyDim[1]-1 or
                  ceil(gridProjected[2])==occupancyDim[2]-1))
            gridOccupiedPoints++;

          occupancy[(int)floor(gridProjected[0])][(int)ceil(gridProjected[1])][(int)ceil(gridProjected[2])]=true;
        }
      } 
    }

    if(0<=ceil(gridProjected[0]) and ceil(gridProjected[0]) < occupancyDim[0]){
      if(0<=floor(gridProjected[1]) and floor(gridProjected[1]) < occupancyDim[1]){
        if(0<=floor(gridProjected[2]) and floor(gridProjected[2]) < occupancyDim[2]){

          if( occupancy[(int)ceil(gridProjected[0])][(int)floor(gridProjected[1])][(int)floor(gridProjected[2])]==false and
                (ceil(gridProjected[0])==0 or ceil(gridProjected[0])== occupancyDim[0]-1 or
                floor(gridProjected[1])==0 or floor(gridProjected[1])== occupancyDim[1]-1 or
                  floor(gridProjected[2])==occupancyDim[2]-1))
            gridOccupiedPoints++;

          occupancy[(int)ceil(gridProjected[0])][(int)floor(gridProjected[1])][(int)floor(gridProjected[2])]=true;
        }
        if(0<=ceil(gridProjected[2]) and ceil(gridProjected[2]) < occupancyDim[2]){

          if( occupancy[(int)ceil(gridProjected[0])][(int)floor(gridProjected[1])][(int)ceil(gridProjected[2])]==false and
                (ceil(gridProjected[0])==0 or ceil(gridProjected[0])== occupancyDim[0]-1 or
                floor(gridProjected[1])==0 or floor(gridProjected[1])== occupancyDim[1]-1 or
                  ceil(gridProjected[2])==occupancyDim[2]-1))
            gridOccupiedPoints++;

          occupancy[(int)ceil(gridProjected[0])][(int)floor(gridProjected[1])][(int)ceil(gridProjected[2])]=true;
        }
      }
      if(0<=ceil(gridProjected[1]) and ceil(gridProjected[1]) < occupancyDim[1]){
        if(0<=floor(gridProjected[2]) and floor(gridProjected[2]) < occupancyDim[2]){

          if( occupancy[(int)ceil(gridProjected[0])][(int)ceil(gridProjected[1])][(int)floor(gridProjected[2])]==false and
                (ceil(gridProjected[0])==0 or ceil(gridProjected[0])== occupancyDim[0]-1 or
                ceil(gridProjected[1])==0 or ceil(gridProjected[1])== occupancyDim[1]-1 or
                  floor(gridProjected[2])==occupancyDim[2]-1))
            gridOccupiedPoints++;

          occupancy[(int)ceil(gridProjected[0])][(int)ceil(gridProjected[1])][(int)floor(gridProjected[2])]=true;
        }
        if(0<=ceil(gridProjected[2]) and ceil(gridProjected[2]) < occupancyDim[2]){

          if( occupancy[(int)ceil(gridProjected[0])][(int)ceil(gridProjected[1])][(int)ceil(gridProjected[2])]==false and
                (ceil(gridProjected[0])==0 or ceil(gridProjected[0])== occupancyDim[0]-1 or
                ceil(gridProjected[1])==0 or ceil(gridProjected[1])== occupancyDim[1]-1 or
                  ceil(gridProjected[2])==occupancyDim[2]-1))
            gridOccupiedPoints++;

          occupancy[(int)ceil(gridProjected[0])][(int)ceil(gridProjected[1])][(int)ceil(gridProjected[2])]=true;
        }
      } 
    } 
    
  }//END FOR POINTS
}

double Reconstruction3D::getMeasure(void){
  processPoints();
  return meanError;//errorVariance/(npoints-1);
}

std::vector<double> Reconstruction3D::getMeasureDetails(void){ //Return meanError, errorVariance & coverage%
  processPoints();
  std::vector<double> results;
  results.push_back(meanError);
  results.push_back(errorVariance/(npoints-1));
  results.push_back(gridOccupiedPoints/(double)gridPoints);
  return results;
}

std::vector<std::string> Reconstruction3D::getNameDetails(void){
  std::vector<std::string> results;
  results.push_back("mean error");
  results.push_back("error variance");
  results.push_back("coverage %");

  return results;
}

int Reconstruction3D::isOn(){
  return Measures::isOn();
}

void Reconstruction3D::reset(){
  Measures::reset();

  npoints=0;
  meanError=0;
  errorVariance=0;
  gridOccupiedPoints=0;

  //Restart occupancy matrix
  for(int i=0;i<occupancyDim[0];i++){
    for(int j=0;j<occupancyDim[1];j++){
      memset(occupancy[i][j], false, occupancyDim[2] * sizeof(bool));
    }
  }
}

int Reconstruction3D::error(){
  return 0;
}

