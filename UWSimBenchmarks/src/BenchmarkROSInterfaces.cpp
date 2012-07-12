#include "BenchmarkROSInterfaces.h"


ROSTrigger::ROSTrigger(std::string topic, TopicTrigger *trigger): ROSSubscriberInterface(topic) {
  this->trigger=trigger;
}

void ROSTrigger::createSubscriber(ros::NodeHandle &nh) {
  sub_ = nh.subscribe<std_msgs::Bool>(topic, 10, &ROSTrigger::processData, this);
}

void ROSTrigger::processData(const std_msgs::Bool::ConstPtr& msg) {
  if(msg->data==1)
    trigger->start();
  else{
    trigger->stop();
    //std::cout<<"TIMER: "<<timer->getMeasure()<<std::endl; //Testing
  }
}
ROSTrigger::~ROSTrigger(){}

ROSArrayToEuclideanNorm::ROSArrayToEuclideanNorm(std::string topic): ROSSubscriberInterface(topic) {
  newMeasure=0;
}

void ROSArrayToEuclideanNorm::createSubscriber(ros::NodeHandle &nh) {
  sub_ = nh.subscribe<std_msgs::Float32MultiArray>(topic, 10, &ROSArrayToEuclideanNorm::processData, this);
}

void ROSArrayToEuclideanNorm::processData(const std_msgs::Float32MultiArray::ConstPtr& msg) {
   int i=0;
   estimated.resize(msg->data.size());
   for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
     estimated[i++]=*it;
   newMeasure=1;
}

int ROSArrayToEuclideanNorm::getVector(std::vector<double> &estim){
  estim=estimated;
  if(newMeasure==1){
    newMeasure=0;
    return 1;
  }
  return 0;
}

ROSArrayToEuclideanNorm::~ROSArrayToEuclideanNorm(){}

ROSServiceTrigger::ROSServiceTrigger(std::string service,ServiceTrigger * trigger) {
  this->trigger=trigger;

  serv= nh.advertiseService( service, &ROSServiceTrigger::callback,this);
}

bool ROSServiceTrigger::callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  trigger->start();
  return true;
}

ROSServiceTrigger::~ROSServiceTrigger(){}

