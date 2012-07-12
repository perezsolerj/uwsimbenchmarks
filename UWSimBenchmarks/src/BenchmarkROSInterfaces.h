#ifndef BENCHMARKROSINTERFACE_H_
#define BENCHMARKROSINTERFACE_H_

#include "ROSInterface.h"
class ServiceTrigger;
#include "Trigger.h"

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

class TopicTrigger;

class ROSTrigger: public ROSSubscriberInterface {
	TopicTrigger *trigger;
public:
	ROSTrigger(std::string topic, TopicTrigger *trigger);
	virtual void createSubscriber(ros::NodeHandle &nh);
	virtual void processData(const std_msgs::Bool::ConstPtr& msg);
	~ROSTrigger();
};

class ROSArrayToEuclideanNorm: public ROSSubscriberInterface {
	std::vector<double> estimated;
	int newMeasure;
public:
	ROSArrayToEuclideanNorm(std::string topic);
	virtual void createSubscriber(ros::NodeHandle &nh);
	virtual void processData(const std_msgs::Float32MultiArray::ConstPtr& msg);
        int getVector(std::vector<double> &estim);
	~ROSArrayToEuclideanNorm();
};

class ROSServiceTrigger {
  ServiceTrigger * trigger;
  ros::NodeHandle nh;
  ros::ServiceServer serv;
public:
	ROSServiceTrigger( std::string ,ServiceTrigger * trigger);

	virtual bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	~ROSServiceTrigger();
};




#endif
