#ifndef BENCHMARKROSINTERFACE_H_
#define BENCHMARKROSINTERFACE_H_

#include <uwsim/ROSInterface.h>
class ServiceTrigger;
#include "Trigger.h"

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/WrenchStamped.h>
#include <topic_tools/shape_shifter.h>
#include <pcl_ros/point_cloud.h>
#include "Current.h"

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

class ROSPointCloudTo3DReconstruction : public ROSSubscriberInterface
{
  std::vector<osg::Vec3f> points;
public:
  ROSPointCloudTo3DReconstruction(std::string topic);
  virtual void createSubscriber(ros::NodeHandle &nh);
  virtual void processData(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
  int get3DPoints(std::vector<osg::Vec3f> &points);
  ~ROSPointCloudTo3DReconstruction();
};

/*class ROSTopicToShapeShifter: public ROSSubscriberInterface {

public:
	ROSTopicToShapeShifter(std::string topic);
	virtual void createSubscriber(ros::NodeHandle &nh);
	virtual void processData(const  topic_tools::ShapeShifter::ConstPtr& msg);
	~ROSTopicToShapeShifter();
};*/

class BenchmarkInfoToROSString : public ROSPublisherInterface
{
  std::string stringToPublish;
public:
  BenchmarkInfoToROSString(std::string topic, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  void changeMessage(std::string newString);

  ~BenchmarkInfoToROSString();
};

class CurrentToROSWrenchStamped : public ROSPublisherInterface
{
  boost::shared_ptr<Current> current;
  SimulatedIAUV *  vehicle;
public:
  CurrentToROSWrenchStamped(std::string topic, int rate,   boost::shared_ptr<Current> current, SimulatedIAUV *  vehicle);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~CurrentToROSWrenchStamped();
};

class BenchmarkResultToROSFloat32MultiArray : public ROSPublisherInterface
{
  std::vector<float> toPublish;
  int publishing;
public:
  BenchmarkResultToROSFloat32MultiArray(std::string topic, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  void newDataToPublish(std::vector<double> data);

  ~BenchmarkResultToROSFloat32MultiArray();
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
