#ifndef FOGADDERBAGPLAYER_H_
#define FOGADDERBAGPLAYER_H_

#include "BagPlayer.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

class FogAdderBagPlayer: public BagPlayer {
  private:
    double fogFactor, distance;
    double fogColor[3];

    ros::Publisher info_pub;
    ros::Publisher image_pub;

    std::string info, image;
    sensor_msgs::Image::Ptr imageMsg;
  protected:


  public:

    FogAdderBagPlayer(std::string bag, double dist, std::string image_topic, std::string image_publisher, std::string info_topic, std::string info_publisher);
    void newFogDensity(double fogDensity);

    void preTimeWait(rosbag::MessageInstance const m);
    void postTimeWait(rosbag::MessageInstance const m);
};

#endif
