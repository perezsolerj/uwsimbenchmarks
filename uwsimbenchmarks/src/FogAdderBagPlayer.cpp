#include "FogAdderBagPlayer.h"


FogAdderBagPlayer::FogAdderBagPlayer(std::string bag, double dist, std::string image_topic,  std::string image_publisher, std::string info_topic, std::string info_publisher)
: BagPlayer::BagPlayer(bag), distance(dist), info(info_topic), image(image_topic)
{
  
  topics.push_back(image_topic);
  topics.push_back(info_topic);

  info_pub = nh.advertise<sensor_msgs::CameraInfo>(info_publisher, 1);
  image_pub = nh.advertise<sensor_msgs::Image>(image_publisher, 1);

  fogColor[1]=0.0*255;
  fogColor[2]=0.05*255;
  fogColor[3]= 0.3*255;

  newFogDensity(0);
}

void FogAdderBagPlayer::newFogDensity(double fogDensity){
  fogFactor=exp(-fogDensity*fogDensity*1.442695*distance*distance);
}

void FogAdderBagPlayer::preTimeWait(rosbag::MessageInstance const m){
  if (m.getTopic() == image){
    imageMsg = m.instantiate<sensor_msgs::Image>();
	
    for(int x=0; x < (imageMsg->width * imageMsg->height) ;x++){
      imageMsg->data[x*3] = imageMsg->data[x*3] * fogFactor + (1-fogFactor) * fogColor[1];
      imageMsg->data[x*3+1] = imageMsg->data[x*3+1] * fogFactor + (1-fogFactor) * fogColor[2];
      imageMsg->data[x*3+2] = imageMsg->data[x*3+2] * fogFactor + (1-fogFactor) * fogColor[3];
    }

  }
}

void FogAdderBagPlayer::postTimeWait(rosbag::MessageInstance const m){
  if (m.getTopic() == image){
    if (imageMsg)
      image_pub.publish(*imageMsg);
  }

  if (m.getTopic() == info){
    sensor_msgs::CameraInfo::ConstPtr infoMsg = m.instantiate<sensor_msgs::CameraInfo>();
    info_pub.publish(*infoMsg);
  }
}
