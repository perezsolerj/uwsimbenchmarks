
#include "BagPlayer.h"
#include <boost/foreach.hpp>
#include <sensor_msgs/Image.h>

BagPlayer::BagPlayer(std::string na) : name(na){
  loops=0;
}

void BagPlayer::loop(){
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    ros::Time const& time = m.getTime(); //Absolute time the message was sent
    ros::Duration messageTime= time-initTime; //Relative time the message was sent

    /*if (m.getTopic() == "/uwsim/camera1"){    
      std::cout<<"AAA THUNDER"<<std::endl;
      sensor_msgs::Image::Ptr imageMsg;
      imageMsg = m.instantiate<sensor_msgs::Image>();
    }
    std::cout<<"AAA"<<std::endl;*/
	

    preTimeWait(m);
    while( ((initLoopTime+messageTime)-ros::Time::now()).toSec() > 0 )
      ;
    postTimeWait(m);

  }

}

void BagPlayer::initLoop(){
  if(loops==0)
    bag.open(name, rosbag::bagmode::Read);
  view.addQuery(bag, rosbag::TopicQuery(topics));
  initTime=view.getBeginTime();

  initLoopTime = ros::Time::now();
  loops++;
}

void BagPlayer::postLoop(){
  //bag.close();
  view.begin();
}

void BagPlayer::run(){
  while(ros::ok()){
    initLoop();
    loop();
    postLoop();
  }
}
