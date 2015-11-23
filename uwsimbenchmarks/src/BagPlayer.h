#ifndef BAGPLAYER_H_
#define BAGPLAYER_H_

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>

class BagPlayer {
  private:
    ros::Time initLoopTime; //Init time of the bag loop.
    ros::Time initTime; //Bag's init time.

    rosbag::Bag bag; 
    rosbag::View  view;

    std::string name;

  protected:
    int loops;
    std::vector<std::string> topics;
    ros::NodeHandle nh;
  public:

    BagPlayer(std::string bag);
    void loop();
    virtual void preTimeWait(rosbag::MessageInstance const m)=0;
    virtual void postTimeWait(rosbag::MessageInstance const m)=0;
    void initLoop();
    void postLoop();
    void run();

};

#endif
