#ifndef HANDS_DEMO
#define HANDS_DEMO

#include<ros/ros.h>
#include<yolo2/ImageDetections.h>
#include<cftld_ros/Track.h>
#include<hands_demo/tools.h>
#include<autonomy_human/human.h>

class HandsDemo{
public:
  HandsDemo( ros::NodeHandle& nh);
  ~HandsDemo();
  void Spin();
private:
  ros::NodeHandle nh_;

  //  Publishers:

  // To reset/initialize the visualal tracker for right hand
  ros::Publisher pub_cftld_right_hand_reset_;
  ros::Publisher pub_cftld_right_hand_init_;

  // To reset/initialize the visualal tracker for left hand
  ros::Publisher pub_cftld_left_hand_reset_;
  ros::Publisher pub_cftld_left_hand_init_;

  // To enable/disable
  ros::Publisher pub_yolo_enable_;
  ros::Publisher pub_human_enable_;

  //  Subscribers:
  behavior_tools::ASyncSub<cftld_ros::Track> sub_left_hand_tracker_;
  behavior_tools::ASyncSub<cftld_ros::Track> sub_right_hand_tracker_;
  behavior_tools::ASyncSub<yolo2::ImageDetections> sub_yolo_;
  behavior_tools::ASyncSub<autonomy_human::human> sub_human_;
};

#endif
