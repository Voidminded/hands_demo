#include "hands_demo/hands_demo.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

HandsDemo::HandsDemo(ros::NodeHandle &nh)
  : nh_( nh),
    sub_right_hand_tracker_(nh_, "gesture/right_hand/cftld/track", 1),
    sub_left_hand_tracker_(nh_, "gesture/left_hand/cftld/track", 1),
    sub_human_(nh, "human/human", 10),
    sub_yolo_(nh_, "yolo2/detections", 1),
    pub_cftld_right_hand_reset_(nh_.advertise<std_msgs::Empty>("gesture/right_hand/cftld/reset" , 1)),
    pub_cftld_right_hand_init_(nh_.advertise<sensor_msgs::RegionOfInterest>("gesture/right_hand/cftld/init_roi", 1)),
    pub_cftld_left_hand_reset_(nh_.advertise<std_msgs::Empty>("gesture/left_hand/cftld/reset" , 1)),
    pub_cftld_left_hand_init_(nh_.advertise<sensor_msgs::RegionOfInterest>("gesture/left_hand/cftld/init_roi", 1)),
    pub_yolo_enable_(nh_.advertise<std_msgs::Bool>("yolo2/enable", 1, true)),
    pub_human_enable_(nh_.advertise<std_msgs::Bool>("human/enable", 1, true))
{
}

HandsDemo::~HandsDemo()
{

}

void HandsDemo::Spin()
{
  //ros::Rate rate(update_rate_);
  std_msgs::Empty reset_msg_;
  pub_cftld_left_hand_reset_.publish(reset_msg_);
  pub_cftld_right_hand_reset_.publish(reset_msg_);
  std_msgs::Bool enable_msgs_;
  enable_msgs_.data = true;
  pub_yolo_enable_.publish( enable_msgs_);
  pub_human_enable_.publish( enable_msgs_);
  sub_left_hand_tracker_.DeactivateIfOlderThan(1.0);
  sub_right_hand_tracker_.DeactivateIfOlderThan(1.0);
  sub_human_.DeactivateIfOlderThan(1.0);
  sub_yolo_.DeactivateIfOlderThan(1.0);
  while (ros::ok())
  {
    if( (sub_left_hand_tracker_.IsActive()
         && sub_left_hand_tracker_()->status != cftld_ros::Track::STATUS_TRACKING)
        || (sub_right_hand_tracker_.IsActive()
            && sub_right_hand_tracker_()->status != cftld_ros::Track::STATUS_TRACKING)
        || !sub_left_hand_tracker_.IsActive()
        || !sub_right_hand_tracker_.IsActive())
    {
      if( sub_yolo_.IsActive()
          && sub_human_.IsActive())
      {
        yolo2::ImageDetections d = sub_yolo_.GetMsgCopy();
        autonomy_human::human h = sub_human_.GetMsgCopy();
        for( int i = 0; i < d.detections.size(); i++)
        {
          if( (!sub_left_hand_tracker_.IsActive()
               || ( sub_left_hand_tracker_.IsActive()
                    && ( sub_left_hand_tracker_()->status == cftld_ros::Track::STATUS_LOST
                         || sub_left_hand_tracker_()->status == cftld_ros::Track::STATUS_UNKNOWN)))
              && d.detections[i].roi.x_offset >= h.leftHROI.x_offset
              && d.detections[i].roi.x_offset < (h.leftHROI.x_offset + h.leftHROI.width)
              && d.detections[i].roi.y_offset >= h.leftHROI.y_offset
              && d.detections[i].roi.y_offset < (h.leftHROI.y_offset + h.leftHROI.height))
          {
            pub_cftld_left_hand_init_.publish( d.detections[i].roi);
          }
          if( (!sub_right_hand_tracker_.IsActive()
               || ( sub_right_hand_tracker_.IsActive()
                    && ( sub_right_hand_tracker_()->status == cftld_ros::Track::STATUS_LOST
                         || sub_right_hand_tracker_()->status == cftld_ros::Track::STATUS_UNKNOWN)))
              && d.detections[i].roi.x_offset >= h.rightHROI.x_offset
              && d.detections[i].roi.x_offset < (h.rightHROI.x_offset + h.rightHROI.width)
              && d.detections[i].roi.y_offset >= h.rightHROI.y_offset
              && d.detections[i].roi.y_offset < (h.rightHROI.y_offset + h.rightHROI.height))
          {
            pub_cftld_right_hand_init_.publish( d.detections[i].roi);
          }
        }
      }
    }
    ros::spinOnce();
    // TODO: Graceful shutdown:
    // http://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
    // if (!rate.sleep())
    // {
    //   ROS_WARN_STREAM("[ANIM] missed target loop rate of " << update_rate_ << " hz.");
    // }
  }

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "hands_demo_node");
  ros::NodeHandle nh;
  HandsDemo hd(nh);
  ROS_INFO("[HND] Demo node initialized...");
  hd.Spin();
  return 0;
}
