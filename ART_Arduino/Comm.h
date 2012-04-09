#ifndef _COMM_H_
#define _COMM_H_

#ifdef USE_ROS
  #include <ros.h>
#endif

class Comm {
 public:

  void init();
#ifdef USE_ROS

  tf::TransformBroadcaster pub_tf;
  
  geometry_msgs::TransformStamped tf_imu;

  char header_frame_tf_euler[] = "/imu",
       child_frame_tf_euler[] = "/world";
  
  void publish_tf_imu ();

#else // Serial
  void publish_imu_raw ();
  void publish_euler ();
#endif
} comm;

#endif
