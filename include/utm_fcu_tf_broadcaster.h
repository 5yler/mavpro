 /**
 * utm_fcu_tf_broadcaster.h
 * Broadcast transform between local_origin and fcu. 
 * 
 * @author  Syler Wagner      <syler@mit.edu>

 * @date    2016-07-18  syler   creation
 *
 * This node listens for an initial odometry message on the 
 * /mavros/global_position/local topic and then broadcasts a 
 * transform between local_origin and fcu. 
 **/

#ifndef UTM_FCU_TF_BROADCASTER_H
#define UTM_FCU_TF_BROADCASTER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
// #include <mavros/mavros.h>

class LocalOriginToFCUBroadcaster
{
public:
  LocalOriginToFCUBroadcaster(ros::NodeHandle *n);  //$ constructor
  ~LocalOriginToFCUBroadcaster();                   //$ destructor
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom); 
  bool verifyOdometry(const nav_msgs::Odometry::ConstPtr& odom);

  void sendTransform();

  ros::NodeHandle* _n;

  std::string _frame_id;  //$ child frame_id (default: fcu)

  bool _transform_initialized;

  geometry_msgs::TransformStamped _odom_trans;  //$ transform
  
  tf::TransformBroadcaster _odom_broadcaster;   //$ transform broadcaster
  ros::Subscriber _odom_sub;                    //$ UTM odometry subscriber
  
  ros::Time _t_first_message;
  ros::Time _t_latest_error;
  int _message_count;
};

#endif 
