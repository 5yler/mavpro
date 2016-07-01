 /**
 * global_odom_to_tf.cpp
 * Broadcast transform between local_origin and fcu. 
 * 
 * @author  Syler Wagner      <syler@mit.edu>

 * @date    2016-07-01  syler   creation
 *
 * This node listens for odometry messages on the /mavros/global_position/local topic
 * and broadcasts a transform between local_origin and fcu. 
 **/

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

//$ TODO: use mavros.get_namespace() instead of hardcoding
 
 geometry_msgs::TransformStamped odom_trans;

 void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "local_origin";
  odom_trans.child_frame_id = "fcu";

  odom_trans.transform.translation.x = odom->pose.pose.position.x;
  odom_trans.transform.translation.y = odom->pose.pose.position.y;
  odom_trans.transform.translation.z = odom->pose.pose.position.z;
  odom_trans.transform.rotation = odom->pose.pose.orientation;

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_odom_to_tf");

  ros::NodeHandle n;
  tf::TransformBroadcaster odom_broadcaster;

  ros::Subscriber odom_sub = n.subscribe("/mavros/global_position/local", 1000, odometryCallback);
  ros::Rate r(1000.0);

  while (n.ok()) {
    ros::spinOnce();
    odom_broadcaster.sendTransform(odom_trans);
    r.sleep();
  }
}
