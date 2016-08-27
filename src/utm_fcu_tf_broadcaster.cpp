/********************************************************************
  Software License Agreement (BSD License)

  Copyright (c) 2016, Syler Wagner.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above
     copyright notice, this list of conditions and the following
     disclaimer in the documentation and/or other materials provided
     with the distribution.
  3. Neither the name of the copyright holder nor the names of its 
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/**
 * utm_fcu_tf_broadcaster.cpp
 * Broadcast transform between local_origin and fcu. 
 * 
 * @author  Syler Wagner      <syler@mit.edu>

 * @date    2016-07-18  syler   creation
 *
 * This node listens for an initial odometry message on the 
 * /mavros/global_position/local topic and then broadcasts a 
 * transform between local_origin and fcu. 
 **/

#include <utm_fcu_tf_broadcaster.h>
#include <tf/transform_datatypes.h>

//$ TODO: use mavros.get_namespace() instead of hardcoding
 
LocalOriginToFCUBroadcaster::LocalOriginToFCUBroadcaster(ros::NodeHandle *n) {
  _n = n;

  _n->param<std::string>("frame_id", _frame_id, "fcu");
  _n->param<std::string>("mavros_ns", _ns, "mavros");

  ROS_WARN("Starting utm_fcu_tf_broadcaster with mavros namespace '%s'", _ns.c_str());

  // _n->param<std::string>("frame_id", _frame_id, "fcu");

  //$ transform is not initialized at startup
  _transform_initialized = false;
  _t_latest_error = ros::Time::now();
  _message_count = 0;

  ros::NodeHandle mavros_nh(_ns); //$ node handle for mavros global position topic

  mavros_nh.param<std::string>("local_position/frame_id", _frame_id, "fcu");
  mavros_nh.param<std::string>("global_position/frame_id", _utm_frame_id, "utm");

  ROS_WARN("Trying to initialize transform between %s and %s", _utm_frame_id.c_str(), _frame_id.c_str());

  //$ UTM odometry subscriber
  _odom_sub = mavros_nh.subscribe("global_position/local", 1, &LocalOriginToFCUBroadcaster::odometryCallback, this);
}

LocalOriginToFCUBroadcaster::~LocalOriginToFCUBroadcaster() {}

bool LocalOriginToFCUBroadcaster::verifyOdometry(const nav_msgs::Odometry::ConstPtr& odom) {
  /*$ 
    The first few messages on /mavros/global_position/local are usually invalid and will usually be something like
      x:  166021.443179
      y:  0.000000
      z:  0.210000
    so the ones with bogus values need to be filtered out.
    */

    ros::Duration time_since_first_callback;

    if (_message_count == 1) {
      _t_first_message = ros::Time::now();
      ROS_WARN("Got first message on %s/global_position/local", _ns.c_str());
    }
    else {
      time_since_first_callback = ros::Time::now() - _t_first_message;
    }

    if (odom->pose.pose.position.y < 1e-6) {
      ROS_ERROR("UTM messages on %s/global_position/local are invalid. Wait a second...", _ns.c_str());
      return false;
    }
    else if (time_since_first_callback < ros::Duration(1)) {
      return false;
    }
    else {
      return true;
    }
}

/*$ 
 * The odometryCallback() function initializes the transform between 
 * local_origin and fcu. The data from the /mavros/global_position/local
 * topic is only read once.
 */
void LocalOriginToFCUBroadcaster::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  
  _message_count += 1;

  if (_transform_initialized == false) {
    if (verifyOdometry(odom)) {
      _odom_trans.transform.translation.x = odom->pose.pose.position.x;
      _odom_trans.transform.translation.y = odom->pose.pose.position.y;
      _odom_trans.transform.translation.z = odom->pose.pose.position.z;
      //$ TODO: I don't think the rotation is necessary
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
      _odom_trans.transform.rotation = odom_quat;

      ROS_WARN("Transform between %s and %s initialized!", _utm_frame_id.c_str(), _frame_id.c_str());
      ROS_WARN("Translation: [x: %4.1f, y: %4.1f, z:  %4.1f]", _odom_trans.transform.translation.x, _odom_trans.transform.translation.y, _odom_trans.transform.translation.z);
      ROS_WARN("Rotation: [x: %4.1f, y: %4.1f, z:  %4.1f, w:  %4.1f]", _odom_trans.transform.rotation.x, _odom_trans.transform.rotation.y, _odom_trans.transform.rotation.z, _odom_trans.transform.rotation.w);

      _transform_initialized = true;
    }
  }
}

/*$ 
 * Send the transform between local_origin and fcu after it has been initialized.
 */
void  LocalOriginToFCUBroadcaster::sendTransform() {
  if (_transform_initialized) {
    _odom_trans.header.stamp = ros::Time::now();
    _odom_trans.header.frame_id = _utm_frame_id;
    _odom_trans.child_frame_id = _frame_id;
    _odom_broadcaster.sendTransform(_odom_trans);
  } 
  else {
    ros::Duration time_since_last_error_message = ros::Time::now() - _t_latest_error;
    if (time_since_last_error_message > ros::Duration(5)) {
      _t_latest_error = ros::Time::now();
      ROS_ERROR("No UTM odometry messages received from UAV. Are you in the air yet?");
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "utm_fcu_tf_broadcaster");

  ros::NodeHandle* n = new ros::NodeHandle("~");
  LocalOriginToFCUBroadcaster broadcaster(n);

  int rate;
  std::string frame_id;

  n->param("rated", rate, 100);

  ros::Rate r(rate);

  while (n->ok()) {
    ros::spinOnce();
    broadcaster.sendTransform();
    r.sleep();
  }
}
