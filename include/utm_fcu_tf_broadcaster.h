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

  std::string _ns;

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
