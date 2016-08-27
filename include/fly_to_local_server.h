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
 * fly_to_local_server.h
 * Action server for UAV FlyToLocal action
 * 
 * @author  Syler Wagner      <syler@mit.edu>
 * @date    2016-08-13  syler       creation
 **/

#ifndef GO_TO_LOCAL_SERVER_H_
#define GO_TO_LOCAL_SERVER_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <mavpro/FlyToLocalAction.h>


namespace fly_to_local {

  typedef actionlib::SimpleActionServer<mavpro::FlyToLocalAction> FlyToLocalActionServer;

  class FlyToLocalServer {
    public:

    /**
     * @brief  Constructor 
     * @param tf A reference to a TransformListener
     */
     FlyToLocalServer(tf::TransformListener& tf);

    /**
     * @brief  Destructor 
     */
     virtual ~FlyToLocalServer();

    private:

      std::string _ns;

      ros::NodeHandle* _nh;
      ros::NodeHandle* _private_nh;

      tf::TransformListener& _tf;

      ros::Publisher _current_goal_pub, _setpoint_pub, _action_goal_pub;
      ros::Subscriber _position_sub;

      FlyToLocalActionServer* _as;

      mavpro::FlyToLocalResult _result;

      double _controller_frequency;
      double _xy_tolerance, _z_tolerance;

      geometry_msgs::PoseStamped _current_position;
      geometry_msgs::PoseStamped _goal_position;

      void resetSetpointToCurrentPosition();

      /**
      * @brief  Reset the state of the action and reset setpoint to current position
      */
      void resetState();

      bool goalToFCUFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      void executeCb(const mavpro::FlyToLocalGoalConstPtr& fly_to_local_goal);

      void poseCb(const geometry_msgs::PoseStampedConstPtr& local_pose);

      double distanceXY(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      double distanceZ(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      bool isGoalReached(const geometry_msgs::PoseStamped& goal);

      /**
      * @brief  Performs a control cycle
      * @param goal A reference to the goal to pursue
      * @return True if processing of the goal is done, false otherwise
      */
      bool executeCycle(geometry_msgs::PoseStamped& goal);

      void preemptCb();


  };
};

#endif