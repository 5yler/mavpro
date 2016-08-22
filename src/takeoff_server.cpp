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
 * takeoff_server.cpp
 * Actionlib implementation of UAV takeoff
 * 
 * @author  Syler Wagner  <syler@mit.edu>
 * @date    2016-08-21    creation
 **/

#include <ros/ros.h>
#include <pid_controller.h>
#include <actionlib/server/simple_action_server.h>
#include <mavpro/TakeoffAction.h>

//$ mavros services
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/CommandTOL.h>		//$ takeoff/landing command
#include <mavros_msgs/CommandBool.h> 	//$ arming command

//$ message types
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>

 typedef actionlib::SimpleActionServer<mavpro::TakeoffAction> TakeoffActionServer;

 class TakeoffServer
 {
 public:
 	TakeoffServer() :
 	_n(NULL),
 	_as(NULL)
 	{
 		_n = new ros::NodeHandle("~");

		// //# for comanding the UAV
		// _vel_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

 		_as = new TakeoffActionServer(ros::NodeHandle(), "takeoff", boost::bind(&TakeoffServer::executeCb, this, _1), false);

		//# for keeping track of current altitude
 		_alt_sub = _n->subscribe<std_msgs::Float64>("/mavros/global_position/rel_alt", 1, &TakeoffServer::altCallback, this);
 		_state_sub = _n->subscribe<mavros_msgs::State>("/mavros/state", 1, &TakeoffServer::stateCallback, this);
 		_rc_sub = _n->subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, &TakeoffServer::radioCallback, this);


 		_rc_pub = _n->advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);



		//$ set parameter
 		_n->param("controller_frequency", _controller_frequency, 10.0);
 		_n->param("z_tolerance", _z_tolerance, 1);

 		ROS_WARN("Done initializing takeoff_server.");
 		_as->start();
 	}

 	~TakeoffServer()
 	{
 		if (_as != NULL)
 		{
 			delete _as;
 		}
 		if (_n != NULL)
 		{
 			delete _n;
 		}
 	}

 private:


 	TakeoffActionServer* _as;
 	mavpro::TakeoffResult _result;

 	ros::NodeHandle* _n;
 	ros::Subscriber _alt_sub;
 	ros::Subscriber _state_sub;
 	ros::Subscriber _rc_sub;

 	ros::Publisher _rc_pub;

 	double _controller_frequency;
 	double _z_tolerance;

 	double _current_alt;
 	std::string _mode;
	bool _armed;	//$ current arming status
	int _throttle_pwm;

	bool checkGCSID() 
	{
		int mavros_sys_id, uav_gcs_id;

		_n->getParam("mavros/system_id", mavros_sys_id);

		ros::ServiceClient param_client = _n->serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");

		mavros_msgs::ParamGet srv;
		srv.request.param_id = "SYSID_MYGCS";

		if (param_client.call(srv))
		{
			uav_gcs_id = srv.response.value.integer;
		}

		if (mavros_sys_id == uav_gcs_id)
		{
			return true;
		}
		else
		{
			ROS_ERROR("mavros system_id: %d", mavros_sys_id);
			ROS_ERROR("fcu SYSID_MYGCS: %d", uav_gcs_id);
			return false;
		}
	}

	bool setMode(const std::string mode)
	{
		ros::ServiceClient mode_client = _n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
		mavros_msgs::SetMode srv;

		srv.request.base_mode = 0;
		srv.request.custom_mode = mode;

		bool mode_changed = false;

		ros::Rate r(_controller_frequency);

		while ((!mode_changed) && _n->ok())
		{

			int num_calls = 0;

			if (mode_client.call(srv))
			{
				if (_mode == mode)
				{
					mode_changed = true;
					ROS_WARN("Changed to %s mode", mode.c_str());

				}
			}
			else
			{
				ROS_ERROR("Failed to switch to %s mode", mode.c_str());
				num_calls += 1;
			}

			r.sleep();

			if (num_calls > 5)
			{
				ROS_ERROR("Tried to change mode 5 times, aborting.");
				break;
			}
		}

		return mode_changed;
	}

	void executeCb(const mavpro::TakeoffGoalConstPtr& goal)
	{

		ROS_INFO("takeoff_server has received takeoff goal");

		ros::Rate r(_controller_frequency);
		ros::NodeHandle n;

		while (n.ok())
		{

			if (_as->isPreemptRequested())
			{
				//$ TODO?
				ROS_INFO("takeoff_server preempting the current goal");
				_as->setPreempted();

				// return from execute after preempting
				return;
			}

			//for timing that gives real time even in simulation
			ros::WallTime start = ros::WallTime::now();


			//$ actually try to land
			bool done = executeTakeoff(goal->altitude);

			if (done)
			{
				return;
			}
			ros::WallDuration t_diff = ros::WallTime::now() - start;
			ROS_DEBUG_NAMED("takeoff_server","Takeoff time: %.9f\n", t_diff.toSec());

			r.sleep();
		}

		//$ if the node is killed then we'll abort and return
		_result.success = false;
		_as->setAborted(_result, "Aborting on the goal because the node has been killed");
		return;
	}

	bool executeArm(bool arming_value) 
	{
		//$ TODO

		ros::ServiceClient arming_client = _n->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
		mavros_msgs::CommandBool srv;

		srv.request.value = arming_value;

		bool arm_status_changed = false;

		while ((!arm_status_changed) && _n->ok())
		{

			int num_calls = 0;

			if (arming_client.call(srv))
			{
				ROS_WARN("Arming status changed");
				arm_status_changed = true;
			}
			else
			{
				ROS_ERROR("Failed to arm");
				num_calls += 1;
			}

			if (num_calls > 5)
			{
				ROS_ERROR("Tried to arm 5 times, aborting.");
				break;
			}
		}

		return arm_status_changed;
	}



	void overrideThrottle(int pwm) {

		ros::Rate r(_controller_frequency);

		ROS_WARN("Overriding throttle to PWM %d", pwm);

		mavros_msgs::OverrideRCIn rc_override;
		int NO_CHANGE = 65535; 	//$ leaves RC channel value unchanged 
		int RELEASE = 0;			//$ releases RC channel

		ros::WallTime start = ros::WallTime::now();

		while ((_throttle_pwm != pwm) && _n->ok())
		{
			//$ throttle is channel 2 (C++ is zero indexed)
			if (pwm == 0)	//$ clear previous override values
			{
				for (int i = 0; i < 8; i++)
				{
					rc_override.channels[i] = RELEASE;
				}
				rc_override.channels[2] = pwm; 
			}
			else			//$ set override for throttle channel
			{
				for (int i = 0; i < 8; i++)
				{
					rc_override.channels[i] = NO_CHANGE;
				}
				rc_override.channels[2] = pwm;
			}

			_rc_pub.publish(rc_override);
			r.sleep();
		}
		ros::WallDuration t_diff = ros::WallTime::now() - start;
		ROS_WARN("Throttle override time: %.9f\n", t_diff.toSec());
	}


	bool isGoalReached(const double goal) 
	{
		ROS_INFO("goal: %4.1f, current alt: %4.1f, error: %4.1f", goal, _current_alt, fabs(goal - _current_alt));

		if (fabs(goal - _current_alt) <= _z_tolerance)
		{
			return true;
		} 
		else 
		{
			return false;
		}
	}

	bool executeTakeoff(const double goal)
	{

		ros::Rate r(10);

		if (!_armed) 
		{
			setMode("STABILIZE");
			overrideThrottle(1000);
			r.sleep();
			executeArm(true);
			setMode("GUIDED");
		}

		 //$ publish feedback
		mavpro::TakeoffFeedback feedback;
		feedback.altitude = _current_alt;
		_as->publishFeedback(feedback);

		 //$ check if goal has been reached
		if (isGoalReached(goal)) 
		{
			ROS_ERROR("SUCCESS");
			ROS_DEBUG_NAMED("takeoff_server", "Goal reached!");
			// resetState();

			_result.success = true;
			_as->setSucceeded(_result, "Goal reached.");
			return true;
		}

		mavros_msgs::CommandTOL srv;

		ros::ServiceClient takeoff_client = _n->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

		srv.request.min_pitch = 5;
		srv.request.yaw = 0;
		srv.request.latitude = 0;
		srv.request.longitude = 0;
		srv.request.altitude = goal;

		bool takeoff_call_success = false;

		while ((!takeoff_call_success) && _n->ok())
		{

			int num_calls = 0;

			if (takeoff_client.call(srv))
			{
				ROS_WARN("Called mavros takeoff service");
				takeoff_call_success = true;
			}
			else
			{
				ROS_ERROR("Failed to call mavros takeoff service");
				num_calls += 1;
			}

			if (num_calls > 5)
			{
				ROS_ERROR("Tried to call mavros takeoff service 5 times, aborting.");
				break;
			}
		}

		 //$ not done yet
		return false;
	}

	void altCallback(const std_msgs::Float64::ConstPtr& rel_alt_msg)
	{
		_current_alt = rel_alt_msg->data;
	}

	void stateCallback(const mavros_msgs::State::ConstPtr& state_msg)
	{
		_armed = state_msg->armed;
		_mode = state_msg->mode;
	}

	void radioCallback(const mavros_msgs::RCIn::ConstPtr& radio_msg)
	{
		_throttle_pwm = radio_msg->channels[2]; 
	}

	void preemptCb()
	{
		ROS_DEBUG_NAMED("takeoff_server", "Preempted!");
		 // set the action state to preempted
		_as->setPreempted();
	}


 }; //$ end of mode_clientass TakeoffServer

 int main(int argc, char **argv) {

 	ros::init(argc, argv, "takeoff_server");

 	TakeoffServer land;

 	ros::spin();
 	return 0;
 }
