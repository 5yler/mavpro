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
 * landing_server.cpp
 * Actionlib implementation of UAV landing
 * 
 * @author  Syler Wagner  <syler@mit.edu>
 * @date    2016-08-15    creation
 **/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mavpro/LandingAction.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>

 typedef actionlib::SimpleActionServer<mavpro::LandingAction> LandingActionServer;

 class LandingServer
 {
 public:
	LandingServer() :
			_nh(NULL),
			_private_nh(NULL),
			_as(NULL)
	{
		std::string ns = ros::this_node::getNamespace();

		if (ns == "/") {
			_ns = "mavros";
		}
		else
		{
			_ns = ns;
		}
		ROS_WARN("Starting landing_server with mavros namespace '%s'", _ns.c_str());


		_private_nh = new ros::NodeHandle("~");
		_nh = new ros::NodeHandle();

		_as = new LandingActionServer(ros::NodeHandle(), "landing", boost::bind(&LandingServer::executeCb, this, _1), false);

		//# for keeping track of current altitude
		_alt_sub = _nh->subscribe<std_msgs::Float64>(_ns+"/global_position/rel_alt", 1, &LandingServer::altCallback, this);
		_state_sub = _nh->subscribe<mavros_msgs::State>(_ns+"/state", 1, &LandingServer::stateCallback, this);

		//$ set parameter
		_private_nh->param("controller_frequency", controller_frequency_, 0.5);

		ROS_WARN("Done initializing landing_server.");
		_as->start();
	}

	 ~LandingServer()
	 {
		 if (_as != NULL)
		 {
			 delete _as;
		 }
		 if (_private_nh != NULL)
		 {
			 delete _private_nh;
		 }
		 if (_nh != NULL)
		 {
			 delete _nh;
		 }
	 }

 private:

	 std::string _ns;

	 LandingActionServer* _as;
	 mavpro::LandingResult _result;

	 ros::NodeHandle* _nh;
	 ros::NodeHandle* _private_nh;

	 ros::Subscriber _alt_sub;
	 ros::Subscriber _state_sub;

	 double controller_frequency_;

	 double _current_alt;
	 std::string _mode;
	 bool _armed;	//$ current arming status



	bool setMode(const std::string mode)
	{
		ros::ServiceClient mode_client = _nh->serviceClient<mavros_msgs::SetMode>(_ns+"/set_mode");
		mavros_msgs::SetMode srv;

		srv.request.base_mode = 0;
		srv.request.custom_mode = mode;



		bool mode_changed = false;

		if (mode_client.call(srv))
		{
			ROS_WARN("Changed to %s mode", mode.c_str());
			mode_changed = true;
		}
		else
		{
			ROS_ERROR("Failed to switch to %s mode", mode.c_str());
			mode_changed = false;
		}

		return mode_changed;
	}

	void executeCb(const mavpro::LandingGoalConstPtr& goal)
	{

		ROS_INFO("landing_server has received landing goal");

		ros::Rate r(controller_frequency_);
		ros::NodeHandle n;

		int num_cycles = 0;

		while (n.ok())
		{

			if (_as->isPreemptRequested())
			{
				setMode("GUIDED");
				ROS_INFO("landing_server preempting the current goal");
				_as->setPreempted();

				// return from execute after preempting
				return;
			}

			//for timing that gives real time even in simulation
			ros::WallTime start = ros::WallTime::now();


			//$ actually try to land
			bool done = executeLanding();

			if (done)
			{
				_result.success = true;
			 	_as->setSucceeded(_result, "Goal reached.");
				return;
			}
			else //$ landing failed
			{
				_as->setAborted(_result);
				return;
			}

			ros::WallDuration t_diff = ros::WallTime::now() - start;
			ROS_DEBUG_NAMED("landing_server","Landing time: %.9f\n", t_diff.toSec());

			r.sleep();
		}

		//$ if the node is killed then we'll abort and return
		_result.success = false;
		_as->setAborted(_result, "Aborting on the goal because the node has been killed");
		return;
	}

	bool executeLanding()
	{
		mavpro::LandingFeedback feedback;
		
		bool landing_call_success = false;

		int num_calls = 0;

		ros::Rate r(1);

		while (_nh->ok())
		{
		 	//$ publish feedback
			feedback.altitude = _current_alt;
			_as->publishFeedback(feedback);


			if (_mode != "LAND")
			{
				num_calls++;
				setMode("LAND");

				if (num_calls > 20)
				{
					ROS_ERROR("Failed to switch to LAND mode, aborting");
					return false;
				}
			}

			//$ check if goal has been reached
			if ((_mode == "LAND") && !_armed)	//$ if it has disarmed
			{
				ROS_ERROR("SUCCESS");
				ROS_DEBUG_NAMED("landing_server", "Goal reached!");
				return true;
			}

			r.sleep();

		}
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

	 void preemptCb()
	 {
		 ROS_DEBUG_NAMED("landing_server", "Preempted!");
		 // set the action state to preempted
		 _as->setPreempted();
	 }


 }; //$ end of class LandingServer

int main(int argc, char **argv) {

	ros::init(argc, argv, "landing_server");

	LandingServer land;

	ros::spin();
	return 0;
}

