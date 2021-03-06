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
#include <actionlib/server/simple_action_server.h>
#include <mavpro/TakeoffAction.h>

//$ mavros services
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/CommandBool.h> 	//$ arming command

//$ message types
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

 enum OperationalMode 
 {
	RC_TRANSMITTER,
	NO_RC_TRANSMITTER
  };



 typedef actionlib::SimpleActionServer<mavpro::TakeoffAction> TakeoffActionServer;

 class TakeoffServer
 {
 public:
 	TakeoffServer() :
 	_nh(NULL), _private_nh(NULL), _as(NULL), _armed(false), _current_alt(0.0)
 	{
		_private_nh = new ros::NodeHandle("~");
		_private_nh->param<std::string>("mavros_ns", _ns, "mavros");
 		ROS_WARN("Starting takeoff_server with mavros namespace '%s'", _ns.c_str());

		_nh = new ros::NodeHandle(_ns);

		// //# for comanding the UAV
		// _vel_pub = nh.advertise<geometry_msgs::PoseStamped>("setpoint_position/local", 1);

 		_as = new TakeoffActionServer(ros::NodeHandle(), "takeoff", boost::bind(&TakeoffServer::executeCb, this, _1), false);

		//# for keeping track of current altitude
 		_alt_sub = _nh->subscribe<std_msgs::Float64>("global_position/rel_alt", 1, &TakeoffServer::altCallback, this);
 		_state_sub = _nh->subscribe<mavros_msgs::State>("state", 1, &TakeoffServer::stateCallback, this);

 		ROS_INFO("Subscribing to mavros_msgs/RCIn on topic: %s/rc/in", _ns.c_str());

 		_rc_sub = _nh->subscribe<mavros_msgs::RCIn>("rc/in", 1, &TakeoffServer::radioCallback, this);

 		_rc_pub = _nh->advertise<mavros_msgs::OverrideRCIn>("rc/override", 1);

 		//# for comanding the UAV
		_setpoint_pub = _nh->advertise<geometry_msgs::PoseStamped>("setpoint_position/local", 1);

		//# for keeping track of current pose
		_position_sub = _nh->subscribe<geometry_msgs::PoseStamped>("local_position/pose", 1, &TakeoffServer::poseCallback, this);


		//$ get some parameters
 		_private_nh->param("feedback_frequency", _feedback_frequency, 1.0);
 		_private_nh->param("controller_frequency", _controller_frequency, 10.0);
 		_private_nh->param("z_tolerance", _z_tolerance, 1.0);
 		_private_nh->param("no_transmitter", _no_transmitter, false);
 		_private_nh->param("timeout", _takeoff_timeout, 180);	//$ timeout in seconds after which goal is aborted
 		_private_nh->param("disable_param_checks",_disable_param_checks, false);	//$ set to true of you want to test without mavros in the loop

 		ROS_WARN("Done initializing takeoff_server.");
 		_as->start();
 	}

 	~TakeoffServer()
 	{
 		if (!_no_transmitter) 
		{
			clearRCOverride(); //$ if transmitter is used, clear RC overrides
		}

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

 	ros::NodeHandle* _nh;
 	ros::NodeHandle* _private_nh;

 	ros::Subscriber _alt_sub;
 	ros::Subscriber _state_sub;
 	ros::Subscriber _rc_sub;
 	ros::Subscriber _position_sub;

 	ros::Publisher _rc_pub;
    ros::Publisher _setpoint_pub;

 	TakeoffActionServer* _as;

 	mavpro::TakeoffResult _result;

 	double _feedback_frequency;
 	double _z_tolerance;
	bool _no_transmitter;	
	int _takeoff_timeout;	
	bool _disable_param_checks;	

	int _throttle_failsafe_cutoff;
 	double _controller_frequency;

 	double _current_alt;
 	std::string _mode;
	bool _armed;	//$ current arming status
	int _throttle_pwm;
	int _ch6_pwm;		//$ current channel 6 PWM value
	int _ch6_pwm_init;	//$ channel 6 PWM value when goal accepted
    geometry_msgs::PoseStamped _current_position;

	bool checkGCSID() 
	{
		if (_disable_param_checks) return true;

		int mavros_sys_id, uav_gcs_id;

		_nh->getParam("system_id", mavros_sys_id);

		ros::ServiceClient param_client = _nh->serviceClient<mavros_msgs::ParamGet>("param/get");

		mavros_msgs::ParamGet srv;
		srv.request.param_id = "SYSID_MYGCS";

		if (param_client.call(srv))
		{
			uav_gcs_id = srv.response.value.integer;
		

			if (mavros_sys_id == uav_gcs_id)
			{
				ROS_WARN("mavros system_id: %d", mavros_sys_id);
				ROS_WARN("fcu SYSID_MYGCS: %d", uav_gcs_id);
				return true;
			}
			else
			{
				ROS_ERROR("Parameter mismatch!");
				ROS_ERROR("mavros system_id: %d", mavros_sys_id);
				ROS_ERROR("fcu SYSID_MYGCS: %d", uav_gcs_id);
				return false;
			}
		}
		else
		{
			ROS_ERROR("Could not get SYSID_MYGCS parameter from FCU");
			return false;
		}
	}

	/**
	 * Get value of FS_THR_VALUE parameter on flight controller, below which the throttle failsafe is activated
	 */
	bool getThrottleFailsafeCutoff()
	{
		if (_disable_param_checks) return true;

		ros::ServiceClient param_client = _nh->serviceClient<mavros_msgs::ParamGet>("param/get");

		mavros_msgs::ParamGet srv;
		srv.request.param_id = "FS_THR_VALUE";

		if (param_client.call(srv))
		{
			_throttle_failsafe_cutoff = srv.response.value.integer;
			return true;
		}
		else 
		{
			return false;
		}
	}

	bool setMode(const std::string mode)
	{
		ros::ServiceClient mode_client = _nh->serviceClient<mavros_msgs::SetMode>("set_mode");
		mavros_msgs::SetMode srv;

		srv.request.base_mode = 0;
		srv.request.custom_mode = mode;

		bool mode_changed = false;

		ros::Rate r(_controller_frequency);
		int num_calls = 0;

		while ((!mode_changed) && _nh->ok())
		{
			num_calls++;

			ROS_INFO("Attempting to switch to %s mode [%d/20]", mode.c_str(), num_calls);

			if (mode_client.call(srv))
			{
				if (_mode == mode)
				{
					mode_changed = true;
					ROS_WARN("Changed to %s mode", mode.c_str());
				}
			}

			r.sleep();

			if (num_calls > 20)
			{
				ROS_ERROR("Tried to change mode %d times, aborting.", num_calls);
				return false;
			}

		}

		return mode_changed;
	}

	void executeCb(const mavpro::TakeoffGoalConstPtr& goal)
	{

		ROS_INFO("takeoff_server has received takeoff goal altitude %4.2f", goal->altitude);

		ros::Rate r(_controller_frequency);
		ros::NodeHandle n;

		//for timing that gives real time even in simulation
		ros::WallTime start = ros::WallTime::now();

		//$ check if GCS IDs match, if not, abort the goal
		if (!checkGCSID())
		{
			ROS_ERROR("Cannot perform RC override with ID mismatch, aborting goal");
			_as->setAborted(_result, "I can't do this!");
			return;
		}

		//$ if can't get throttle failsafe cutoff parameter, abort the goal
		if (!getThrottleFailsafeCutoff())
		{
			ROS_ERROR("Could not get FS_THR_VALUE parameter from flight controller! Aborting goal.");
			_as->setAborted(_result, "I can't do this!");
			return;
		}

		//$ disarm UAV first
		executeArm(false);

		//$ keep track of channel 6 PWM value, so we can abort if it changes during takeoff attempt
		_ch6_pwm_init = _ch6_pwm;

		while (n.ok())
		{
			ros::WallDuration t_diff = ros::WallTime::now() - start;

			if (t_diff.toSec() > _takeoff_timeout)
			{
				ROS_WARN("It has been %4.1f seconds since the takeoff goal was accepted...", t_diff.toSec());
				ROS_ERROR("THIS IS TAKING TOO LONG, SO I AM JUST GOING TO LAND INSTEAD.");
				setMode("LAND");
				clearRCOverride();
				_as->setAborted(_result, "Aborting on the goal because an attempt at manual control was initiated");
				//$ return from execute if manual control was initiated
				return;
			}


			if (_as->isPreemptRequested())
			{
				//$ TODO?
				ROS_INFO("takeoff_server preempting the current goal");
				_as->setPreempted();

				// return from execute after preempting
				return;
			}

			if (_ch6_pwm != _ch6_pwm_init)
			{
				ROS_ERROR("Detected manual activity on channel 6 switch, aborting takeoff!");
				setMode("LAND");
				clearRCOverride();
				_as->setAborted(_result, "Aborting on the goal because an attempt at manual control was initiated");
				//$ return from execute if manual control was initiated
				return;
			}

			bool done = false;

			//$ actually try to take off
			if (executeTakeoff(goal->altitude))
			{
				//$ publish feedback
				mavpro::TakeoffFeedback feedback;
				feedback.altitude = _current_alt;
				_as->publishFeedback(feedback);
			}

			//$ check if goal has been reached
			done = isGoalReached(goal->altitude);

			if (done)
			{
				ros::WallDuration t_diff = ros::WallTime::now() - start;
				ROS_ERROR("SUCCESS!");
				ROS_INFO_NAMED("takeoff_server","Takeoff time: %.9f\n", t_diff.toSec());
				ROS_DEBUG_NAMED("takeoff_server", "Goal reached!");
				clearRCOverride();
				//$ done!
				_result.success = true;
				_as->setSucceeded(_result);
				return;
			}

			r.sleep();
		}

		//$ if the node is killed then we'll abort and return
		_result.success = false;
		_as->setAborted(_result, "Aborting on the goal because the node has been killed");
		return;
	}

	geometry_msgs::PoseStamped generateTakeoffTargetPose(const double goal)
	{
		geometry_msgs::PoseStamped target;
		target.header = _current_position.header;
		target.pose.position.x = _current_position.pose.position.x;
		target.pose.position.y = _current_position.pose.position.y;
		target.pose.position.z = _current_position.pose.position.z + goal;
		return target;
	}

	bool executeTakeoff(const double goal)
	{
		ros::Rate r(_controller_frequency);

		if (!_armed) 
		{
			setMode("STABILIZE");
			overrideThrottle(_throttle_failsafe_cutoff + 10);
			r.sleep();
			executeArm(true);
			overrideThrottle(1400);
			setMode("GUIDED");
			
			geometry_msgs::PoseStamped target = generateTakeoffTargetPose(goal);
			_setpoint_pub.publish(target);
		}
		return true;
	}

	bool executeArm(bool arming_value) 
	{
		//$ TODO

		ros::ServiceClient arming_client = _nh->serviceClient<mavros_msgs::CommandBool>("cmd/arming");
		mavros_msgs::CommandBool srv;

		srv.request.value = arming_value;

		bool arm_status_changed = false;

		int num_calls = 0;
		while ((!arm_status_changed) && _nh->ok())
		{
			num_calls++;
			if (arming_client.call(srv))
			{
				if (arming_value == true) ROS_WARN("Armed!");
				else ROS_WARN("Disarmed!");
				arm_status_changed = true;
			}
			else
			{
				ROS_ERROR("Failed to arm [%d/20]", num_calls);
			}

			if (num_calls > 20)
			{
				ROS_ERROR("Tried to arm %d times, aborting.", num_calls);
				return false;
			}
		}

		return arm_status_changed;
	}



	void overrideThrottle(int pwm) {

		ros::Rate r(_controller_frequency);

		ROS_WARN("Overriding throttle to PWM %d", pwm);

		mavros_msgs::OverrideRCIn rc_override;
		int NO_CHANGE = 65535; 	//$ leaves RC channel value unchanged 

		ros::WallTime start = ros::WallTime::now();

		int num_calls = 0;

		while ((_throttle_pwm != pwm) && _nh->ok())
		{
			num_calls++;
			ROS_INFO("Trying to override throttle PWM from %d to %d [%d]", _throttle_pwm, pwm, num_calls);

			//$ throttle is channel 2 (C++ is zero indexed)		
			for (int i = 0; i < 8; i++)
			{
				rc_override.channels[i] = NO_CHANGE;
			}
			rc_override.channels[2] = pwm; 		//$ set override for throttle channel only

			_rc_pub.publish(rc_override);
			ros::spinOnce();

			r.sleep();

			if (num_calls > 20) break;
		}
		ros::WallDuration t_diff = ros::WallTime::now() - start;
		ROS_WARN("Throttle override time: %.9f\n", t_diff.toSec());
	}

	/**
	 * @brief	clear RC override by releasing all channels
	 */
	void clearRCOverride() 
	{
		ROS_WARN("Clearing all RC overrides");

		mavros_msgs::OverrideRCIn rc_override;
		int RELEASE = 0;			//$ releases RC channel

		for (int i = 0; i < 8; i++)
		{
			rc_override.channels[i] = RELEASE;
		}

		_rc_pub.publish(rc_override);
	}


	bool isGoalReached(const double goal) 
	{
		ROS_INFO("goal: %4.1f, current alt: %4.1f, error: %4.1f", goal, _current_alt, fabs(goal - _current_alt));

		if ((goal - _current_alt) >= _z_tolerance)
		{
			return false;
		} 
		else 
		{
			return true;
		}
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

	void poseCallback(const geometry_msgs::PoseStampedConstPtr& local_pose)
	{
		_current_position.header = local_pose->header;
		_current_position.pose = local_pose->pose;
	}

	void radioCallback(const mavros_msgs::RCIn::ConstPtr& radio_msg)
	{
		_throttle_pwm = radio_msg->channels[2]; 
		_ch6_pwm = radio_msg->channels[4];
	}

	void preemptCb()
	{
		ROS_ERROR_NAMED("takeoff_server", "Preempted!");
		 // set the action state to preempted
		_as->setPreempted();
	}

	/**
	 * Reset UAV state by clearing RC override and setting mode to STABILIZE.
	 */
	void resetState()
	{
		if (!_no_transmitter) 
		{
			clearRCOverride(); //$ if transmitter is used, clear RC overrides
			setMode("STABILIZE");	//$ switch to STABILIZE mode
		}
	}


 }; //$ end of class TakeoffServer

 int main(int argc, char **argv) {

 	ros::init(argc, argv, "takeoff_server");

 	TakeoffServer takeoff;

 	ros::spin();
 	return 0;
 }

