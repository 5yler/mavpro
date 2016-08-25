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
 * fly_to_local_server.cpp
 * Action server for UAV FlyToLocal action
 * 
 * @author	Syler Wagner	<syler@mit.edu>
 * @date	2016-08-13	syler 	creation
 **/

#include <fly_to_local_server.h>
#include <cassert>	//$ for assertions

namespace fly_to_local {

	FlyToLocalServer::FlyToLocalServer(tf::TransformListener& tf) :
	_tf(tf),
	_as(NULL)
	{

		ROS_WARN("Starting fly_to_local_server with mavros namespace '%s'", _ns.c_str());

		_private_nh = new ros::NodeHandle("~");
		_private_nh->param<std::string>("mavros_ns", _ns, "mavros");
		_nh = new ros::NodeHandle(_ns);

		_as = new FlyToLocalActionServer(ros::NodeHandle(), "fly_to_local", boost::bind(&FlyToLocalServer::executeCb, this, _1), false);

		_result.success = false; //$ initialize the result success to false (set to true once goal is reached)

		//# for comanding the UAV
		_setpoint_pub = _nh->advertise<geometry_msgs::PoseStamped>("setpoint_position/local", 1);

		//# for keeping track of current pose
		_position_sub = _nh->subscribe<geometry_msgs::PoseStamped>("local_position/pose", 1, &FlyToLocalServer::poseCb, this);

		_current_goal_pub = _private_nh->advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

		ros::NodeHandle action_nh("fly_to_local_server");
		_action_goal_pub = action_nh.advertise<mavpro::FlyToLocalActionGoal>("goal", 1);

		//$ get some parameters
		_private_nh->param("controller_frequency", _controller_frequency, 0.5);
		_private_nh->param("xy_tolerance", _xy_tolerance, 2.0);
		_private_nh->param("z_tolerance", _z_tolerance, 2.0);

		ROS_WARN("Done initializing fly_to_local_server. Ready to handle FlyToLocalServer.action");

		//$ start the action server
		_as->start();
	}

	FlyToLocalServer::~FlyToLocalServer()
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


	void FlyToLocalServer::resetSetpointToCurrentPosition()
	{
		_setpoint_pub.publish(_current_position);
	}


	void FlyToLocalServer::resetState()
	{
		resetSetpointToCurrentPosition();
		ROS_WARN("Reset setpoint to current location.");
	}

	geometry_msgs::PoseStamped FlyToLocalServer::goalToFCUFrame(const geometry_msgs::PoseStamped& goal_pose_msg) 
	{
		std::string fcu_frame = "fcu";

		geometry_msgs::PoseStamped fcu_pose_msg;
		fcu_pose_msg.header.frame_id = "fcu";

		tf::StampedTransform transform;

		try{
			_tf.lookupTransform(goal_pose_msg.header.frame_id, fcu_frame, ros::Time(0), transform);
			
			fcu_pose_msg.pose.position.x = goal_pose_msg.pose.position.x - transform.getOrigin().x();
			fcu_pose_msg.pose.position.y = goal_pose_msg.pose.position.y - transform.getOrigin().y();
			fcu_pose_msg.pose.position.z = goal_pose_msg.pose.position.z - transform.getOrigin().z();
			fcu_pose_msg.header.stamp = ros::Time::now(); 
		}
		catch(tf::TransformException& ex) 
		{
			ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
				goal_pose_msg.header.frame_id.c_str(), fcu_frame.c_str(), ex.what());
			
			ROS_ERROR("THIS IS REALLY BAD");
			return goal_pose_msg;
		}

		return fcu_pose_msg;
	}

	void FlyToLocalServer::poseCb(const geometry_msgs::PoseStampedConstPtr& local_pose)
	{
		_current_position.header = local_pose->header;
		_current_position.pose = local_pose->pose;
	}


	void FlyToLocalServer::executeCb(const mavpro::FlyToLocalGoalConstPtr& fly_to_local_goal) 
	{
		geometry_msgs::PoseStamped goal = goalToFCUFrame(fly_to_local_goal->target);

		_current_goal_pub.publish(goal);
		ROS_INFO("fly_to_local has received a goal of x: %.2f, y: %.2f, z: %.2f (frame id: %s)", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal.header.frame_id.c_str());

		ros::Rate r(_controller_frequency);

		ros::NodeHandle n;

		while(n.ok())
		{
			if (_as->isPreemptRequested())
			{
				if (_as->isNewGoalAvailable())
				{
					// if we're active and a new goal is available, we'll accept it
					mavpro::FlyToLocalGoal new_goal = *_as->acceptNewGoal();

					goal = goalToFCUFrame(new_goal.target);

					// publish current goal
					ROS_INFO("fly_to_local has received a goal of x: %.2f, y: %.2f, z: %.2f (frame id: %s)", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal.header.frame_id.c_str());
					_current_goal_pub.publish(goal);
				}
				else 
				{ // if we've been preempted explicitly, shut things down
					resetState();

					// notify the ActionServer that we've successfully preempted
					ROS_INFO("fly_to_local preempting the current goal");
					_as->setPreempted();

					// return from execute after preempting
					return;
				}
			}

			// we also want to check if we've changed global frames because we need to transform our goal pose
			if (goal.header.frame_id != "fcu") 
			{
				goal = goalToFCUFrame(goal);

				//publish the goal point to the visualizer
				ROS_DEBUG_NAMED("fly_to_local","The global frame for fly_to_local has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
				_current_goal_pub.publish(goal);
			}

			//for timing that gives real time even in simulation
			ros::WallTime start = ros::WallTime::now();

			//the real work on pursuing a goal is done here
			bool done = executeCycle(goal);

			if (done)
			{
				return;
			}

			ros::WallDuration t_diff = ros::WallTime::now() - start;
			ROS_DEBUG_NAMED("fly_to_local","Full control cycle time: %.9f\n", t_diff.toSec());

			r.sleep(); 
		}


		//$ if the node is killed then we'll abort and return
		_as->setAborted(_result, "Aborting on the goal because the node has been killed");
		return;
	}


	bool FlyToLocalServer::isGoalReached(const geometry_msgs::PoseStamped& goal) 
	{
		if ((distanceXY(_current_position, goal) <= _xy_tolerance) &&
			(distanceZ(_current_position, goal) <= _z_tolerance))
		{
			return true;
		} 
		else 
		{
			return false;
		}
	}

	double FlyToLocalServer::distanceXY(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) 
	{
		return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
	}

	double FlyToLocalServer::distanceZ(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) 
	{
		return abs(p1.pose.position.z - p2.pose.position.z);
	}


	bool FlyToLocalServer::executeCycle(geometry_msgs::PoseStamped& goal) 
	{

		//$ the goal should always be in fcu frame, double check
		assert(goal.header.frame_id == "fcu");

		//$ publish setpoint to mavros setpoint_position which will handle the controls
		_setpoint_pub.publish(goal);

		//$ TODO: use feedback in same frame as goal?

		//$ publish feedback
		mavpro::FlyToLocalFeedback feedback;
		feedback.position = _current_position;
		_as->publishFeedback(feedback);


		//$ check if goal has been reached
		if (isGoalReached(goal)) 
		{
			ROS_ERROR("SUCCESS");
			ROS_DEBUG_NAMED("fly_to_local_server", "Goal reached!");
			resetState();

			_result.success = true;
			_as->setSucceeded(_result, "Goal reached.");
			return true;
		}

		// we aren't done yet
		return false;
	}

	void FlyToLocalServer::preemptCb()     
	{
		ROS_DEBUG_NAMED("fly_to_local_server", "Preempted!");
		// set the action state to preempted
		_as->setPreempted();
	}

}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "fly_to_local_server");
	tf::TransformListener tf(ros::Duration(10));

	fly_to_local::FlyToLocalServer fly_to_local(tf);

	ros::spin();
	return(0);
}