/**
* fly_to_local_server.h
* Action server for FlyToLocal action
* 
* @author  Syler Wagner      <syler@mit.edu>

* @date    2016-08-13  syler   creation
*
**/

#include <fly_to_local.h>
#include <cassert>	//$ for assertions

namespace fly_to_local {

	FlyToLocal::FlyToLocal(tf::TransformListener& tf) :
	tf_(tf),
	as_(NULL) 
	{
		as_ = new FlyToLocalActionServer(ros::NodeHandle(), "fly_to_local", boost::bind(&FlyToLocal::executeCb, this, _1), false);

		ros::NodeHandle private_nh("~");
		ros::NodeHandle nh;

		//# for comanding the UAV
		setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

		//# for keeping track of current pose
		position_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &FlyToLocal::poseCb, this);

		current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

		ros::NodeHandle action_nh("fly_to_local_server");
		action_goal_pub_ = action_nh.advertise<mavpro::FlyToLocalActionGoal>("goal", 1);

		//$ set some parameters
		private_nh.param("controller_frequency", controller_frequency_, 0.5);
		private_nh.param("xy_tolerance", xy_tolerance_, 2.0);
		private_nh.param("z_tolerance", z_tolerance_, 2.0);

		ROS_WARN("Done initializing fly_to_local_server. Ready to handle FlyToLocal.action");

		//$ start the action server
		as_->start();
	}

	FlyToLocal::~FlyToLocal()
	{
		if (as_ != NULL)
			delete as_;
	}


	void FlyToLocal::resetSetpointToCurrentPosition()
	{
		setpoint_pub_.publish(current_position_);
	}


	void FlyToLocal::resetState()
	{
		resetSetpointToCurrentPosition();
		ROS_WARN("Reset setpoint to current location.");
	}

	geometry_msgs::PoseStamped FlyToLocal::goalToFCUFrame(const geometry_msgs::PoseStamped& goal_pose_msg) 
	{
		std::string fcu_frame = "fcu";

		geometry_msgs::PoseStamped fcu_pose_msg;
		fcu_pose_msg.header.frame_id = "fcu";

		tf::StampedTransform transform;

		try{
			tf_.lookupTransform(goal_pose_msg.header.frame_id, fcu_frame, ros::Time(0), transform);
			
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

	void FlyToLocal::poseCb(const geometry_msgs::PoseStampedConstPtr& local_pose)
	{
		current_position_.header = local_pose->header;
		current_position_.pose = local_pose->pose;   
	}


	void FlyToLocal::executeCb(const mavpro::FlyToLocalGoalConstPtr& fly_to_local_goal) 
	{
		geometry_msgs::PoseStamped goal = goalToFCUFrame(fly_to_local_goal->target);

		current_goal_pub_.publish(goal);
		ROS_INFO("fly_to_local has received a goal of x: %.2f, y: %.2f, z: %.2f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

		ros::Rate r(controller_frequency_);

		ros::NodeHandle n;

		while(n.ok())
		{
			if (as_->isPreemptRequested()) 
			{
				if (as_->isNewGoalAvailable()) 
				{
					// if we're active and a new goal is available, we'll accept it
					mavpro::FlyToLocalGoal new_goal = *as_->acceptNewGoal();

					goal = goalToFCUFrame(new_goal.target);

					// publish current goal
					ROS_INFO("fly_to_local has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
					current_goal_pub_.publish(goal);
				}
				else 
				{ // if we've been preempted explicitly, shut things down
					resetState();

					// notify the ActionServer that we've successfully preempted
					ROS_INFO("fly_to_local preempting the current goal");
					as_->setPreempted();

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
				current_goal_pub_.publish(goal);
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
		as_->setAborted(mavpro::FlyToLocalResult(), "Aborting on the goal because the node has been killed");
		return;
	}


	bool FlyToLocal::isGoalReached(const geometry_msgs::PoseStamped& goal) 
	{
		if ((distanceXY(current_position_, goal) <= xy_tolerance_) &&
			(distanceZ(current_position_, goal) <= z_tolerance_)) 
		{
			return true;
		} 
		else 
		{
			return false;
		}
	}

	double FlyToLocal::distanceXY(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) 
	{
		return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
	}

	double FlyToLocal::distanceZ(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) 
	{
		return abs(p1.pose.position.z - p2.pose.position.z);
	}


	bool FlyToLocal::executeCycle(geometry_msgs::PoseStamped& goal) 
	{

		//$ the goal should always be in fcu frame, double check
		assert(goal.header.frame_id == "fcu");

		//$ publish setpoint to mavros setpoint_position which will handle the controls
		setpoint_pub_.publish(goal);

		//$ TODO: use feedback in same frame as goal?

		//$ publish feedback
		mavpro::FlyToLocalFeedback feedback;
		feedback.position = current_position_;
		as_->publishFeedback(feedback);


		//$ check if goal has been reached
		if (isGoalReached(goal)) 
		{
			ROS_ERROR("SUCCESS");
			ROS_DEBUG_NAMED("fly_to_local_server", "Goal reached!");
			resetState();

			as_->setSucceeded(mavpro::FlyToLocalResult(), "Goal reached.");
			return true;
		}

		// we aren't done yet
		return false;
	}

	void FlyToLocal::preemptCb()     
	{
		ROS_DEBUG_NAMED("fly_to_local_server", "Preempted!");
		// set the action state to preempted
		as_->setPreempted();
	}

}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "fly_to_local_server");
	tf::TransformListener tf(ros::Duration(10));

	fly_to_local::FlyToLocal fly_to_local(tf);

	ros::spin();
	return(0);
}