 /**
 * landing_server.cpp
 * Transform and republish PoseStamped messages to a topic with a different frame_id
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
			_n(NULL),
			_as(NULL)
	{
		_n = new ros::NodeHandle("~");
		_as = new LandingActionServer(ros::NodeHandle(), "landing", boost::bind(&LandingServer::executeCb, this, _1), false);

		//# for keeping track of current altitude
		_alt_sub = _n->subscribe<std_msgs::Float64>("/mavros/global_position/rel_alt", 1, &LandingServer::altCallback, this);
		_state_sub = _n->subscribe<mavros_msgs::State>("/mavros/state", 1, &LandingServer::stateCallback, this);

		//$ set parameter
		_n->param("controller_frequency", controller_frequency_, 0.5);

		ROS_WARN("Done initializing landing_server.");

	}

	 ~LandingServer()
	 {
		 if (_as != NULL)
		 {
			 delete _as;
		 }
	 }

 private:


	 LandingActionServer* _as;
	 mavpro::LandingResult _result;
	 
	 ros::NodeHandle* _n;
	 ros::Subscriber _alt_sub;
	 ros::Subscriber _state_sub;

	 double controller_frequency_;

	 double _current_alt;
	 std::string _mode;
	 bool _armed;	//$ current arming status



	bool setMode(const std::string mode)
	{
		ros::ServiceClient mode_client = _n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
		mavros_msgs::SetMode srv;

		srv.request.base_mode = 0;
		srv.request.custom_mode = mode;

		bool mode_changed = false;

		while (!mode_changed)
		{

			int num_calls = 0;

			if (mode_client.call(srv))
			{
				ROS_WARN("Changed to %s mode", mode.c_str());
				mode_changed = true;
			}
			else
			{
				ROS_ERROR("Failed to switch to %s mode", mode.c_str());
				num_calls += 1;
			}

			if (num_calls > 5)
			{
				ROS_ERROR("Tried to change mode 5 times, aborting.");
				break;
			}
		}

		return mode_changed;
	}

	void executeCb(const mavpro::LandingGoalConstPtr& goal)
	{

		ROS_INFO("landing_server has received landing goal");

		ros::Rate r(controller_frequency_);
		ros::NodeHandle n;

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
		 //$ publish feedback
		 mavpro::LandingFeedback feedback;
		 feedback.altitude = _current_alt;
		 _as->publishFeedback(feedback);


		 if (_mode != "LAND")
		 {
			setMode("LAND");
		 }

		 //$ check if goal has been reached
		 if ((_mode == "LAND") && !_armed)	//$ if it has disarmed
		 {
			 ROS_ERROR("SUCCESS");
			 ROS_DEBUG_NAMED("landing_server", "Goal reached!");

			 _result.success = true;
			 _as->setSucceeded(_result, "Goal reached.");
			 return true;
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

	 void preemptCb()
	 {
		 ROS_DEBUG_NAMED("landing_server", "Preempted!");
		 // set the action state to preempted
		 _as->setPreempted();
	 }


 }; //$ end of mode_clientass LandingServer

int main(int argc, char **argv) {

	ros::init(argc, argv, "landing_server");

	LandingServer land;

	ros::spin();
	return 0;
}

