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
 * translate_pose.cpp
 * Transform and republish PoseStamped messages to a topic 
 * with a different frame_id
 * 
 * @author	Syler Wagner	<syler@mit.edu>
 * @date	2016-08-15	syler 	creation
 **/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <string.h>

 class TranslatedPoseRepublisher
 {
 public:
 	TranslatedPoseRepublisher()
 	{
		ros::NodeHandle _n;

 		ros::param::get("~topic", _topic);
 		
 		 ROS_WARN("Translating PoseStamped messages:");

 		if (!ros::param::has("~frame_id")) //$ if frame_id parameter not set
 		{
 			_frame_id_specified = false;
 			ROS_WARN("From: %s", _topic.c_str());

 		}
 		else 
 		{
 			_frame_id_specified = true;
 			ros::param::get("~frame_id", _frame_id);
 			ROS_WARN("From: %s [frame_id: %s]", _topic.c_str(), _frame_id.c_str());
 		}

 		ros::param::get("~repub_topic", _repub_topic);
 		ros::param::get("~repub_frame_id", _repub_frame_id);

 		ROS_WARN("To: %s [frame_id: %s]", _repub_topic.c_str(), _repub_frame_id.c_str());

    	//$ set up subscriber
 		_pose_sub = _n.subscribe<geometry_msgs::PoseStamped>(_topic, 1, &TranslatedPoseRepublisher::poseCallback, this);

    	//$ set up publisher
 		_pose_pub = _n.advertise<geometry_msgs::PoseStamped>(_repub_topic, 1);

 		ROS_WARN("Done initializing translate_pose.");

 	}

 	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
 	{
 		geometry_msgs::PoseStamped translated_msg;
 		translated_msg.header.frame_id = _repub_frame_id;
 		translated_msg.header.stamp = msg->header.stamp; 

 		if (msg->header.frame_id == _repub_frame_id)
 		{
 			//$ if frame_id for target topic and republished topic are the same
 			//$ just republish the original message
 			translated_msg.pose = msg->pose;
 			_pose_pub.publish(translated_msg);
 		}
 		else
 		{
	 		tf::StampedTransform transform;

	 		if (_frame_id_specified && (msg->header.frame_id != _frame_id))
	 		{
	 			ROS_ERROR("frame_id on %s topic does not match (expected %s, got %s)", _topic.c_str(), _frame_id.c_str(), msg->header.frame_id.c_str());
	 		}

	 		try
	 		{
	 			//$ look up transform between frames
	 			_tf.lookupTransform(msg->header.frame_id, _repub_frame_id, ros::Time(0), transform);

	 			translated_msg.pose.position.x = msg->pose.position.x - transform.getOrigin().x();
	 			translated_msg.pose.position.y = msg->pose.position.y - transform.getOrigin().y();
	 			translated_msg.pose.position.z = msg->pose.position.z - transform.getOrigin().z();
	 			_pose_pub.publish(translated_msg);
	 		}
	 		catch(tf::TransformException& ex) 
	 		{
	 			ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
	 				msg->header.frame_id.c_str(), _frame_id.c_str(), ex.what());
	 		}
	 	}
 	}


 private:
 	tf::TransformListener _tf;

 	ros::Subscriber _pose_sub;
 	ros::Publisher _pose_pub;

 	std::string _topic, _frame_id;
 	std::string _repub_topic, _repub_frame_id;

 	bool _frame_id_specified;

}; //$ end of class TranslatedPoseRepublisher

int main(int argc, char **argv) {

	ros::init(argc, argv, "translate_pose");

	TranslatedPoseRepublisher republisher;

	while (ros::ok())
	{
		ros::spinOnce();
	}

	return 0;
}

