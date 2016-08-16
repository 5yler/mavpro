/**
* fly_to_local_server.h
* Action server for FlyToLocal action
* 
* @author  Syler Wagner      <syler@mit.edu>

* @date    2016-08-13  syler   creation
*
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
       * @brief  Constructor for the actions
       * @param tf A reference to a TransformListener
       */
       FlyToLocalServer(tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       */
       virtual ~FlyToLocalServer();

    private:

      double controller_frequency_;
      double xy_tolerance_, z_tolerance_;

      void resetSetpointToCurrentPosition();
      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
       void resetState();

       geometry_msgs::PoseStamped goalToFCUFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

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

       tf::TransformListener& tf_;

       FlyToLocalActionServer* as_;
       mavpro::FlyToLocalResult _result;


       ros::Publisher current_goal_pub_, setpoint_pub_, action_goal_pub_;

       ros::Subscriber position_sub_;
       geometry_msgs::PoseStamped current_position_;

  };
};

#endif