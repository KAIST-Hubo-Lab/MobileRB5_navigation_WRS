
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

/*custom defined Action header for robot motion */
#include <mobile_path_planning/naviAction.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include  <tf/tf.h>

/* global variable */
// send a goal to the action
mobile_path_planning::naviGoal goal_navi;
  
int received_goal = 0;

/*subscribe to goal pose and update flag*/
void set_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
		ROS_INFO("Client Requesting NEW GOAL!! goalX: %f, goalY: %f",msg->pose.position.x, msg->pose.position.y);
    
    received_goal = 1;
     /* ============== Base Data Action  ==============  */
	  goal_navi.use_marker = 1;
	  goal_navi.pose_x = msg->pose.position.x;
	  goal_navi.pose_y = msg->pose.position.y;
	  goal_navi.pose_z = msg->pose.position.z;
	  
	  goal_navi.ori_x = msg->pose.orientation.x;
	  goal_navi.ori_y = msg->pose.orientation.y;
	  goal_navi.ori_z = msg->pose.orientation.z;
	  goal_navi.ori_w = msg->pose.orientation.w;

	  
  
  
  
}


int main (int argc, char **argv)
{
	

	ros::init(argc, argv, "testClient");
	ros::NodeHandle nh;
	
	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<mobile_path_planning::naviAction> ac_navi("hubo_navigation", true);


  ros::Subscriber goal_pose_subscriber = nh.subscribe("/move_base_simple/goal", 10, set_goal_pose);

  
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_navi.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  
   ros::Rate rate(10);
   while(nh.ok())
   {
		
		if(received_goal == 1)
		{
			ac_navi.sendGoal(goal_navi);
			received_goal = 0;
		}
		
		rate.sleep();

        ros::spinOnce();
		
	}
		  

  //exit
  return 0;
}
