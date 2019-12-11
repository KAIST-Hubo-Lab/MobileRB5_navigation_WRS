/* =============================================================
 *
 * This ROS node is Navigation Action Server for Mobile-Hubo Platform.
 * Receives Navi Action goal and outputs feedback / result.
 * In order for correct operation, please ensure all octomap related packages are installed. 
 * Navi Action header files must also be included in the folder.
 * Refer to Navigation Manual at www.kirobotics.com
 * 
 *
 * Output : /hubo_navigation/result
 * 			/hubo_navigation/feedback
 * 			/hubo_navigation/status
 * 
 * Input  : /hubo_navigation/goal
 * 			/mobile_hubo/fake_map
			/tf
 
 * 
 * E-mail : ml634@kaist.ac.kr    (Moonyoung Lee)
 * E-mail : shn4438@gmail.com (Shin Heechan)
 *
 * Versions :
 * v1.0.2019.10
 * =============================================================
 */
 

/*for ROS Action msg */
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

/*Pre-defined Action msg*/
#include <mobile_path_planning/naviAction.h>


#include <stdio.h>
#include <string.h>

/* path planning related files*/

#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Path.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <mobile_path_planning/path_planner.h>

#define MININUM_HEIGHT 0.0
#define MAXIMUM_HEIGHT 1.0
#define arrival_threshold 2 //units of grid

// Global variable for subscribing the data
geometry_msgs::Pose start_pose;
geometry_msgs::Pose goal_pose;
octomap::OcTree* environment;
geometry_msgs::PoseStamped test_poseStamp;
int start_plan_flag = 0;
int arrived_flag = 0;

void set_start_pose(const tf2_msgs::TFMessage::ConstPtr& _msg); // set start pose, which is current robot pose
void set_octomap(const octomap_msgs::Octomap::ConstPtr& _msg);
void update_arrived_flag(const geometry_msgs::PoseStamped::ConstPtr& _msg);
bool project_environment(octomap::OcTree* _environment_3D, quadmap::QuadTree* _environment_2D); //convert 3D octomap to 2D quadmap


/*subscribe stop plan*/
void update_arrived_flag(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	start_plan_flag = 0;
	arrived_flag = 1;
    ROS_INFO("ARRIVED AT GOAL. STOP PLAN");
}

/*===================== Action =====================*/
class naviAction
{
protected:
    ros::NodeHandle nh_navi;
    actionlib::SimpleActionServer<mobile_path_planning::naviAction> asNavi_;
    std::string action_name_;

    // create messages that are used to published feedback&result
    mobile_path_planning::naviFeedback feedback_;
    mobile_path_planning::naviResult result_;


public:

    naviAction(std::string name) :
        asNavi_(nh_navi, name, boost::bind(&naviAction::executeCB, this, _1), false),
        action_name_(name)
    {
        asNavi_.start();
    }

    ~naviAction(void)
    {
    }



    /*Call Back function when goal is received from Action client*/
    void executeCB(const mobile_path_planning::naviGoalConstPtr &goal)
    {
        ros::Rate r(10);
        r.sleep();

        std::cout << std::endl << "navi_dummy CB" <<std::endl;
        
        //update flags 
        start_plan_flag = 1;
        arrived_flag = 0;

        //Print recieved msg
        std::cout <<"use_marker: " << goal->use_marker << std::endl;
        goal_pose.position.x = goal->pose_x;
        goal_pose.position.y = goal->pose_y;
        goal_pose.position.z = goal->pose_z;
        goal_pose.orientation.x = goal->ori_x;
        goal_pose.orientation.y = goal->ori_y;
        goal_pose.orientation.z = goal->ori_z;
        goal_pose.orientation.w = goal->ori_w;
        
        ROS_INFO("NEW GOAL!! goalX: %f, goalY: %f",goal_pose.position.x, goal_pose.position.y);

 

		while(arrived_flag == 0)
		{
			r.sleep();
		}
		
        //set Result
        result_.result_flag = 1;
        result_.pose_x = start_pose.position.x;
        result_.pose_y = start_pose.position.y;
        result_.pose_z = start_pose.position.z;
        result_.ori_x = start_pose.orientation.x;
        result_.ori_y = start_pose.orientation.y;
        result_.ori_z = start_pose.orientation.z;
        result_.ori_w = start_pose.orientation.w;

		ROS_INFO("Action Finished! ResultX: %f, ResultY: %f",result_.pose_x, result_.pose_y);

        asNavi_.setSucceeded(result_);

    }
};
/*===================== End of Action =====================*/

/*===================== Main Loop =====================*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "navi_dummy");
  ros::NodeHandle n;
  
   // Initialize the global variables
    start_pose.orientation.w = 1.0;
    // Goal pose is set as arbitrary values
    goal_pose.orientation.w = 1.0;
    goal_pose.position.x = 0.0;
    goal_pose.position.y = 0.0;
    environment = nullptr;

    // Publish output of path planning
    ros::Publisher path_publisher = n.advertise<nav_msgs::Path>("mobile_hubo/navigation_path", 1);

	// Subscribe the current pose, goal pose, and obstacles map
    ros::Subscriber start_pose_subscriber = n.subscribe("/tf", 1, set_start_pose);
	ros::Subscriber arrival_subscriber = n.subscribe("/mobile_hubo/arrived_path", 10, update_arrived_flag);
    ros::Subscriber map_subscriber = n.subscribe("mobile_hubo/fake_map", 1, set_octomap);  // fake_octomap is used as collision map
    
	naviAction navi_dummy("hubo_navigation");
	ROS_INFO("Starting hubo_navigation module");

	ros::Rate rate(10);
	
    while(n.ok())
    {
		// wait until the 'environment' is set
        if(environment == nullptr){
            ros::spinOnce();
            continue;
        }

        // 1. Prepare a data for path planning
        ros::Time start = ros::Time::now();
        
        quadmap::QuadTree* environment_2D = new quadmap::QuadTree(environment->getResolution());
        project_environment(environment, environment_2D);

        // 2. Path planning
        nav_msgs::Path path;

        if(start_plan_flag == 1) //start condition
        {
			// RobotModel : [1: mobileHUBO coordinate (0,0), radius 0.4 ], [2: DRCHUBO coordinate (0,0), radius 0.8 ]
		   
			AstarPlanner path_planner(environment_2D, RobotModel(0.0, 0.0, 0.45)); 
			path_planner.planning(start_pose, goal_pose, path);

			ros::Time end = ros::Time::now();

			double path_planning_time = (end - start).toSec() * 1000;
			//std::cout << "Planning Time: " << path_planning_time << " [ms]" << std::endl;
			

			delete environment_2D;
			
	
			 // 3. Publish the path
			 
			 if(path.poses.empty())
			 {
				 ROS_ERROR("empty path detected");
			 }
			 
			 else
			 {
				 
				 std::cout << "Path size: " << path.poses.size() << std::endl;
				 path_publisher.publish(path);
			 }

		}
		
        rate.sleep();

        ros::spinOnce();
	}
	
	return 0;
}


void set_start_pose(const tf2_msgs::TFMessage::ConstPtr& _msg)
{
    // Convert tf2_msgs to geometry_msgs::Pose
    for(int i = 0; i < _msg->transforms.size(); i++){
        if(std::string("base_link").compare(_msg->transforms.at(i).child_frame_id) == 0){
            start_pose.position.x = _msg->transforms.at(i).transform.translation.x;
            start_pose.position.y = _msg->transforms.at(i).transform.translation.y;
            start_pose.position.z = 0.0;

            start_pose.orientation.w = _msg->transforms.at(i).transform.rotation.w;
            start_pose.orientation.x = _msg->transforms.at(i).transform.rotation.x;
            start_pose.orientation.y = _msg->transforms.at(i).transform.rotation.y;
            start_pose.orientation.z = _msg->transforms.at(i).transform.rotation.z;

            break;
        }
    }

    //std::cout << "Set start pose" << std::endl;
}


void set_octomap(const octomap_msgs::Octomap::ConstPtr& _msg)
{
    if(environment == nullptr)
        delete environment;

    environment = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*_msg);
    //std::cout << "Set octomap" << std::endl;
}

bool project_environment(octomap::OcTree* _environment_3D, quadmap::QuadTree* _environment_2D)
{
    if(_environment_3D == nullptr)
        return false;

    _environment_3D->expand();
    for(octomap::OcTree::leaf_iterator it = _environment_3D->begin_leafs(); it != _environment_3D->end_leafs(); it++){
        octomap::point3d point = _environment_3D->keyToCoord(it.getKey());
        if(point.z() < MININUM_HEIGHT && point.z() > MAXIMUM_HEIGHT)
            continue;
        _environment_2D->updateNode(point.x(), point.y(), true);
    }

    return true;
}

