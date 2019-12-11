#include <ros/ros.h>

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


void set_start_pose(const tf2_msgs::TFMessage::ConstPtr& _msg); // set start pose, which is current robot pose
//void set_goal_pose(const geometry_msgs::Pose::ConstPtr& _msg);  // set goal pose. This callback function not yet used by the planner. The goal pose needs to be published at some where. We hard coded the goal pose at line 33~34.
void set_octomap(const octomap_msgs::Octomap::ConstPtr& _msg);

bool project_environment(octomap::OcTree* _environment_3D, quadmap::QuadTree* _environment_2D); //convert 3D octomap to 2D quadmap


/*subscribe to goal pose and update flag*/
void set_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	start_plan_flag = 1;
    goal_pose = msg->pose;
    ROS_INFO("NEW GOAL!! goalX: %f, goalY: %f",msg->pose.position.x, msg->pose.position.y);
}



/*subscribe stop plan*/
void update_arrived_flag(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	start_plan_flag = 0;
    ROS_INFO("ARRIVED AT GOAL. STOP PLAN");
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile_path_planning");
    ros::NodeHandle nh;

    // Initialize the global variables
    start_pose.orientation.w = 1.0;
    // Goal pose is set as arbitrary values
    goal_pose.orientation.w = 1.0;
    goal_pose.position.x = 0.0;
    goal_pose.position.y = 0.0;
    environment = nullptr;

    // Publish output of path planning
    ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("mobile_hubo/navigation_path", 1);

    // Subscribe the current pose, goal pose, and obstacles map
    ros::Subscriber start_pose_subscriber = nh.subscribe("/tf", 1, set_start_pose);
    ros::Subscriber goal_pose_subscriber = nh.subscribe("/move_base_simple/goal", 10, set_goal_pose);
    ros::Subscriber arrival_subscriber = nh.subscribe("/mobile_hubo/arrived_path", 10, update_arrived_flag);
    ros::Subscriber map_subscriber = nh.subscribe("mobile_hubo/fake_map", 1, set_octomap);  // fake_octomap is used as collision map

    ros::Rate rate(10);
    while(nh.ok()){
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
		   
			AstarPlanner path_planner(environment_2D, RobotModel(0.0, 0.0, 0.4)); 
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
