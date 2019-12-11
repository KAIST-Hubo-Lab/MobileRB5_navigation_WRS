#include <mobile_path_planning/path_planner.h>

#include <algorithm>

AstarPlanner::AstarPlanner(const quadmap::QuadTree* _environment, const RobotModel& _robot)
        : environment(_environment)
{
    collision_checker = new CollisionChecker();
    robotmodel = _robot;

    // collision_map_voxel_size : If this value is low, the map become high resolution. This value is user definable
    double collision_map_voxel_size = _environment->getResolution() * 2;
    // collision_map = new ConfigurationMap(_environment->getResolution());
    collision_map = new ConfigurationMap(collision_map_voxel_size);

#ifdef USE_4_DIRECTION
    direction.resize(4);
    direction[0] = ConfigurationKey(1, 0);
    direction[1] = ConfigurationKey(0, 1);
    direction[2] = ConfigurationKey(-1, 0);
    direction[3] = ConfigurationKey(0, -1);
#else
    #ifdef USE_8_DIRECTION
    direction.resize(8);
    direction[0] = ConfigurationKey(1, 0);
    direction[1] = ConfigurationKey(1, 1);
    direction[2] = ConfigurationKey(0, 1);
    direction[3] = ConfigurationKey(-1, 1);
    direction[4] = ConfigurationKey(-1, 0);
    direction[5] = ConfigurationKey(-1, -1);
    direction[6] = ConfigurationKey(0, -1);
    direction[7] = ConfigurationKey(1, -1);
#endif
#endif
}

AstarPlanner::~AstarPlanner()
{
    if(collision_checker)
        delete collision_checker;

    if(collision_map)
        delete collision_map;
}

bool AstarPlanner::planning(const geometry_msgs::Pose& _start_pose, const geometry_msgs::Pose& _goal_pose, nav_msgs::Path& _nav_path)
{
    // Check inputs
    ConfigurationKey startKey = collision_map->coordToKey(_start_pose.position.x, _start_pose.position.y);
    ConfigurationKey goalKey = collision_map->coordToKey(_goal_pose.position.x, _goal_pose.position.y);
    if(startKey == goalKey)
        return true;

    // Check incorrect inputs
    if(isCollide(startKey) || isCollide(goalKey))
        return false;

    // Initialize
    AstarNode* current = nullptr;
    OpenSet openSet;
    ClosedSet closedSet;
    openSet.push(new AstarNode(startKey, 0, cost_to_go(startKey, goalKey), nullptr));

    // Try to find path
    while(!openSet.empty()){
        // Get current node
        current = openSet.top();

        // Found goal
        if(current->key == goalKey){
            break;
        }

        closedSet.insert(std::pair<ConfigurationKey, AstarNode*>(current->key, current));
        openSet.pop();

        // Check new candidates
        for(int i = 0; i < (int)direction.size(); ++i){
            ConfigurationKey newKey = current->key;
            newKey.k[0] += direction[i].k[0];
            newKey.k[1] += direction[i].k[1];

            if(isCollide(newKey) || closedSet.find(newKey) != closedSet.end())
                continue;

            AstarNode* neighbor = openSet.find(newKey);

            double g_score = current->G + cost_to_come(current->key, newKey);
            double h_score = g_score + cost_to_go(newKey, newKey);

            if(neighbor == nullptr){
                neighbor = new AstarNode(newKey, g_score, h_score, current);
                openSet.push(neighbor);
            }
            else if(g_score < neighbor->G){
                neighbor->parent = current;
                neighbor->G = g_score;
                neighbor->H = h_score;
            }
        }
    }

    // Refine the path
    std::vector<Configuration> path;
    while(current != nullptr){
        path.push_back(collision_map->keyToCoord(current->key));
        current = current->parent;
    }
    refinePath(path, _nav_path);

    // Release memories
    releaseNodes(openSet);
    releaseNodes(closedSet);

    return true;
}

void AstarPlanner::refinePath(std::vector<Configuration>& _path, nav_msgs::Path& _refined_path)
{
    // Convert grid based path to nav_msgs::Path

    _refined_path.header.frame_id = "map";
    _refined_path.header.stamp = ros::Time::now();

    for(int i = (int)_path.size() - 1; i > 0; i--){
        quadmath::Vector2 from((float)_path[i].x(), (float)_path[i].y());
        quadmath::Vector2 to((float)_path[i-1].x(), (float)_path[i-1].y());
        quadmath::Vector2 delta = to - from;

        geometry_msgs::PoseStamped path_node;
        path_node.header.frame_id = _refined_path.header.frame_id;
        path_node.header.stamp = _refined_path.header.stamp;

        path_node.pose.position.x = from.x();
        path_node.pose.position.y = from.y();
        path_node.pose.position.z = 0.0;
        //euler angle to quaternion
        double rotation = atan2(delta.y()/delta.norm(), delta.x()/delta.norm()); // acos(delta.norm() / delta.x());
        octomath::Quaternion q(0.0, 0.0, rotation);
        path_node.pose.orientation.x = q.x();
        path_node.pose.orientation.y = q.y();
        path_node.pose.orientation.z = q.z();
        path_node.pose.orientation.w = q.u();

        _refined_path.poses.push_back(path_node);
    }
}
