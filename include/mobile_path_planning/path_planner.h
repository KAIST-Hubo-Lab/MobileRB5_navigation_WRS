#ifndef __PLANNER_H_
#define __PLANNER_H_

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

#include <mobile_path_planning/collision_checker.h>

#include <cmath>
#include <set>
#include <queue>

// #define USE_4_DIRECTION
#define USE_8_DIRECTION

struct AstarNode{
    // Constructor
    AstarNode(ConfigurationKey& _key, double _g, double _h, AstarNode* _parent) {
        key = _key;
        G = _g;
        H =_h;
        parent = _parent;
    }

    // Node information
    double G;   // cost to come
    double H;   // cost to go
    ConfigurationKey key;
    AstarNode* parent;

    // Score
    inline double getScore() { return G + H; }

    struct compare {
        bool operator()(AstarNode* _a, AstarNode* _b) {
            return _a->H > _b->H;
        };
    };
};

class OpenSet : public std::priority_queue<AstarNode*, std::vector<AstarNode*>, AstarNode::compare>
{
public:
    AstarNode* find(const ConfigurationKey& _key) const {
        for(std::vector<AstarNode*>::const_iterator it = this->c.cbegin(); it != this->c.cend(); it++){
            if((*it)->key == _key)
                return *it;
        }
        return nullptr;
    }
};

struct AstarPlanner{
public:
    AstarPlanner(const quadmap::QuadTree* _environment, const RobotModel& _robot);
    ~AstarPlanner();

    bool planning(const geometry_msgs::Pose& _start_pose, const geometry_msgs::Pose& _goal_pose, nav_msgs::Path& _nav_path);

private:
    typedef unordered_ns::unordered_map<ConfigurationKey, AstarNode*, ConfigurationKey::KeyHash> ClosedSet;

    inline double cost_to_come(ConfigurationKey& _from, ConfigurationKey& _to){
        int delta[2] = {_to[0] - _from[0], _to[1] - _from[1]};
        return sqrt((double)(delta[0]*delta[0] + delta[1]*delta[1]));
    }

    inline double cost_to_go(ConfigurationKey& _from, ConfigurationKey& _to){
        int delta[2] = {_to[0] - _from[0], _to[1] - _from[1]};
        return sqrt((double)(delta[0]*delta[0] + delta[1]*delta[1]));
    }


    void refinePath(std::vector<Configuration>& _path, nav_msgs::Path& _refined_path);

    inline bool isCollide(const ConfigurationKey& _conf_key){
        if(collision_map->collision_hash.find(_conf_key) != collision_map->collision_hash.end())
            return collision_map->collision_hash.find(_conf_key)->second;

        const Configuration& conf = collision_map->keyToCoord(_conf_key);
        robotmodel.center.x() = (float)conf.x();
        robotmodel.center.y() = (float)conf.y();

        bool collision_result = collision_checker->doCollide(environment, robotmodel);
        collision_map->collision_hash.insert(std::pair<ConfigurationKey, bool>(_conf_key, collision_result));
        return collision_result;
    }

    inline AstarNode* findNodeOnList(std::set<AstarNode*>& _nodes, const ConfigurationKey& _key){
        for(auto node : _nodes){
            if(node->key == _key){
                return node;
            }
        }

        return nullptr;
    }

    inline void releaseNodes(OpenSet& _nodes){
        while(!_nodes.empty()){
            delete _nodes.top();
            _nodes.pop();
        }
    }
    inline void releaseNodes(ClosedSet& _nodes){
        for(auto it = _nodes.begin(); it != _nodes.end(); it++){
            delete it->second;
        }
    }

    const quadmap::QuadTree* environment;
    RobotModel robotmodel;

    CollisionChecker* collision_checker;
    ConfigurationMap* collision_map;
    std::vector<ConfigurationKey> direction;


};

#endif
