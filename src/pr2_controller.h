#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <tf/transform_listener.h>

using namespace std;

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

typedef map<string, vector<double>* > map_string_2d_t;

class RobotArmLeft
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
    TrajClient* traj_client_;
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    bool record_endpoint;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    vector<double> endpoint_positions;

public:
    RobotArmLeft();
    void startTrajectory();
    void start_arm_extension_trajectory(int waypoints);
    void add_arm_extension_trajectory(int cur_trajectory, map_string_2d_t* trajectory_map);
    actionlib::SimpleClientGoalState getState();
    void record_endpoint_positions();
    ~RobotArmLeft();

};