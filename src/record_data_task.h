#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <sstream>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <rosbag/bag.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include "goal_points.h"
using namespace std;



typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RecordDataTask
{
public:
	RecordDataTask(GoalPoints goal_input);
	void check_initial_configuration();
	void tf_callback(const tf::tfMessage::ConstPtr& msg);
	void joint_callback(const sensor_msgs::JointState::ConstPtr& msg);
	void begin(bool record_entire_trajectory_input, bool record_endpoints_input, bool playback_input);
	void write_csv(string filename, vector<double> input);
	
	
	
	

    bool shutdown;
    bool task_done;
    bool init_mode;
    rosbag::Bag bag;
    ros::Subscriber tf_sub;
    ros::Subscriber joint_sub;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    GoalPoints goal;
    int cur_it;

    bool record_entire_trajectory;
    bool record_endpoints;
    bool playback;
    vector<double> endpoints;
    TrajClient* traj_client_;

};