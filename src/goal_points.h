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

using namespace std;

struct GoalPoint
{
    public:
        double x,y,z,roll,pitch,yaw;

    GoalPoint(double x, double y, double z, double roll, double pitch, double yaw)
    : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
};

class GoalPoints
{
    public:
        GoalPoints();
        void increment_goal();
        void reset_task();
        bool is_task_done();
        void add_goal(double x, double y, double z, double roll, double pitch, double yaw);
        bool check_init_goal(double x_cur, double y_cur, double z_cur, double roll_cur, double pitch_cur, double yaw_cur);
        bool check_goal(double x_cur, double y_cur, double z_cur, double roll_cur, double pitch_cur, double yaw_cur);
        void visualize_init_points();
        void visualize_goal_points();


        vector<GoalPoint> goal_points;
        ros::Publisher vis_pub;
        unsigned int cur_goal_it;
        bool task_done;
};