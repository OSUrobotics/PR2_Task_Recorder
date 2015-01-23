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
#define DEBUG true


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
        vector<GoalPoint> goal_points;
        ros::Publisher vis_pub;
        unsigned int cur_goal_it;
        bool task_done;

    GoalPoints()
    {
        ros::NodeHandle node_handle;
        vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );        
        cur_goal_it = 0;
    }
    void increment_goal()
    {
        cur_goal_it++;
        task_done = false;
        if(cur_goal_it >= goal_points.size())
        {
            cur_goal_it--;
            task_done = true;
        }
    }
    void reset_task()
    {
        cur_goal_it = 0;
    }
    bool is_task_done()
    {
        return task_done;
    }
    void add_goal(double x, double y, double z, double roll, double pitch, double yaw)
    {
        GoalPoint cur_point(x,y,z,roll,pitch,yaw);
        goal_points.push_back(cur_point);
    }

    bool check_init_goal(double x_cur, double y_cur, double z_cur, double roll_cur, double pitch_cur, double yaw_cur)
    {
        double x = 0.6;
        double y = 0.6;
        double z = 0.6;
        double roll = -2.7;
        double pitch = 0;
        double yaw = 0;
        double x_diff = abs(x_cur - x);
        double y_diff = abs(y_cur - y);
        double z_diff = abs(z_cur - z);
        double roll_diff = abs(roll_cur - roll);
        double pitch_diff = abs(pitch_cur - pitch);
        double yaw_diff = abs(yaw_cur - yaw);


        if(roll_diff > 3.14) roll_diff -= 3.14;
        if(yaw_diff > 3.14) yaw_diff -= 3.14;
        cout << roll_diff << " ";
        cout << pitch_diff << " ";
        cout << yaw_diff << " " << endl;
        if(x_diff < 0.1 && y_diff < 0.1 && z_diff < 0.1)
        {
            cout << "CORRECT XYZ";
            if(roll_diff < 0.5 && pitch_diff < 0.5 && yaw_diff < 0.5)
            {
                return true;
            }
        }
        return false;
    }
    bool check_goal(double x_cur, double y_cur, double z_cur, double roll_cur, double pitch_cur, double yaw_cur)
    {
        GoalPoint cur_point = goal_points[cur_goal_it];
        double x = cur_point.x;
        double y = cur_point.y;
        double z = cur_point.z;
        double roll = cur_point.roll;
        double pitch = cur_point.pitch;
        double yaw = cur_point.yaw;
        cout << endl;
        double x_diff = abs(x_cur - x);
        double y_diff = abs(y_cur - y);
        double z_diff = abs(z_cur - z);
        double roll_diff = abs(roll_cur - roll);
        double pitch_diff = abs(pitch_cur - pitch);
        double yaw_diff = abs(yaw_cur - yaw);


        if(roll_diff > 3.14) roll_diff -= 3.14;
        if(yaw_diff > 3.14) yaw_diff -= 3.14;
        cout << roll_diff << " ";
        cout << pitch_diff << " ";
        cout << yaw_diff << " " << endl;
        if(x_diff < 0.1 && y_diff < 0.1 && z_diff < 0.1)
        {
            cout << "CORRECT XYZ";
            if(roll_diff < 0.5 && pitch_diff < 0.5 && yaw_diff < 0.5)
            {
                return true;
            }
        }
        return false;
    }
    void visualize_goal_points()
    {
        GoalPoint cur_point = goal_points[cur_goal_it];
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cur_point.x;
        marker.pose.position.y = cur_point.y;
        marker.pose.position.z = cur_point.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(cur_point.roll, cur_point.pitch, cur_point.yaw);
        /*
        if (DEBUG)
        {
            cout << q.x();
            cout << q.y();
            cout << q.z();
            cout << q.w();
        }
        */
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
    
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        vis_pub.publish( marker );
    }
};
class RecordDataTask
{
public:
    bool shutdown;
    bool task_done;
    bool init_mode;
    rosbag::Bag bag;
    ros::Subscriber tf_sub;
    ros::Subscriber joint_sub;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    GoalPoints cup_turning;
    int cur_it;

    void check_initial_configuration()
    {
        bool goal_found = false;
        do
        {
            listener.waitForTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), transform);
            tf::Quaternion q = transform.getRotation();
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            goal_found = cup_turning.check_init_goal(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z(),roll,pitch,yaw);
        } while (!goal_found); 
    }

    void tf_callback(const tf::tfMessage::ConstPtr& msg)
    {
        if(shutdown) return;
        if(init_mode) return;
        /*
        if(cur_it > 100)
        {   
            cur_it = 0;
            cup_turning.increment_goal();
        }
        */
        cup_turning.visualize_goal_points();
        
        listener.waitForTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), transform);
        
        bag.write("tf", ros::Time::now(), msg);

        tf::Quaternion q = transform.getRotation();
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        bool goal_found = cup_turning.check_goal(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z(),roll,pitch,yaw);
        if (DEBUG)
        {
            cout << "left:" << '\n';
            cout << "X: " << transform.getOrigin().x() << '\n';
            cout << "Y: " << transform.getOrigin().y() << '\n';
            cout << "Z: " << transform.getOrigin().z() << '\n';
            cout << "roll: " << roll << '\n';
            cout << "pitch: " << pitch << '\n';
            cout << "yaw: " << yaw << '\n';
            cout << goal_found << endl;
        }

        if(goal_found)
        {
            cup_turning.increment_goal();
            if(cup_turning.is_task_done())
            {
                cur_it++;
                bag.close();
                int i;
                cout << "Would you like another? (1/0).";
                cin >> i;
                if(i)
                {
                    init_mode = true;
                    check_initial_configuration();
                    init_mode = false;
                    std::ostringstream s;
                    s << "test-" << cur_it << ".bag";
                    bag.open(s.str(), rosbag::bagmode::Write);
                    cup_turning.reset_task();
                }
                else
                {
                    shutdown = true;
                    ros::shutdown();
                }
                //Prompt user for another experiment.
                //Put them in a "random/valid" state. Right now it's up to the user.
            }
        }
        /*
        listener.waitForTransform("/base_link", "/r_gripper_palm_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/base_link", "/r_gripper_palm_link", ros::Time(0), transform);
        tf::Quaternion q2 = transform.getRotation();
        tf::Matrix3x3 m2(q2);
        m.getRPY(roll, pitch, yaw);
        if (DEBUG)
        {
            cout << "right:" << '\n';
            cout << "X: " << transform.getOrigin().x() << '\n';
            cout << "Y: " << transform.getOrigin().y() << '\n';
            cout << "Z: " << transform.getOrigin().z() << '\n';
            cout << "roll: " << roll << '\n';
            cout << "pitch: " << pitch << '\n';
            cout << "yaw: " << yaw << '\n';
        }
        */

    }
    void joint_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        if(shutdown) return;
        if(init_mode) return;
        bag.write("joint_states", ros::Time::now(), msg);
    }      
    RecordDataTask()
    {
        init_mode = true;
        shutdown = false;
        std::ostringstream s;
        cur_it = 0;
        s << "test-" << cur_it << ".bag";
        bag.open(s.str(), rosbag::bagmode::Write);
        listener.waitForTransform("/base_link", "/l_gripper_r_finger_link", ros::Time::now(), ros::Duration(3.0));
        ros::NodeHandle nh;
        tf_sub = nh.subscribe("tf", 1000, &RecordDataTask::tf_callback, this);
        joint_sub = nh.subscribe("joint_states", 1000, &RecordDataTask::joint_callback, this);
        cup_turning.add_goal(0.669708, 0.25714, 0.503503, -2.79872, -0.0415155, -0.0286199);
        cup_turning.add_goal(0.676796, 0.62424, 0.685592, -2.79861, -0.0419864, -0.0269426);
        cup_turning.add_goal(0.676832, 0.611959, 0.723373,  1.3262, -0.0322003, -0.0362714);
        cur_it = 0;
    }
};
/*
Cup Turning:
1st:
left:
X: 0.669708
Y: 0.25714
Z: 0.503503
roll: -2.79872
pitch: -0.0415155
yaw: -0.0286199

2nd:
left:
X: 0.676796
Y: 0.624241
Z: 0.685592
roll: -2.79861
pitch: -0.0419864
yaw: -0.0269426

3rd:
left:
X: 0.676832
Y: 0.611959
Z: 0.723373
roll: 1.3262
pitch: -0.0322003
yaw: -0.0362714

Shelf:
1st:
left:
X: 0.410139
Y: 0.226037
Z: 1.18755
roll: 1.32623
pitch: -0.0336219
yaw: -0.0370385

2nd:
left:
X: 0.561775
Y: 0.206725
Z: 1.17391
roll: 0.132723
pitch: -0.0384217
yaw: -0.0420233

3rd:
left:
X: 0.410139
Y: 0.226037
Z: 1.18755
roll: 0.132723
pitch: -0.0384217
yaw: -0.0420233

4th:
left:
X: 0.808507
Y: 0.323432
Z: 0.877818
roll: 0.132723
pitch: -0.0384217
yaw: -0.0420233

Random:
1st:
left:
X: 0.733454
Y: 0.0298174
Z: 0.674502
roll: 1.52469
pitch: 0.115068
yaw: -0.878966
right:
X: 0.726977
Y: -0.413603
Z: 0.718715
roll: 1.52469
pitch: 0.115068
yaw: -0.878966

2nd:
left:
X: 0.492616
Y: 0.636465
Z: 0.301991
roll: 0.602759
pitch: 0.821469
yaw: 2.18754
right:
X: 0.522646
Y: -0.415537
Z: 0.653796
roll: 0.602759
pitch: 0.821469
yaw: 2.18754

*/
int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "record_data_task");
    //RecordDataTask* a = new RecordDataTask();
    ros::spin();
    //cout << transform.getOrigin().x();
    //cout << transform.getOrigin().y();
    //cout << transform.getOrigin().z();
}