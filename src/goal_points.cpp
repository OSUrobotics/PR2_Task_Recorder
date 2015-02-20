#include "goal_points.h"




GoalPoints::GoalPoints()
{
    ros::NodeHandle node_handle;
    vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );        
    cur_goal_it = 0;
}
void GoalPoints::increment_goal()
{
    cur_goal_it++;
    task_done = false;
    if(cur_goal_it >= goal_points.size())
    {
        cur_goal_it--;
        task_done = true;
    }
}
void GoalPoints::reset_task()
{
    cur_goal_it = 0;
}
bool GoalPoints::is_task_done()
{
    return task_done;
}
void GoalPoints::add_goal(double x, double y, double z, double roll, double pitch, double yaw)
{
    GoalPoint cur_point(x,y,z,roll,pitch,yaw);
    goal_points.push_back(cur_point);
}

bool GoalPoints::check_init_goal(double x_cur, double y_cur, double z_cur, double roll_cur, double pitch_cur, double yaw_cur)
{
    double x = 0.790274;
    double y = 0.115416;
    double z = 0.718985;
    double roll = -0.485113;
    double pitch = -0.819378;
    double yaw = 0.269066;
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
bool GoalPoints::check_goal(double x_cur, double y_cur, double z_cur, double roll_cur, double pitch_cur, double yaw_cur)
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

void GoalPoints::visualize_init_points()
{
    double x = 0.790274;
    double y = 0.115416;
    double z = 0.718985;
    double roll = -0.485113;
    double pitch = -0.819378;
    double yaw = 0.269066;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
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
void GoalPoints::visualize_goal_points()
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