
#include "record_data_task.h"

#define DEBUG true





void RecordDataTask::check_initial_configuration()
{
    goal.visualize_init_points();
    bool goal_found = false;
    do
    {
        listener.waitForTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), transform);
        tf::Quaternion q = transform.getRotation();
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        goal_found = goal.check_init_goal(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z(),roll,pitch,yaw);
        goal.visualize_init_points();
    } while (!goal_found);
    init_mode = false;
}

void RecordDataTask::tf_callback(const tf::tfMessage::ConstPtr& msg)
{
    if(shutdown) return;
    if(init_mode) return;
    if(record_entire_trajectory) bag.write("tf", ros::Time::now(), msg);


    goal.visualize_goal_points();
    /*
    if(cur_it > 100)
    {   
        cur_it = 0;
        cup_turning.increment_goal();
    }
    */
    
    listener.waitForTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), transform);
    
    

    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    endpoints.push_back(x);
    endpoints.push_back(y);
    endpoints.push_back(z);
    endpoints.push_back(roll);
    endpoints.push_back(pitch);
    endpoints.push_back(yaw);
    bool goal_found = goal.check_goal(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z(),roll,pitch,yaw);
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
    if(playback)
    {
        bool done = false;
        done = traj_client_->getState().isDone();
        if(done)
        {
            write_csv("endpoints.csv", endpoints);
            shutdown = true;
            ros::shutdown();
        }
    }
    if(goal_found)
    {
        goal.increment_goal();
        if(goal.is_task_done())
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
                if(record_entire_trajectory)
                {
                    std::ostringstream s;
                    s << "test-" << cur_it << ".bag";
                    bag.open(s.str(), rosbag::bagmode::Write);
                }
                goal.reset_task();
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
void RecordDataTask::joint_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(shutdown) return;
    if(init_mode) return;
    if(record_entire_trajectory) bag.write("joint_states", ros::Time::now(), msg);
}

void RecordDataTask::begin(bool record_entire_trajectory_input, bool record_endpoints_input, bool playback_input)
{
    record_entire_trajectory = record_entire_trajectory_input;
    record_endpoints = record_endpoints_input;
    playback = playback_input;


    if(record_entire_trajectory)
    {
        std::ostringstream s;
        cur_it = 0;
        s << "test-" << cur_it << ".bag";
        bag.open(s.str(), rosbag::bagmode::Write);
    }

    if(playback)
    {
        traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);
    }
    listener.waitForTransform("/base_link", "/l_gripper_r_finger_link", ros::Time::now(), ros::Duration(3.0));
    ros::NodeHandle nh;
    tf_sub = nh.subscribe("tf", 1000, &RecordDataTask::tf_callback, this);
    joint_sub = nh.subscribe("joint_states", 1000, &RecordDataTask::joint_callback, this);

    goal.visualize_goal_points();
    check_initial_configuration();
}

void RecordDataTask::write_csv(string filename, vector<double> input)
{
    ofstream output_file(filename.c_str());
    for(unsigned int i = 0; i < input.size(); i++)
    {
        if(i == input.size() - 1)
        {
            output_file << input[i];
        }
        else
        {
            output_file << input[i] << ",";
        }
    }
}

RecordDataTask::RecordDataTask(GoalPoints goal_input)
{
    init_mode = true;
    shutdown = false;
    goal = goal_input;
    record_entire_trajectory = false;
    record_endpoints = false;
}

/*
int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "record_data_task");
    GoalPoints cup_turning;
    cup_turning.add_goal(0.669708, 0.25714, 0.503503, -2.79872, -0.0415155, -0.0286199);
    cup_turning.add_goal(0.676796, 0.62424, 0.685592, -2.79861, -0.0419864, -0.0269426);
    cup_turning.add_goal(0.676832, 0.611959, 0.723373,  1.3262, -0.0322003, -0.0362714);
    RecordDataTask* a = new RecordDataTask(cup_turning);
    a->begin(false, true, true);
    ros::spin();

}
*/


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