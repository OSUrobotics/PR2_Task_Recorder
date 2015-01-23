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

typedef map<string, vector<double> > TrajectoryData;
typedef map<string, TrajectoryData*> map_string_2d_t;


class Parser
{
public:
	map_string_2d_t* traj_map;
	Parser()			
	{
		traj_map = new map_string_2d_t();
	}
	~Parser()
	{
		delete traj_map;
	}
	//Example: traj_map->find("fr_caster_l_wheel_joint")->second->find("position")->second.at(1);
	void parse_dictionary(string filename)
	{
		ifstream inf("/nfs/attic/smartw/users/curranw/pr2_pca_experiment/src/pr2_pca_experiment/src/workfile");
		string s;
		char c;

		//Read outer key
		while(true)
		{
			string outer_field;
			getline(inf, outer_field, '\'');
			getline(inf, outer_field, '\'');
			traj_map->insert(pair<string, TrajectoryData*> (outer_field, new TrajectoryData()));
			//Read inner key
			while(true)
			{
				string inner_field;
				TrajectoryData* traj_data = traj_map->find(outer_field)->second;
				getline(inf, inner_field, '\'');
				getline(inf, inner_field, '\'');
				getline(inf, s, '[');
				getline(inf, s, ']');
				vector<double> lst = parse_csv(s);		
				inf.get(c);
				cout << c;
				traj_data->insert(pair<string, vector<double> >(inner_field,lst));
				if (c == '}') break;
			}
			inf.get(c);
			cout << c;
			if (c == '}') break;
		}
	}

	vector<double> parse_csv(string input)
	{
		stringstream ss(input);
		string s;
		vector<double> output;
		while(getline(ss,s,','))
		{
			output.push_back(strtod(s.c_str(), NULL));
		}
		return output;
	}
};
class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
	TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
	RobotArm() 
	{
    // tell the action client that we want to spin a thread by default
		traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
		while(!traj_client_->waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the joint_trajectory_action server");
		}
	}

  //! Clean up the action client
	~RobotArm()
	{
		delete traj_client_;
	}

  //! Sends the command to start a given trajectory
	void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
	{
    // When to start the trajectory: 1s from now
		goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
		traj_client_->sendGoal(goal);
	}

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
      pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
      {
    //our goal variable
      	pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
      	goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
      	goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
      	goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
      	goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
      	goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
      	goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
      	goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
      	goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
      	int ind = 0;
      	goal.trajectory.points[ind].positions.resize(7);
      	goal.trajectory.points[ind].positions[0] = 0.0;
      	goal.trajectory.points[ind].positions[1] = 0.0;
      	goal.trajectory.points[ind].positions[2] = 0.0;
      	goal.trajectory.points[ind].positions[3] = 0.0;
      	goal.trajectory.points[ind].positions[4] = 0.0;
      	goal.trajectory.points[ind].positions[5] = 0.0;
      	goal.trajectory.points[ind].positions[6] = 0.0;
    // Velocities
      	goal.trajectory.points[ind].velocities.resize(7);
      	for (size_t j = 0; j < 7; ++j)
      	{
      		goal.trajectory.points[ind].velocities[j] = 0.0;
      	}
    // To be reached 1 second after starting along the trajectory
      	goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
      	ind += 1;
      	goal.trajectory.points[ind].positions.resize(7);
      	goal.trajectory.points[ind].positions[0] = -0.3;
      	goal.trajectory.points[ind].positions[1] = 0.2;
      	goal.trajectory.points[ind].positions[2] = -0.1;
      	goal.trajectory.points[ind].positions[3] = -1.2;
      	goal.trajectory.points[ind].positions[4] = 1.5;
      	goal.trajectory.points[ind].positions[5] = -0.3;
      	goal.trajectory.points[ind].positions[6] = 0.5;
    // Velocities
      	goal.trajectory.points[ind].velocities.resize(7);
      	for (size_t j = 0; j < 7; ++j)
      	{
      		goal.trajectory.points[ind].velocities[j] = 0.0;
      	}
    // To be reached 2 seconds after starting along the trajectory
      	goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
      	return goal;
      }

  //! Returns the current state of the action
      actionlib::SimpleClientGoalState getState()
      {
      	return traj_client_->getState();
      }

  };

  int main(int argc, char** argv)
  {
    // Init the ROS node
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    Parser p = Parser();
    p.parse_dictionary("");
    tf::TransformListener listener(nh);
    tf::StampedTransform transform;
    listener.waitForTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), transform);
    cout << transform.getOrigin().x();
    cout << transform.getOrigin().y();
    cout << transform.getOrigin().z();
    //RobotArm arm;
    // Start the trajectory
    //arm.startTrajectory(arm.armExtensionTrajectory());
    // Wait for trajectory completion
    //while(!arm.getState().isDone() && ros::ok())
    //{
    //	printf("Howdie");
    //}
  }