
#include "pr2_controller.h"





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
		ifstream inf("/nfs/attic/smartw/users/curranw/pr2_pca_experiment/data/manifolded_trajectory");
		string s;
		char c;

		//Read outer key
		while(true)
		{
			string outer_field;
			getline(inf, outer_field, '\'');
			getline(inf, outer_field, '\'');
			
      getline(inf, s, '[');
      getline(inf, s, ']');
      vector<double>* lst = parse_csv(s);

      traj_map->insert(pair<string, vector<double>* > (outer_field, lst ) );

			inf.get(c);
			cout << c;
			if (c == '}') break;
		}
    cout << traj_map->find("r_shoulder_lift_joint")->second->at(1);
	}

	vector<double>* parse_csv(string input)
	{
		stringstream ss(input);
		string s;
		vector<double>* output = new vector<double>();
		while(getline(ss,s,','))
		{
			output->push_back(strtod(s.c_str(), NULL));
		}
		return output;
	}

  void write_csv(string filename, vector<double> input)
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
};



//! Initialize the action client and wait for action server to come up
RobotArmLeft::RobotArmLeft() 
{
// tell the action client that we want to spin a thread by default
    record_endpoint = false;
	traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);

// wait for action server to come up
	while(!traj_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the joint_trajectory_action server");
	}
}

//! Clean up the action client
RobotArmLeft::~RobotArmLeft()
{
	delete traj_client_;
}

//! Sends the command to start a given trajectory
void RobotArmLeft::startTrajectory()
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
void RobotArmLeft::start_arm_extension_trajectory(int waypoints)
{
  /*
  goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
  goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
  goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
  goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
  goal.trajectory.joint_names.push_back("r_gripper_joint");
  goal.trajectory.joint_names.push_back("r_gripper_l_finger_joint");
  goal.trajectory.joint_names.push_back("r_gripper_r_finger_joint");
  goal.trajectory.joint_names.push_back("r_gripper_r_finger_tip_joint");
  goal.trajectory.joint_names.push_back("r_gripper_l_finger_tip_joint");
  goal.trajectory.joint_names.push_back("r_gripper_motor_screw_joint");
  goal.trajectory.joint_names.push_back("r_gripper_motor_slider_joint");
  */
  goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
  goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
  goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
  goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
  //goal.trajectory.joint_names.push_back("l_gripper_joint");
  /*
  goal.trajectory.joint_names.push_back("l_gripper_l_finger_joint");
  goal.trajectory.joint_names.push_back("l_gripper_r_finger_joint");
  goal.trajectory.joint_names.push_back("l_gripper_r_finger_tip_joint");
  goal.trajectory.joint_names.push_back("l_gripper_l_finger_tip_joint");
  goal.trajectory.joint_names.push_back("l_gripper_motor_screw_joint");
  goal.trajectory.joint_names.push_back("l_gripper_motor_slider_joint");
  */
  goal.trajectory.points.resize(waypoints);

}

void RobotArmLeft::add_arm_extension_trajectory(int cur_trajectory, map_string_2d_t* trajectory_map)
{
  static double duration = 0.01;
  int ind = cur_trajectory;
  goal.trajectory.points[ind].positions.resize(goal.trajectory.joint_names.size());
  goal.trajectory.points[ind].velocities.resize(goal.trajectory.joint_names.size());

  for(unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++)
  {
    string name =  goal.trajectory.joint_names[i];
    goal.trajectory.points[ind].positions[i] = trajectory_map->find(name)->second->at(ind);
    goal.trajectory.points[ind].velocities[i] = 0.0;
  }
  duration += 0.01;
  goal.trajectory.points[ind].time_from_start = ros::Duration(duration);
}
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
actionlib::SimpleClientGoalState RobotArmLeft::getState()
{
    if(record_endpoint)
    {
        listener.lookupTransform("/base_link", "/l_gripper_r_finger_link", ros::Time(0), transform);
        tf::Quaternion q = transform.getRotation();
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        endpoint_positions.push_back(x);
        endpoint_positions.push_back(y);
        endpoint_positions.push_back(z);
        endpoint_positions.push_back(roll);
        endpoint_positions.push_back(pitch);
        endpoint_positions.push_back(yaw);
    }
    return traj_client_->getState();
}

void RobotArmLeft::record_endpoint_positions()
{
  record_endpoint = true;
  listener.waitForTransform("/base_link", "/l_gripper_r_finger_link", ros::Time::now(), ros::Duration(3.0));
}



/*
int main(int argc, char** argv)
{
// Init the ROS node
ros::init(argc, argv, "robot_driver");
ros::NodeHandle nh;
Parser p = Parser();
p.parse_dictionary("");
//exit(1);
RobotArm arm;
//arm.record_endpoint_positions();
// Start the trajectory
arm.start_arm_extension_trajectory(p.traj_map->find("r_shoulder_lift_joint")->second->size());

for(int i = 0; i < p.traj_map->find("r_shoulder_lift_joint")->second->size(); i++)
{
  arm.add_arm_extension_trajectory(i, p.traj_map);
}
arm.startTrajectory();
// Wait for trajectory completion
int count = 0;
while(!arm.getState().isDone() && ros::ok())
{
    if(count++ % 10000 == 0)
	   printf("working\n");
}

//p.write_csv("endpoint_positions_orig.csv", arm.endpoint_positions);
}

*/