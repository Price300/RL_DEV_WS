#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <fstream>

//Include KDL libraries
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#define pi 3.14159265

using namespace std;


class KUKA_INVKIN {
	public:
		KUKA_INVKIN();

		//From the run method we will start all the threads needed to accomplish the task
		void run();
		//Function to load the model from the URDF file (parameter server)
		bool init_robot_model();
		//Function to retrieve the pose of the end-effector using KDL
		void get_dirkin();
		//Callback for the /joint_state message to retrieve the value of the joints
		void joint_states_cb( sensor_msgs::JointState );
		//Joint space positioning to set an initial position
		void goto_initial_position( float dp[7] );
		//Main control loop function
		void ctrl_loop();

		ifstream pos,ori;
		ofstream jt;
		
	private:
		ros::NodeHandle _nh;

		//Kinematic chain
		KDL::Chain _k_chain;
		//Kinematic tree
		KDL::Tree iiwa_tree;
	
		//Forward kinematics solver
		KDL::ChainFkSolverPos_recursive *_fksolver; 	
		//Implementation of a inverse velocity kinematics algorithm based on 
		//the generalize pseudo inverse to calculate the velocity transformation 
		//from Cartesian to joint space of a general KDL::Chain
		KDL::ChainIkSolverVel_pinv *_ik_solver_vel;   	//Inverse velocity solver
		
		//Implementation of a general inverse position kinematics algorithm based on Newton-Raphson 
		//iterations to calculate the position transformation from Cartesian 
		//to joint space of a general KDL::Chain. 
		KDL::ChainIkSolverPos_NR *_ik_solver_pos;

		ros::Subscriber _js_sub;
		ros::Publisher _cartpose_pub;
		ros::Publisher _cmd_pub[7];
	
		//Variable to store the joint configuration
		KDL::JntArray *_q_in;
		//Variable to store the end effector pose
		KDL::Frame _p_out;

		//Control flags to check 
		//that data have been received
		bool _first_js;
		bool _first_fk;
	

};

KUKA_INVKIN::KUKA_INVKIN() {

	//If the robot motdel is not correctly loaded, exit from the program
	if (!init_robot_model()) 
		exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");

	//Get some output from the kinemtic object: number of joints and links
	cout << "Joints and segments: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;
 
	//Input: the current configuration of the robot (its joints value)
	_js_sub = _nh.subscribe("/lbr_iiwa/joint_states", 0, &KUKA_INVKIN::joint_states_cb, this);
	
	//Output: the cartesian position of the end-effector
	_cartpose_pub = _nh.advertise<geometry_msgs::Pose>("/lbr_iiwa/eef_pose", 0);
	//Output: the command to the robot joints
	_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint1_position_controller/command", 1);
	_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint2_position_controller/command", 1);
	_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint3_position_controller/command", 1);
	_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint4_position_controller/command", 1);
	_cmd_pub[4] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint5_position_controller/command", 1);
	_cmd_pub[5] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint6_position_controller/command", 1);
	_cmd_pub[6] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint7_position_controller/command", 1);

	//Set the control flags to false
	_first_js = false;
	_first_fk = false;

	pos.open("pos2.txt");
	ori.open("ori2.txt");
	if(!pos.is_open()||!ori.is_open()) cout<<"files not open \n";
	jt.open("joint_state.txt");
}

bool KUKA_INVKIN::init_robot_model() {

	//Retrieve the robot description (URDF) from the robot_description param
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	
	//Use the treeFromString function to convert the robot model into a kinematic tree 
	if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	//Define the links of the desired chain base_link -> tip_link
	std::string base_link = "lbr_iiwa_link_0";
	std::string tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;

	//Initialize the solvers
	//Solvers are declared as pointer in the class definition
	//Here we instantiate the solvers on the desired kinematic chain
	_fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain );
	_ik_solver_vel = new KDL::ChainIkSolverVel_pinv( _k_chain );
	//The invers kinematic solver object needs must be initialized considering also
	//the number of iterations to solve the ik problem on a given robot configuration
	//and the allowed error on the joint positioning 
	_ik_solver_pos = new KDL::ChainIkSolverPos_NR( _k_chain, *_fksolver, *_ik_solver_vel, 100, 1e-6 );

	//The _q_in is the vetor where the values of the joints at a current time
	//are saved. Also this variable is a pointer
	//Must be instantiated before its usage 
	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );


	return true;
}

//Callback for the joint state
void KUKA_INVKIN::joint_states_cb( sensor_msgs::JointState js ) {

	//We assume to know the number of joints
	for(int i=0; i<7; i++ ) 
		_q_in->data[i] = js.position[i];

	//We can start the calculation of the fw kinematic
	//We know that the first joint value set is arrived
	_first_js = true;
}

//Initial robot positioning
//Command the initial velue until that value has not been reached
//	If the robot is controlled in position, the inner loop of the acutator 
//	will assure that the value will be reached in a smooth way
void KUKA_INVKIN::goto_initial_position( float dp[7] ) {
	
	ros::Rate r(10);

	float min_e = 1000.0;
	float max_e = 1000.0;

	std_msgs::Float64 cmd[7];

	//While the maximum error over all the joints is higher than a given threshold 
	while( max_e > 0.002 ) {
 		max_e = -1000;
		//Command the same value for all the joints and calculate the maximum error
		for(int i=0; i<7; i++) {
 			cmd[i].data = dp[i];
			_cmd_pub[i].publish (cmd[i]);
			float e = fabs( cmd[i].data - _q_in->data[i] );
			//max_e is the maximum error over all the joints
			max_e = ( e > max_e ) ? e : max_e;
		}
		r.sleep();
	}

	sleep(2);
}

void KUKA_INVKIN::get_dirkin() {

	ros::Rate r(50);

	KDL::JntArray q_curr(_k_chain.getNrOfJoints());

	//Wait the first Joint state message
	//	Without the first joint value
	//	is not useful to calculate the Fk
	while( !_first_js ) usleep(0.1);

	//Output message to publish the pose of the end effector
	geometry_msgs::Pose cpose;

	while(ros::ok()) {

		//JntToCart: Joint values to Cartesian space
		//	First argument: the current value of the robot joints
		//	Second argument: the calculated pose of the end effector
		_fksolver->JntToCart(*_q_in, _p_out);


		double qx, qy, qz, qw;

		cpose.position.x = _p_out.p.x();
		cpose.position.y = _p_out.p.y();
		cpose.position.z = _p_out.p.z();

		//In KDL the p_out is a KDL::Frame data
		//	The orientation is reported in the rotation matrix form
		//	We can convert into quaternion
		_p_out.M.GetQuaternion( qx, qy, qz, qw);
		cpose.orientation.w = qw;
		cpose.orientation.x = qx;
		cpose.orientation.y = qy;
		cpose.orientation.x = qz;

		_cartpose_pub.publish( cpose );		
	
		//Flag to true for the fk calculation (we can proceed with the ik solution)
		_first_fk = true;
	
		r.sleep();
	}
}

void KUKA_INVKIN::ctrl_loop() {

	//Wait until the first fk has not been calculated
	while( !_first_fk ) usleep(0.1);


	//Control the robot towards a fixed initial position
	float i_cmd[7];
	i_cmd[0] = 0.0;
	i_cmd[1] = 0.0;
	i_cmd[2] = 0.0;
	i_cmd[3] = -pi/2;
	i_cmd[4] = 0.0;
	i_cmd[5] = pi/2;
	i_cmd[6] = 0.0;
	goto_initial_position( i_cmd );

	//F_dest is the target frame: where we want to bring the robot end effector 
	KDL::Frame F_dest;

	//q_out is the variable storing the output of the Inverse kinematic
	KDL::JntArray q_out(_k_chain.getNrOfJoints());

	//Lock the code to start manually the execution of the trajectory
	cout << "Press enter to start the trajectory execution" << endl;
	string ln;
	getline(cin, ln);			
	while(!pos.eof()){
		//Generate the goal position

		float p_x,p_y,p_z;

		pos>>p_x;
		pos>>p_y;
		pos>>p_z; 

		F_dest.p.x(p_x);
		F_dest.p.y(p_y);
		F_dest.p.z(p_z);

		double q0,q1,q2,q3;

		ori>>q1;
		ori>>q2;
		ori>>q3;
		ori>>q0;

		F_dest.M.Quaternion(q1,q2,q3,q0);	
		
		std_msgs::Float64 cmd[7];

		//CartToJnt: transform the desired cartesian position into joint values
		if( _ik_solver_pos->CartToJnt(*_q_in, F_dest, q_out) != KDL::SolverI::E_NOERROR ) 
			cout << "failing in ik!" << endl;

		//Convert KDL output values into std_msgs::Float datatype
		for(int i=0; i<7; i++) {
			cmd[i].data = q_out.data[i];
			jt<<cmd[i].data<<endl;
		}

		//Publish all the commands in topics
		for(int i=0; i<7; i++) {
			_cmd_pub[i].publish (cmd[i]);
		}
		usleep(1e5);
	}
}

void KUKA_INVKIN::run() {

	//In the run method, we start the threads to:
	//	- Calculate the forward kinematic
	//	- Calculate the inverse kinematic 
	boost::thread get_dirkin_t( &KUKA_INVKIN::get_dirkin, this);
	boost::thread ctrl_loop_t ( &KUKA_INVKIN::ctrl_loop, this);
	ros::spin();	

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "iiwa_kdl");
	KUKA_INVKIN ik;
	ik.run();

	ik.pos.close();
	ik.ori.close();
	ik.jt.close();

	return 0;
}
