//include ros header file
#include "ros/ros.h" 
//header file of the custom message
//  This message belongs to the package in which it is defined
#include "sine_pkg/my_msg.h"

#include <iostream>
#include "math.h"
#define pi 3.14159265
 
using namespace std;


int main(int argc, char **argv) {

	//Initialize the ROS node with name: ros_topic_publisher
	ros::init(argc, argv,"sine_publisher");
	
	//Declare the node handle: our interface with the ROS system
	ros::NodeHandle nh;

	//Create a publisher object:
	//	Input:  - type of message: std_msgs::Int32
	//			- topic name: /numbers
	//			- message queue: 10 (0: infinite queue)
	ros::Publisher topic_pub = nh.advertise<sine_pkg::my_msg>("/sine", 1);

	//Rate object: 10 Hz of rate
	ros::Rate rate(10); 

    //Define the custom datatype
    sine_pkg::my_msg sine;

    //insert desired amplitude and frequency
    cout<<"insert the amplitude \n";
    cin>>sine.amplitude;
    cout<<"insert the frequency in Hz\n";
    cin>>sine.frequency;

    float Ts = 0.0001;

    int i = 0;

	// Typical loop: neverending loop: a controller works until actuators are activated
	//		while (ros::ok()): works until the ROS node is not terminated (by the user with ctrl+c or similar)
	while ( ros::ok() ) {

        //Fill the data part
        sine.value = sine.amplitude*sin(2*pi*sine.frequency*i*Ts);
        i++;

		//ROS_INFO: Like a printf, but with the timestamp
		ROS_INFO("%f",sine.value); 

		//Publish the message over the ROS network
		topic_pub.publish(sine);
		
		//Rate to maintain the 10 Hz
		rate.sleep();
	}
	
	return 0;
}