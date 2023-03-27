#include "ros/ros.h"
#include "sine_pkg/my_msg.h"
#include "std_msgs/Float32.h"
#include "boost/thread.hpp"
#include <iostream>

using namespace std;

//Use a class to store the topic data 
//	Just a choice: the best way to share data between a main loop and the ROS callbacks
class ROS_SUB {
	public:
		ROS_SUB();
		void topic_cb( sine_pkg::my_msgConstPtr data);
        void low_filter_thread();
	
	private:
		ros::NodeHandle _nh;
		//Subscriber object
		ros::Subscriber _topic_sub;
        //Publisher object
        ros::Publisher _topic_pub;
        //filtered variables
        float f_value;
        float f_frequency; 
};

ROS_SUB::ROS_SUB() {
	//Initialize a subscriber:
	//	Input: 	topic name: /numbers
	//			queue:	1
	//			Callback function
	//			Object context: the value of data members
	_topic_sub = _nh.subscribe("/sine", 1, &ROS_SUB::topic_cb, this);
    boost::thread filter_thread( &ROS_SUB::low_filter_thread, this);
}

//Callback function: the input of the function is the data to read
//	In this function, a smart pointer is used
void ROS_SUB::topic_cb( sine_pkg::my_msgConstPtr d) {

	//d is a pointer of sine_pkg::my_msg type
	//	to access to its field, the "." can not be used
	ROS_INFO("Listener value: %f", d->value);
    ROS_INFO("Listener amplitude: %f", d->amplitude);
    ROS_INFO("Listener frequency: %f", d->frequency);
    f_value = d->value;
    f_frequency = d->frequency;

}

//Low-pass filter: the input is di sine signal published and
// the output is the sine filtered by a gain chosen on frequency
void ROS_SUB::low_filter_thread(){
    _topic_pub = _nh.advertise<std_msgs::Float32>("/sine_filtered", 1);
    ros::Rate rate(10);
    float cut_freq = 50;
    while(ros::ok()){
        std_msgs::Float32 msg; 
        float gain = 1-(f_frequency - cut_freq)/100;
        if(f_frequency >= 100+cut_freq) gain = 0;
        if(f_frequency <= cut_freq) gain = 1;
        cout<<"[ INFO] [1679676594.404039906]: GAIN: "<<gain<<endl; //debug
        msg.data = gain*f_value;
        cout<<endl<<endl;
        ROS_INFO("Publish filtered sine value: %f",msg.data);
        _topic_pub.publish(msg); 
        rate.sleep();
    }
}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "subscriber");

	//Create the ROS_SUB class object
	ROS_SUB rs;
	
	//ros::spin() blocks the main thread from exiting until ROS invokes a shutdown - via a Ctrl + C for example
	// It is written as the last line of code of the main thread of the program.
	//Also the spin invokes the callbacks to flush their queue and process incoming data
	ros::spin(); 

	//----This function will be never overcome

	return 0;
}