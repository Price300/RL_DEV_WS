#include "ros/ros.h"
#include <iostream>
//Header of the service message. 
//	The service message belongs to the ros_service package
#include "rl_kuka/pose.h"
#include <tf/transform_listener.h>

using namespace std;

//Callback function
//	Return value: boolean
//		If this function returns true, the service function has been corretly called
//		You can use this value to check if the function has been called with correct parameters
//		i.e. call a service calculating the square of a number, calling the service with a negative number
//	Input values:  the request part of the servive 
//				   the output of the service to fill
bool service_callback( rl_kuka::pose::Request &req, rl_kuka::pose::Response &res) {


	cout << "Service frames received: " << endl;
	//We know that the service is called with 2 parameters: frame_a and frame_b
	// These parameters are put in a data structure called req
	cout << req.frame_a << req.frame_b << endl;

	tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
		listener.waitForTransform(req.frame_a.data, req.frame_b.data, ros::Time(0), ros::Duration(3.0));
    	listener.lookupTransform(req.frame_a.data, req.frame_b.data, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
    	ROS_ERROR("%s",ex.what());
    	ros::Duration(1.0).sleep();
    }

    ROS_INFO_STREAM(" Transform: " << 
        
        transform.getOrigin().x() << ", " << 
        transform.getOrigin().y() << ", " <<
        transform.getOrigin().z() << ", " << 
        transform.getRotation().x() << ", " << 
        transform.getRotation().y() << ", " << 
        transform.getRotation().z()
    );	

	//The return value is store in the res datastrcutre
	cout << "Returning the pose" << endl;
	res.pose.position.x=transform.getOrigin().x();
	res.pose.position.y=transform.getOrigin().y();
	res.pose.position.z=transform.getOrigin().z();
	res.pose.orientation.x=transform.getRotation().x();
	res.pose.orientation.y=transform.getRotation().y();
	res.pose.orientation.z=transform.getRotation().z();
	res.pose.orientation.w=transform.getRotation().w();

	return true;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "kuka_service");
	ros::NodeHandle n;

	//Initialize the service object: name of the service and callback function
	//	Like subscribers, also tje callback function can be declared as a class function
	ros::ServiceServer service = n.advertiseService("pose", service_callback);

	//Call the spin function to maintain the node alive
	ros::spin();

	return 0;
}
