#include "ros/ros.h"

//Header of the service message. 
//	The service message belongs to the ros_service package
#include "rl_kuka/pose.h"
#include <tf/transform_broadcaster.h>

using namespace std;

int main(int argc, char **argv) {

	//Init the ROS node with service_client name
	ros::init(argc, argv, "kuka_client");
	string frame_a = argv[1];
	string frame_b = argv[2];
	ros::NodeHandle n;

	tf::TransformBroadcaster br;
    tf::Transform transform;

	ros::Rate r(100);
	//while( ros::ok() ){  uncomment if you want a broadcast in real time (every prose)

		//Init the service client. Data to use for the service (the .srv file) and the name of the service
		ros::ServiceClient client = n.serviceClient<rl_kuka::pose>("pose");
		
		//Define and initialize the service data structure 
		//	This datastructure brings with it the input value (in the request fields) and the output values, in the response field
		rl_kuka::pose srv;
		srv.request.frame_a.data = frame_a;
		srv.request.frame_b.data = frame_b;
		
		//Wait that in the ROS network, the service pose is advertised
		//	If you call a service and the service has not been advertised, you will have back an error
		ROS_INFO("Waiting for the client server");
		client.waitForExistence();
		ROS_INFO("Client server up now");
		
		//Call the service callback
		//	The return value is false if:
		//		- the callback returns false
		//		- the service has not been found in the ROS network
		if (!client.call(srv)) {
			ROS_ERROR("Error calling the service");
			return 1;
		}

		//Just print the output
		cout << "Service output position x: " << srv.response.pose.position.x << endl;
		cout << "Service output position y: " << srv.response.pose.position.y << endl;
		cout << "Service output position z: " << srv.response.pose.position.z << endl;
		cout << "Service output orientation x: " << srv.response.pose.orientation.x << endl;
		cout << "Service output orientation y: " << srv.response.pose.orientation.y << endl;
		cout << "Service output orientation z: " << srv.response.pose.orientation.z << endl;
		cout << "Service output orientation w: " << srv.response.pose.orientation.w << endl;
		
	while( ros::ok() ){ // boroadcast of one pose (sencond frame_b stuck on last pose detected)
        transform.setOrigin( 
			tf::Vector3(
				srv.response.pose.position.x, 
				srv.response.pose.position.y, 
				srv.response.pose.position.z
			) 
		);
        tf::Quaternion q;
	 	q.setX(srv.response.pose.orientation.x);
		q.setY(srv.response.pose.orientation.y);
		q.setZ(srv.response.pose.orientation.z);
		q.setW(srv.response.pose.orientation.w);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_a, frame_b));

        r.sleep();
    }

	return 0;
}
