#include "ros/ros.h"
#include "std_msgs/String.h"
int main(int argc, char** argv)
{
	ros::init(argc, argv, "talker");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate looprate(10);
	int count = 0;
	while(ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << "dumb" << count;
		msg.data = ss.str();
		ros::spinOnce();
		ROS_INFO("I talk u r %s", msg.data.c_str());
		pub.publish(msg);
		++count;
		looprate.sleep();
	}
	return 0;
}