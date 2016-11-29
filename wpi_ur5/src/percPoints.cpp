#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

int main(int argc, char **argv)
{
	int nop = 5;
	ros::init(argc, argv, "percPoints");
	ros::NodeHandle n;
	ros::Publisher percTopic_pub = n.advertise<geometry_msgs::PoseArray>("percTopic",1);
	geometry_msgs::PoseArray poseArray;
	poseArray.poses.clear(); //Clear last block perception result
	poseArray.header.stamp = ros::Time::now();
	poseArray.header.frame_id = "world";
	for(float i=0;i<=nop;i++){
		geometry_msgs::Pose pose;	
		pose.position.x = i+0.2;
		pose.position.y = i+0.2;
		pose.position.z = i+0.2;
		poseArray.poses.push_back(pose);
	}
	percTopic_pub.publish(poseArray);
        ROS_INFO("Perception points sent");
	//ROS_INFO("poseArray size: %i",poseArray.poses.size()); //No. of points from perception block
	ros::spinOnce();	
	return 0;
}

///****ISSUES***///
// Publishing just once does not get recognised neither by the subscriber nor by the rostopic echo /percTopic
// But looping the messages works fine
