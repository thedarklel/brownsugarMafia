#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

void percCallback(const geometry_msgs::PoseArray& points){
	int size = points.poses.size();
        std::vector<geometry_msgs::Pose> waypoints;
	waypoints.clear();
	for(int i=0;i<=size;i++){
		geometry_msgs::Pose targetPose;
		targetPose = points.poses[i];
		waypoints.push_back(targetPose);
        }
	ROS_INFO("Received msg size : %i",waypoints.size());
}
int main(int argc, char **argv){
	ros::init(argc,argv,"percRec");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("percTopic",10, percCallback);
	ros::spin();
	return 0;
}

