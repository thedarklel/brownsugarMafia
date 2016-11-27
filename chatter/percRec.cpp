#include "ros/ros.h"
#include "geomtry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

void percCallback(const geometry_msgs::PoseArray& points){ // The motion planning stuff goes in here
	int size = points.poses.size();
	for(int i=0;i<=size;i++){
		geometry_msgs::PoseArray targetPose;
		targetPose = points.poses[i];
		waypoints.push_back(targetPose);
        }
}
int main(int argc, char **argv){
	ros::init(argc,argv,"percRec");
	ros::NodeHandle n;
	ros::Subscriber sub - n.subscribe("percTopic",100, percCallback);
	ros::spin();
	return 0;
}

