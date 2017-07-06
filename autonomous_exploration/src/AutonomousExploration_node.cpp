// Creates a ros node for autonomous exploration

#include <ros/ros.h>
#include <autonomous_exploration/AutonomousExploration.h>

using namespace AutonomousExploration_server;

int main(int argc, char** argv)
{
	// Start the AutonomousExploration_server node
    ros::init(argc, argv, "AutonomousExploration_server");

    AutonomousExploration autonomous_exploration;
    
    ros::spin();
    return 0;
}