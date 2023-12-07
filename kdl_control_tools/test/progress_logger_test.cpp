#include<ros/ros.h>

#include<kdl_control_tools/progress_logger.h>

using namespace std;

kdl_control_tools::ProgressLogger p_logger_;

int main(int argc, char **argv){

  ros::init(argc, argv, "progress_logger_test");
  ROS_INFO("ProgressLogger test node");

  ros::NodeHandle nh;

  double d = 0;
  while(ros::ok() && d < 20.0){

    p_logger_.log("test", d);

    d += 1.0;
    ros::spinOnce();
  }

  p_logger_.writeVarToFile("test", "");

  return 0;

}
