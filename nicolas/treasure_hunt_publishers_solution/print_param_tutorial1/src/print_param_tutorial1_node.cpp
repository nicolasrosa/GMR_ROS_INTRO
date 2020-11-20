#include <ros/ros.h>
#include <string>

int main(int argc, char* argv[]){
    ros::init(argc,argv,"print_param_tutorial1_node");
    ros::NodeHandle nh;
    
    std::string param_tutorial1;
    nh.getParam("/tutorial1", param_tutorial1);

    ROS_INFO_STREAM(param_tutorial1);

    ros::spinOnce();
}