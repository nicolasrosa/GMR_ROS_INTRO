#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "publish_my_status_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Publisher pub_status = nh.advertise<diagnostic_msgs::DiagnosticStatus>("/my_status", 1);

    diagnostic_msgs::DiagnosticStatus status_msg;

    while(ros::ok()){
        status_msg.level = atoi(argv[1]);
        pub_status.publish(status_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}