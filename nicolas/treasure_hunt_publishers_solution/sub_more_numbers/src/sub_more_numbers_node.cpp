/* ============ */
/*  Libraries   */
/* ============ */
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <control_msgs/JointTolerance.h>
#include <string>
#include <iostream>

/* ============ */
/*  Functions   */
/* ============ */
double position, velocity, acceleration;

void subCb_more_numbers(const std_msgs::Int16MultiArray::ConstPtr &msg){
    int msg_size = msg->data.size();
    double output[msg_size];

    std::cout<< "data_decoded: [";
    for(int i=0;i<msg_size;i++){
        output[i] = (msg->data[i]+velocity)/acceleration-position;
        // ROS_INFO_STREAM(msg->data[i] << " " <<output[i]);
        std::cout << output[i] << ", ";
    }
    std::cout<< "]\n\n";
}

void subCb_some_wrong_physics(const control_msgs::JointTolerance::ConstPtr &msg){
    // ROS_INFO_STREAM(msg->position);
    // ROS_INFO_STREAM(msg->velocity);
    // ROS_INFO_STREAM(msg->acceleration);

    position = msg->position;
    velocity = msg->velocity;
    acceleration = msg->acceleration;
}


/* ======= */
/*  Main   */
/* ======= */
int main(int argc, char* argv[]){
    ros::init(argc, argv, "sub_more_numbers_node");
    ros::NodeHandle nh;

    /* --- Topic Subscription --- */
    std::string topic_name = "/more_numbers";
    std::string topic_name2 = "/some_wrong_physics";
    int queue_size = 10;
    
    ros::Subscriber sub_more_numbers = nh.subscribe(topic_name, queue_size, &subCb_more_numbers);
    ros::Subscriber sub_some_wrong_physics = nh.subscribe(topic_name2, queue_size, &subCb_some_wrong_physics);
    
    ros::spin();
}