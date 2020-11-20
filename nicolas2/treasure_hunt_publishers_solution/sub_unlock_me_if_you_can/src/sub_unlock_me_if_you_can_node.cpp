#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <string.h>

void subCb(const std_msgs::Int32MultiArray::ConstPtr &msg){
    int msg_size = msg->data.size();
    double output[msg_size];

    std::cout<< "data_decoded: [";
    for(int i;i<msg_size;i++){
        output[i] = msg->data[i]-1993;
        std::cout << output[i] << ", ";
    }
    std::cout<< "]\n\n";
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "sub_unlock_me_if_you_can_node");
    ros::NodeHandle nh;

    std::string topic_name = "/unlock_me_if_you_can";
    int queue_size = 10;

    ros::Subscriber sub_unlock_me_if_you_can = nh.subscribe(topic_name, queue_size, subCb);

    ros::spin();
}