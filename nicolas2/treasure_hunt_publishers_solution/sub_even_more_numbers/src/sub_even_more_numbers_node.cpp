/* ============ */
/*  Libraries   */
/* ============ */
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <string>

/* ============ */
/*  Functions   */
/* ============ */
void subCb(const std_msgs::Int16MultiArray::ConstPtr &msg){
    int msg_size = msg->data.size();
    double output[msg_size];
    
    std::cout<< "data_decoded: [";
    for(int i=0;i<msg_size;i++){
        output[i] = msg->data[i]/50;
        std::cout << output[i] << ", ";
    }
    std::cout<< "]\n\n";
}

/* ======= */
/*  Main   */
/* ======= */
int main(int argc, char* argv[]){
    ros::init(argc, argv, "sub_even_more_numbers_node");
    ros::NodeHandle nh;

    /* --- Topic Subscription --- */
    std::string topic_name = "/even_more_numbers";
    int queue_size = 10;
    ros::Subscriber sub_even_more_numbers = nh.subscribe(topic_name, queue_size, &subCb);

    ros::spin();
}