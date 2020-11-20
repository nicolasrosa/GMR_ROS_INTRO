/* ============ */
/*  Libraries   */
/* ============ */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>

/* ============ */
/*  Functions   */
/* ============ */
/**
 * @brief Subscribler CallBack Function
 * @param std_msgs/Float32 messages from "/left_rpm" topic
 * std_msgs/Float32:
 *  float32 data
 */
void subCb(const std_msgs::Float32::ConstPtr &msg){
    ROS_INFO_STREAM(ros::this_node::getName() << " listening " << msg->data);
}

/* ======= */
/*  Main   */
/* ======= */
int main(int argc, char* argv[]){
    // ROS Node Initialization
    ros::init(argc, argv, "sub_rpm_node");
    //ros::NodeHandle nh; // Public
    ros::NodeHandle nh("~"); // Private

    /* --- Params Initialization --- */
    std::string topic_name = "/left_rpm";
    int queue_size = 10;

    // Using ros::param::get() (Recommended)
    /* Better readability, allows to set default values. */
    /* It is preferable due to the greater control and ease of reading the code, especially when there are a greater number of parameters. */
    nh.param<std::string>("topic_name", topic_name, "/left_rpm");
    nh.param("queue_size", queue_size, 10);

    // Using ros::NodeHandle::getParam()
    //nh.getParam("topic_name", topic_name);
    //nh.getParam("queue_size", queue_size);

    /* --- Topic Subscription --- */
    ROS_INFO_STREAM("topic_name: " << topic_name << "queue_size: " << queue_size);
    ros::Subscriber sub_rpm = nh.subscribe(topic_name, queue_size, subCb);

    ros::spin();
}