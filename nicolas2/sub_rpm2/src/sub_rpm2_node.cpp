/* ============ */
/*  Libraries   */
/* ============ */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iterator>
#include <random>
#include <string>

/* ========= */
/*  Classes  */
/* ========= */
class SubscriberAndPublisher{
public:
    // Constructor
    SubscriberAndPublisher(){
        ROS_INFO("Hello World2!");
        // Create Subscriber
        std::string topic_name = "/left_rpm";
        int queue_size = 10;
        sub_ = nh_.subscribe(topic_name, queue_size, &SubscriberAndPublisher::subCb, this);
        
        // Create Publisher
        pub_ = nh_.advertise<std_msgs::Float32> ("/left_rpm2", 1);
    }
        
    // Subscriber CallBack
    void subCb(const std_msgs::Float32::ConstPtr &msg){
        std_msgs::Float32 msg2;
        msg2.data = msg->data*2;

        ROS_INFO("%f %f", msg->data, msg2.data);
        pub_.publish(msg2);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;        
};

/* ======= */
/*  Main   */
/* ======= */
int main(int argc, char **argv){
    // ROS Node Initialization
    ros::init(argc, argv, "sub_rpm2_node");

//    ROS_INFO("Hello World!");

//    while(ros::ok())

    //Create an object of class SubscribeAndPublish that constains the Subscriber and the Publisher declarations
    SubscriberAndPublisher SABObj;

    ros::spin();
}