#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>

class RobotClass
{
    public:
        RobotClass(ros::NodeHandle* nh);

    private:
        void                subLeft(const std_msgs::Float32::ConstPtr &msg);
        void                subRight(const std_msgs::Float32::ConstPtr &msg);

        ros::NodeHandle*    _nh;
        ros::Subscriber     _sub_left;
        ros::Subscriber     _sub_right;

        struct Params
        {
            double          axle_track;
            double          gear_ratio;
            double          wheel_radius;
            std::string     topic_name_left_rpm;
            std::string     topic_name_right_rpm;
        }_params;

        struct Vel_m_s
        {
            double          left;
            double          right;
        }_vel_m_s;
};