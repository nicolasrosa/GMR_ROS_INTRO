#include <gmr_intro_2/gmr_intro_2.hpp>

RobotClass::RobotClass(ros::NodeHandle* nh)
{
    _nh = nh;
    _nh->param<std::string>("nome_topico_left_rpm", _params.topic_name_left_rpm, "/left_rpm");
    _nh->param<std::string>("nome_topico_right_rpm", _params.topic_name_right_rpm, "/right_rpm");
    _nh->param("/axle_track", _params.axle_track, 1.0);
    _nh->param("/gear_ratio", _params.gear_ratio, 1.0);
    _nh->param("/wheel_radius", _params.wheel_radius, 1.0);

    _sub_left = _nh->subscribe(_params.topic_name_left_rpm, 1, &RobotClass::subLeft, this);
    _sub_right = _nh->subscribe(_params.topic_name_right_rpm, 1, &RobotClass::subRight, this);

    _vel_m_s.left = 0.0;
    _vel_m_s.right = 0.0;    
}
void RobotClass::subLeft(const std_msgs::Float32::ConstPtr &msg)
{
    _vel_m_s.left = msg->data/_params.gear_ratio*_params.wheel_radius*2*M_PI/60.0;
    ROS_INFO_STREAM( "velocidade linear esquerda: " << _vel_m_s.left );
}
void RobotClass::subRight(const std_msgs::Float32::ConstPtr &msg)
{
    _vel_m_s.right  = msg->data/_params.gear_ratio*_params.wheel_radius*2*M_PI/60.0;
    ROS_INFO_STREAM( "velocidade linear direita: " << _vel_m_s.right );
}