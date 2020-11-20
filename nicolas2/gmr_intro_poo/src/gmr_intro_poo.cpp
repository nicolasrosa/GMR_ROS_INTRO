/* =========== */
/*  Libraries  */
/* =========== */
#include "../include/gmr_intro_poo/gmr_intro_poo.hpp"

/* ============================ */
/*  RobotClass, Implementation  */
/* ============================ */
RobotClass::RobotClass(ros::NodeHandle *nh){
    _nh = nh;
    
    // Params Initialization
    _nh->param("time_between_toggles", _params.time_between_toggles, -1.0);
    _nh->param("/axle_track", _params.axle_track, 1.0);
    _nh->param("/gear_ratio", _params.gear_ratio, 1.0);
    _nh->param("/wheel_radius", _params.wheel_radius, 1.0);
    _nh->param<std::string>("topic_name_left_rpm", _params.topic_name_left_rpm, "/left_rpm");  //_nh->param("topic_name_left_rpm", _params.topic_name_left_rpm, std::string("/left_rpm"));
    _nh->param<std::string>("topic_name_right_rpm", _params.topic_name_right_rpm, "/right_rpm");  //_nh->param("topic_name_right_rpm", _params.topic_name_left_rpm, std::string("/left_rpm"));
    
    // Subscribers Initialization
    _sub_left = _nh->subscribe(_params.topic_name_left_rpm, 1, &RobotClass::subLeft, this);
    _sub_right = _nh->subscribe(_params.topic_name_right_rpm, 1, &RobotClass::subRight, this);
    
    // Publishers Initialization
    _pub_odom = _nh->advertise<nav_msgs::Odometry>("/odom",1);
    _prev_timestamp = ros::Time::now();

    // Services Initialization
    _client_toggle_robot = _nh->serviceClient<std_srvs::Trigger>("/toggle_robot");
    _prev_timestamp_toggle = _prev_timestamp;

    // Robot Initialization
    _robot_pose = (const struct RobotPose){0};
    _vel_m_s.left = 0.0;
    _vel_m_s.right = 0.0;
}

void RobotClass::subLeft(const std_msgs::Float32::ConstPtr &msg){
    _vel_m_s.left = msg->data/_params.gear_ratio*_params.wheel_radius*2*M_PI/60.0;
    //ROS_INFO_STREAM( "velocidade linear esquerda: " << _vel_m_s.left );
    //ROS_INFO_STREAM("motor_left, angular speed (RPM): " << msg->data << "\t" << "motor_left, linear velocity (m/s): " << _vel_m_s.left);
    ROS_INFO_STREAM("motor_left,  ang_speed (RPM): " << msg->data << "\t" << "motor_left,  lin_vel (m/s): " << _vel_m_s.left);

}

void RobotClass::subRight(const std_msgs::Float32::ConstPtr &msg){
    _vel_m_s.right = msg->data/_params.gear_ratio*_params.wheel_radius*2*M_PI/60.0;
    //ROS_INFO_STREAM( "velocidade linear direita: " << _vel_m_s.right );
    //ROS_INFO_STREAM("motor_right, angular speed (RPM): " << msg->data << "\t" << "motor_right, lin velocity (m/s): " << _vel_m_s.left);
    ROS_INFO_STREAM("motor_right, ang_speed (RPM): " << msg->data << "\t" << "motor_right, lin_vel (m/s): " << _vel_m_s.left);
}

void RobotClass::checkToggleRobot(){
    ros::Time cur_timestamp = ros::Time::now();
    std_srvs::Trigger srv;
    
    if(_params.time_between_toggles > 0 && cur_timestamp.toSec() - _prev_timestamp_toggle.toSec() > _params.time_between_toggles){
        _prev_timestamp_toggle = cur_timestamp;
        if(_client_toggle_robot.call(srv)){
            ROS_WARN_STREAM("Message: " << srv.response.message);
        }else{
            ROS_ERROR("Failed to call the service!");
        }
    }
}

/*
nav_msgs/Odometry:
    std_msgs/Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
*/
void RobotClass::calculateOdom(){
    ros::Time cur_timestamp = ros::Time::now();
    double vl = _vel_m_s.left;
    double vr = _vel_m_s.right;
    double dt, vx, wz;
    nav_msgs::Odometry odom;
    tf2::Quaternion odom_quat;

    // Cinematic equations for a differential traction robot on unicycle model representation
    dt = cur_timestamp.toSec() - _prev_timestamp.toSec();
    _prev_timestamp = cur_timestamp;
    vx = 0.5*(vr+vl);                                   // Eq. 1, vx = (vr+vl)/2
    wz = (vr-vl)/_params.axle_track;                    // Eq. 2, wz = (vr-vl)/L
    _robot_pose.theta += wz*dt;                         // Eq. 8, θ(t) = θ(t-1) + wz*dt
    _robot_pose.x += vx*std::cos(_robot_pose.theta)*dt;    // Eq. 6, x(t) = x(t-1) + vx*cosθ
    _robot_pose.y += vx*std::sin(_robot_pose.theta)*dt;    // Eq. 7, y(t) = y(t-1) + vx*sinθ

    // ----- Odometry Quaternion ----- //
    //Set the quaternion using fixed axis RPY (Roll, Pitch, Yaw).
    odom_quat.setRPY(0, 0, _robot_pose.theta);

    // ----- Odometry Message ----- //
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "/map";
    odom.child_frame_id = "/base_link";
    
    // geometry_msgs/Twist: This expresses velocity in free space broken into its linear and angular parts.
    odom.twist.twist.angular.z = wz;
    odom.twist.twist.linear.x = vx;
    
    // geometry_msgs/Pose: A representation of pose in free space, composed of position and orientation. 
    odom.pose.pose.position.x = _robot_pose.x;
    odom.pose.pose.position.y = _robot_pose.y;
    
    // geometry_msgs/Quaternion: This represents an orientation in free space in quaternion form.
    odom.pose.pose.orientation = tf2::toMsg(odom_quat); // tf2::Quaternion -> geometry_msgs::Quaternion
    
    // ----- Odometry transform ----- //
    // First, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "/map";
    odom_trans.child_frame_id = "/base_link";
    odom_trans.header.stamp = cur_timestamp;
    
    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.rotation = tf2::toMsg(odom_quat);

    // Send the transform and Publish nav_msgs/Odometry message
    _odom_broadcaster.sendTransform(odom_trans);
    _pub_odom.publish(odom);
}