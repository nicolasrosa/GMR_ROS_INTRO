/* ============ */
/*  Libraries   */
/* ============ */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iterator>
#include <random>
#include <std_srvs/Trigger.h>

bool toggle_robot = true;

/* ============ */
/*  Functions   */
/* ============ */
bool toggleRobot(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  toggle_robot = !toggle_robot;
  //ROS_WARN_STREAM("Toogling Robot...");
  response.success = true;
  response.message = "Someone toggled me...";
  return true;
}

/* ======= */
/*  Main   */
/* ======= */
int main(int argc, char **argv)
{
  // Start ROS within the context of this node.
  ros::init(argc, argv, "gmr_intro_ds4_node");
  
  // Declare Node
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(50);  // Hertz (Hz)

  // Create Topics
  /* If we change to "/<side>_name", attention to the "/", different nodes will publish to the same node! */
  // ros::Publisher pub_left_rpm = nh.advertise<std_msgs::Float32> ("/left_rpm",1);
  // ros::Publisher pub_right_rpm = nh.advertise<std_msgs::Float32> ("/right_rpm",1);
  
  // Create Service Server
  ros::ServiceServer service = nh.advertiseService("/toggle_robot", toggleRobot);

  // Create Params
  double axle_track, wheel_radius, gear_ratio, rpm_ref, gaussian_noise_mean, gaussian_noise_stddev;
  nh.param("/axle_track", axle_track, 0.5);                         // Meters
  nh.param("/gaussian_noise_mean", gaussian_noise_mean, 0.0);       // RPM
  nh.param("/gaussian_noise_stddev", gaussian_noise_stddev, 1.0);   // RPM
  nh.param("/gear_ratio", gear_ratio, 20.0);                        // Ratio
  nh.param("/rpm_ref", rpm_ref, 15.0);                              // RPM
  nh.param("/wheel_radius", wheel_radius, 0.05);                    // Meters

  std_msgs::Float32 wl, wr;
  bool toggle_curve = false;

  //ros::ServiceClient client = nh.serviceClient<diagnostics_msgs::AddDiagnostics>("/toggle_the_robot");
  
  // Define random generator with Gaussian distribution
  std::default_random_engine generator_l, generator_r;
  std::normal_distribution<double> dist(gaussian_noise_mean, gaussian_noise_stddev);

  ros::Time current_time, first_time = ros::Time::now(), initial_curve_time = first_time, initial_straight_time = first_time; 
  ros::Duration(0.02).sleep();

  double vl, vr, wz, t_curve;
  vl = -rpm_ref*wheel_radius*2*M_PI/60.0;
  vr = -vl;
  wz = (vr-vl)/axle_track;
  t_curve = 0.5*M_PI/wz;

  while(ros::ok())
  {
    nh.setParam("/axle_track", axle_track);
    nh.setParam("/gear_ratio", gear_ratio);
    nh.setParam("/wheel_radius", wheel_radius);
    current_time = ros::Time::now();
    if(current_time.toSec() - initial_curve_time.toSec() > t_curve && toggle_curve)
    {     
      toggle_curve = !toggle_curve;
      initial_straight_time = current_time;
      ROS_INFO_STREAM("Toggle");
    }
    else if(current_time.toSec() - initial_straight_time.toSec() > 5.0 && !toggle_curve )
    {
      toggle_curve = !toggle_curve;
      initial_curve_time = current_time;
      ROS_INFO_STREAM("Toggled!");

    }

    if(toggle_robot)
    {
      if(toggle_curve)
      {
        wl.data = - (rpm_ref + dist(generator_l)) * gear_ratio;
        wr.data =   (rpm_ref + dist(generator_r)) * gear_ratio;
        ROS_INFO_STREAM("Now on State 1.");
      }
      else
      {
        wl.data = (rpm_ref/4 + dist(generator_l)) * gear_ratio;
        wr.data = (rpm_ref/4 + dist(generator_r)) * gear_ratio;
        ROS_INFO_STREAM("Now on State 2.");
      }
    }
    else
    {
      wl.data = 0;
      wr.data = 0;
      first_time = current_time;
    }

    // Publishing Values
    // pub_left_rpm.publish(wl);
    // pub_right_rpm.publish(wr);
    //ROS_INFO_STREAM("Speeds published.");
    ros::spinOnce();
    loop_rate.sleep();
  }

}
