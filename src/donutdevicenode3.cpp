#include "ros/ros.h"
#include <string>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "donutdevice/Steer.h"
#include "donutdevice/IntWheels.h"
#include "donutdevice/Donut.h"
#include <sstream>

class Translater
{
  public:
  Translater()
  {
  ROS_INFO("Starting translater node");
  imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
  mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
  wheels = n.advertise<geometry_msgs::QuaternionStamped>("wheels", 10);
  steer = n.advertise<donutdevice::Steer>("steer", 10);
  vo = n.advertise<nav_msgs::Odometry>("vo", 10);

  donutsub = n.subscribe("donut", 1, &Translater::donutCallback, this);
  mocapsub = n.subscribe("ground_pose", 1, &Translater::mocapCallback, this);
  }
  void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    odom_msg.header.seq = msg->header.seq;
    odom_msg.header.stamp = msg->header.stamp;                   // time of gps measurement
    odom_msg.header.frame_id = base_footprint;          // the tracked robot frame
    odom_msg.pose.pose.position.x = msg->pose.position.x;             // x measurement GPS.
    odom_msg.pose.pose.position.y = msg->pose.position.y;              // y measurement GPS.
    odom_msg.pose.pose.position.z = msg->pose.position.z;              // z measurement GPS.
    odom_msg.pose.pose.orientation.x = msg->pose.orientation.x;             // identity quaternion
    odom_msg.pose.pose.orientation.y = msg->pose.orientation.y;               // identity quaternion
    odom_msg.pose.pose.orientation.z = msg->pose.orientation.z;               // identity quaternion
    odom_msg.pose.pose.orientation.w = msg->pose.orientation.w;               // identity quaternion
    odom_msg.pose.covariance = {cov_x, 0, 0, 0, 0, 0,  // covariance on gps_x
                                0, cov_y, 0, 0, 0, 0,  // covariance on gps_y
                                0, 0, cov_z, 0, 0, 0,  // covariance on gps_z
                                0, 0, 0, cov_x, 0, 0,  // large covariance on rot x
                                0, 0, 0, 0, cov_y, 0,  // large covariance on rot y
                                0, 0, 0, 0, 0, cov_z};  // large covariance on rot z
    vo.publish(odom_msg);
  }
  void donutCallback(const donutdevice::Donut::ConstPtr& msg)
  {

    s_msg.header.seq = msg->dynamixel.header.seq;
    s_msg.header.stamp = msg->dynamixel.header.stamp;
    s_msg.angle = inttoangle(msg->dynamixel.angle);
    s_msg.load = inttoload(msg->dynamixel.load);


    w_msg.header.seq = msg->wheels.header.seq;
    w_msg.header.stamp = msg->wheels.header.stamp;
    w_msg.quaternion.x = msg->wheels.fl*pi;
    w_msg.quaternion.y = msg->wheels.fr*pi;
    w_msg.quaternion.z = msg->wheels.rl*pi;
    w_msg.quaternion.w = msg->wheels.rr*pi;


    imu_msg.header.seq = msg->mpu.header.seq;
    imu_msg.header.stamp = msg->mpu.header.stamp;

    imu_msg.linear_acceleration.x = msg->mpu.linear_acceleration.x;
    imu_msg.linear_acceleration.y = msg->mpu.linear_acceleration.y;
    imu_msg.linear_acceleration.z = msg->mpu.linear_acceleration.z;
    imu_msg.angular_velocity.x = msg->mpu.angular_velocity.x;
    imu_msg.angular_velocity.y = msg->mpu.angular_velocity.y;
    imu_msg.angular_velocity.z = msg->mpu.angular_velocity.z;

    mag_msg.header.seq = msg->mpu.header.seq;
    mag_msg.header.stamp = msg->mpu.header.stamp;

    mag_msg.magnetic_field.x =  msg->mpu.magnetic_field.x;
    mag_msg.magnetic_field.y = msg->mpu.magnetic_field.y;
    mag_msg.magnetic_field.z = msg->mpu.magnetic_field.z;

    mag_pub.publish(mag_msg);
    imu_pub.publish(imu_msg);
    wheels.publish(w_msg);
    steer.publish(s_msg);
  }
  private:
  std::string base_footprint = "DonutDevice/base_link";

  const double pi = 0.26179938779;

  float cov_x =  999;
  float cov_y =  999;
  float cov_z = 999;

  double inttoangle(int intangle){
    double angle;
    angle = (intangle-511)*(0.24719101123);
    return angle;
  }
  int inttoload(int intload){
    if(intload >= 1023){
      intload = intload - 1023;
    }
    return intload;
  }
  ros::NodeHandle n;
  ros::Publisher imu_pub;
  ros::Publisher mag_pub;
  ros::Publisher wheels;
  ros::Publisher steer;
  ros::Publisher vo;

  ros::Subscriber donutsub;
  ros::Subscriber mocapsub;

  nav_msgs::Odometry odom_msg;
  sensor_msgs::Imu imu_msg;
  sensor_msgs::MagneticField mag_msg;
  geometry_msgs::QuaternionStamped w_msg;
  donutdevice::Steer s_msg;
  donutdevice::Donut donut_msg;


};


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "donutdevice");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  Translater Translater;
  ros::spin();
  return 0;
}
