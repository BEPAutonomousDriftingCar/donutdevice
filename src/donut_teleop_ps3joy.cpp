// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopDonut
{
public:
  TeleopDonut();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh;

  int linear_, angular_;
  double l_scale_, a_scale_,r_scale;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};


TeleopDonut::TeleopDonut():
  linear_(1),
  angular_(2)
{

  nh.param("axis_linear", linear_, linear_);
  nh.param("axis_angular", angular_, angular_);
  nh.param("scale_angular", a_scale_, a_scale_);
  nh.param("scale_linear", l_scale_, l_scale_);
  nh.param("scale_reverse", r_scale_, r_scale_);

  vel_pub_ = nh.advertise<geometry_msgs::Twist>("donutdevice/cmd_vel", 1);


  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopDonut::joyCallback, this);

}

void TeleopDonut::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  if (joy->axes[PS3_BUTTON_REAR_RIGHT_1]==0) {
    vel.linear.x = l_scale_*(joy->axes[PS3_AXIS_BUTTON_REAR_RIGHT_2]-joy->axes[PS3_AXIS_BUTTON_REAR_LEFT_2]);
    if(vel.linear.x<=0){
      vel.linear.x=r_scale*vel.linear.x;
    }
  }
  else {
    vel.angular.x=-1;
  }
  vel.angular.z = a_scale_*joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
  vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_donut");
  TeleopDonut teleop_donut;

  ros::spin();
  return 0;
}