#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "serial.hpp"
#include "roboclaw.hpp"

// Geometrical paramters
#define WHEEL_RADIUS (0.123825/2.)                 // m
#define Wheelbase  (0.380)

// CPR + Gear
#define CPR_CODEUR 2048                            // tics / tr
#define GEAR_RATIO 16                              //
#define CPR_GEAR   (CPR_CODEUR*4*GEAR_RATIO)       // tics / tr (quadrature)

#define VMAX_MOT   (3000)                          // rpm
#define VMAX       ((VMAX_MOT/GEAR_RATIO)/60*CPR_GEAR)  // tics / s (quad)
#define PI         (3.145926)

// Unit conversion
#define M_2_TICS   (CPR_GEAR/(2*PI*WHEEL_RADIUS))
#define TICS_2_M   (1/M_2_TICS)

// Motor disposition
#define LEFT 0
#define RIGHT 1

// Gloal variables
int raw_motor_encoder[2];            // motor encoders tics (quad)
int raw_motor_encoder_prev[2];       // previous
int raw_motor_delta_encoder[2];      // delta
double wheel_velocity[2];
double battery_voltage;

ros::Time current_time, last_time;
ros::Time last_cmd_vel_time;

// jointstate
// odom
RoboClaw *motors_driver;



//-----------------------------------------------------------------------------
void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
  double vx = msg->linear.x;
  double wz = msg->angular.z;

  double v_right = vx + (Wheelbase/2.)*wz;
  double v_left  = vx - (Wheelbase/2.)*wz;

  double v_right_raw =  v_right*M_2_TICS;
  double v_left_raw  =  v_left *M_2_TICS;

//  ROS_INFO("cmd_vel = (%.2f,%.2f) -> (%.2f,%.2f) -> (%i,%i)",
//	   vx,wz, v_left, v_right, (int)v_left_raw, (int)v_right_raw);

  // Inverted rotation sign on right wheel
  motors_driver->set_PID_target(RIGHT, -(int) v_right_raw);
  motors_driver->set_PID_target(LEFT, (int) v_left_raw);
  last_cmd_vel_time = ros::Time::now();
}





//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{

  std_msgs::Float64 v_batt_msg;

  // initial position
  double x = 0;
  double y = 0;
  double th = 0;

  // Init driver
  motors_driver = new RoboClaw(128);
  if(motors_driver->init("/dev/ttyRoboClaw", 38400, 10e3))
    return 1;
  motors_driver->read_firmware();
  printf("Power voltage : %.2f V\n", motors_driver->get_power_voltage());
  motors_driver->reset_encoders();
  for(int id=0;id<2;id++) {
    motors_driver->set_PID_parameters(id, 22100, 6100, 1500, VMAX);
    usleep(200e3);
    motors_driver->set_PID_target(id,0);
  }


  // Init ROS
  ros::init(argc, argv, "minilab_node");
  ros::NodeHandle n;
  ros::Publisher batt_volt_pub = n.advertise<std_msgs::Float64>("v_batt", 10 );
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10 );
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, cmd_vel_cb);
  ros::Rate loop_rate(10);

  // Init Data
  last_cmd_vel_time = ros::Time::now();
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  // Main loop
  while (ros::ok())
  {    
    ros::spinOnce();
    current_time = ros::Time::now();

    double last_cmd_vel_dt = (current_time-last_cmd_vel_time).toSec();
    //ROS_INFO("last time = %.2f", last_cmd_vel_dt);
    if(last_cmd_vel_dt>1.0) {
      motors_driver->set_PID_target(RIGHT, 0);
      motors_driver->set_PID_target(LEFT, 0);
    }

    // Get wheel encoder measurments
    // Inverted rotation sign on right wheel
    raw_motor_encoder_prev[LEFT]  = raw_motor_encoder[LEFT];
    raw_motor_encoder_prev[RIGHT] = raw_motor_encoder[RIGHT];
    raw_motor_encoder[LEFT]  = +motors_driver->get_encoder(LEFT);
    usleep(20e3);
    raw_motor_encoder[RIGHT] = -motors_driver->get_encoder(RIGHT);
    usleep(20e3);

    raw_motor_delta_encoder[LEFT]=
      raw_motor_encoder[LEFT]-raw_motor_encoder_prev[LEFT];
    raw_motor_delta_encoder[RIGHT]=
      raw_motor_encoder[RIGHT]-raw_motor_encoder_prev[RIGHT];

    battery_voltage =  motors_driver->get_power_voltage();
    v_batt_msg.data = battery_voltage;
    batt_volt_pub.publish(v_batt_msg);
    
    // Compute robot delta pos in tics
    int raw_delta_x     = (raw_motor_delta_encoder[RIGHT]
			   +raw_motor_delta_encoder[LEFT])/2;
    int raw_delta_theta = (raw_motor_delta_encoder[RIGHT]
			   -raw_motor_delta_encoder[LEFT])/Wheelbase;

    // Compute robot delta pos in m
    double dl  = raw_delta_x * TICS_2_M;
    double dth = raw_delta_theta * TICS_2_M;
/*
    ROS_INFO("Vr, Vl = %i, %i", 
	     raw_motor_delta_encoder[LEFT],
	     raw_motor_delta_encoder[RIGHT]);
*/
    //ROS_INFO("dl, dth = %.3f, %.3f", dl, dth);

    ROS_INFO("Volts = %.3f", battery_voltage);

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = dl * cos(th);
    double delta_y = dl * sin(th);
    double delta_th = dth;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = dl/dt;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = dth/dt;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    loop_rate.sleep();
  }


  return 0;
}
