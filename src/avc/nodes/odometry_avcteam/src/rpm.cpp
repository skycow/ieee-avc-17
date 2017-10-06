#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "jetsonGPIO.h"

int main(int argc, char** argv){

  jetsonGPIO odomSensor = gpio166 ;
  gpioExport(odomSensor) ;
  gpioSetDirection(odomSensor,inputPin) ;

  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  unsigned int rpm_ping = 0;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0; 
  double vy = 0;
  double vth = 0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  double dt_last = current_time;  

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    
    gpioGetValue(odomSensor, &rpm_ping) ; //rpm_ping is active low
    
    //compute odometry by reading the rpm_ping data and dt value. 
    //if rpm_ping is low, the magnet is in range. This means it has traveled 
    //10.5cm. Since this code runs at high speed, the no new values can be 
    //read for 50ms. Therfore, we must store the curr_time of the last ping,
    //so we can ensure that 50ms have passed.
    ///TODO: account for super slow conditions. I.E events where the magnet
    //spends significant time in front of sensor. 
    ///TODO: account for backwars travel. 
    
    double dt = (current_time - last_time).toSec(); 
    if(rpm_ping == 0)
    {
      if(dt-dt_last >= .05)
      {
        double delta_x =  10.5; //cm
        dt_last = current_time;
      }
    }

    x += delta_x;// replace delta_x with odom measurments
    y = 0;// set to zero since the car should not move in this direction
    th = 0; //set to zero since theta is not relizable in this node

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

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
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
  gpioExport(odomSensor);      // export the odom sensor
}
