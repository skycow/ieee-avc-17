#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


def cmd_callback(data):
    global ackermann_cmd_topic
    global frame_id
    global pub
    
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.drive.steering_angle = 0.2
    msg.drive.speed = 0.0
    
    pub.publish(msg)
    

if __name__ == '__main__': 
    try:
        
        rospy.init_node('cmd_vel_to_ackermann_drive')
                
        laser_topic = '/sensors/scan'
        cmd_topic = '/vesc/ackermann_cmd_mux/input/navigation'
        frame_id = rospy.get_param('~frame_id', '/vesc/odom')
        
        rospy.Subscriber(laser_topic, LaserScan, cmd_callback, queue_size=1)
        pub = rospy.Publisher(cmd_topic, AckermannDriveStamped, queue_size=1)
        
        rospy.loginfo("Node 'auto_steering' started")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

