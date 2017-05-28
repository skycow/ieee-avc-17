#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from steering_utils.utils import denormalize
import tensorflow as tf


n_input = 271
n_output = 1
dropout = 1.0

x = tf.placeholder(tf.float32, [None, n_input])
y = tf.placeholder(tf.float32, [None, n_output])
keep_prob = tf.placeholder(tf.float32)

def conv1d(x, W, b, stride=1):
    x = tf.nn.conv1d(x, W, stride=stride, padding='SAME')
    x = tf.nn.bias_add(x, b)
    return tf.nn.elu(x)

def conv_net(x, weights, biases, dropout):
    out = tf.reshape(x, shape=[-1, 271, 1])
    out = conv1d(out, weights['wc1'], biases['bc1'])
    out = conv1d(out, weights['wc2'], biases['bc2'])
    out = tf.reshape(out, [-1, weights['fc1'].get_shape().as_list()[0]])
    out = tf.add(tf.matmul(out, weights['fc1']), biases['fc1'])
    out = tf.nn.elu(out)
    out = tf.add(tf.matmul(out, weights['fc2']), biases['fc2'])
    out = tf.nn.elu(out)
    out = tf.add(tf.matmul(out, weights['out']), biases['out'])
    return out

conv1_size = 7
conv2_size = 7
l1_size = 24
l2_size = l1_size
full_size_1 = 64
full_size_2 = 32
full_size_3 = 32
weights = {

    'wc1': tf.Variable(tf.random_normal([conv1_size, 1, l1_size])),
    'wc2': tf.Variable(tf.random_normal([conv2_size, l1_size, l2_size])),

    'fc1': tf.Variable(tf.random_normal([271 * l2_size, full_size_1])),
    'fc2': tf.Variable(tf.random_normal([full_size_1, full_size_2])),
    'fc3': tf.Variable(tf.random_normal([full_size_2, full_size_3])),

    'out': tf.Variable(tf.random_normal([full_size_3, n_output]))
}

biases = {
    'bc1': tf.Variable(tf.random_normal([l1_size])),
    'bc2': tf.Variable(tf.random_normal([l2_size])),
    'fc1': tf.Variable(tf.random_normal([full_size_1])),
    'fc2': tf.Variable(tf.random_normal([full_size_2])),
    'fc3': tf.Variable(tf.random_normal([full_size_3])),
    'out': tf.Variable(tf.random_normal([n_output]))
}

pred = conv_net(x, weights, biases, keep_prob)
init = tf.global_variables_initializer()
saver = tf.train.Saver()
sess = tf.Session()
saver.restore(sess, 'steering_utils/model/test.ckpt')


def cmd_callback(data):
    global frame_id
    global pub
    normalized_prediction = sess.run(pred, feed_dict={x: [data.ranges]})
    prediction = denormalize(normalized_prediction)
    
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.drive.steering_angle = prediction
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

