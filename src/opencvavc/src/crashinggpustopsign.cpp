#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include <sstream>
#include "time.h"

using namespace std;
#include "gpu/gpu.hpp"
#include "highgui/highgui.hpp"
//#include "objdetect/objdetect.hpp"
using namespace cv;



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
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
  ros::init(argc, argv, "safetyoverride");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/ackermann_cmd_mux/input/safety", 100);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int msgcount = 0;
  bool lookingforsign = true;
  int threshold = 20;
  int count = 0;
  int pixelsignsize = 180;
  int percentofscreen = 5;
  bool notStopped = true;
  int waitTime = 5;
  //std::string cascPath = "/home/ubuntu/ieee_avc/src/opencvavc/src/stopsign_classifier.xml";
  std::string cascPath = "~/opencv-3.2.0/data/haarcascades_GPU/haarcascade_eye.xml";
  cv::gpu::CascadeClassifier_GPU myCascade;// = cv2::CascadeClassifier(cascPath);
  myCascade.load(cascPath);
  cv::VideoCapture video_capture(0);
  cv::Mat frame;
  cv::gpu::GpuMat gray, signs;
  //std::vector<cv::Rect> signs;

  while (ros::ok())
  {
    
    video_capture >> frame;
    //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.0
    //imshow( "Display window", frame );                   // Show our image inside it.
    waitKey(0);
    cv::gpu::cvtColor(cv::gpu::GpuMat(frame), gray, CV_BGR2GRAY);
    if(lookingforsign)
    {
      int found = myCascade.detectMultiScale(gray,signs, 1.1, 5, Size(30,30));
      if(found == 0)
      {
        count = 0;
      }
      else
      {
        count++;
      }
      if(count > threshold)
      {
        ROS_INFO("Found a stop sign with c++.");
        time_t t_end = time(NULL) + waitTime;
        while(time(NULL) < t_end)
        {
          /**
           * This is a message object. You stuff it with data, and then publish it.
           */
          //std_msgs::String msg;
          ackermann_msgs::AckermannDriveStamped msg;
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = "";
          msg.drive.steering_angle = 0;
          msg.drive.speed = 0;


          /**
           * The publish() function is how you send messages. The parameter
           * is the message object. The type of this object must agree with the type
           * given as a template parameter to the advertise<>() call, as was done
           * in the constructor above.
           */
          chatter_pub.publish(msg);
          loop_rate.sleep();
          ++msgcount;

        }
        count = 0;
      }

    }
    

    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());


    //ros::spinOnce();

  }


  return 0;
}

