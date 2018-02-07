#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Poses {
	float pos[2];
	float rot[2];
};

int main(int argc, char** argv){
	ros::init(argc, argv, "nav_goals");
	ros::param::set("/free_goal_vel", true);
	ros::param::set("/xy_goal_tolerance", 0.4);
	ros::param::set("/yaw_goal_tolerance", 1.0);

	int numPoses = 12;
	struct Poses* poses = new Poses[numPoses];

	//Position(12.840, -1.076, 0.000), Orientation(0.000, 0.000, -0.065, 0.998) = Angle: -0.131
    poses[0].pos[0] = 20.3788032532;
    poses[0].pos[1] = -2.31015872955;
    poses[0].rot[0] = 0.188468012335;
    poses[0].rot[1] = 0.982079328938;

    poses[1].pos[0] = 30.1692428589;
    poses[1].pos[1] = 2.53006577492;
    poses[1].rot[0] = 0.55481614631;
    poses[1].rot[1] = 0.83197298261;
   
    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[2].pos[0] = 31.4894523621;
    poses[2].pos[1] = 8.62498855591;
    poses[2].rot[0] = 0.854742789544;
    poses[2].rot[1] = 0.519051792909;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[3].pos[0] = 24.9882183075;
    poses[3].pos[1] = 16.4297847748;
    poses[3].rot[0] = 0.984987104418;
    poses[3].rot[1] = 0.172627935546;

    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[4].pos[0] = 15.9903507233;
    poses[4].pos[1] = 15.3797588348;
    poses[4].rot[0] = 0.88044295348;
    poses[4].rot[1] = -0.474152091283;

    poses[5].pos[0] = 12.5379505157;
    poses[5].pos[1] = 8.28273296356;
    poses[5].rot[0] = -0.635429816132;
    poses[5].rot[1] = 0.772158629279;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[6].pos[0] = 14.0567789078; 
    poses[6].pos[1] = 2.54482293129;
    poses[6].rot[0] = -0.661639725412;
    poses[6].rot[1] = 0.749821894691;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[7].pos[0] = 13.5165920258;
    poses[7].pos[1] = -6.9725446701;
    poses[7].rot[0] = -0.644821513673;
    poses[7].rot[1] = 0.764333183569;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[8].pos[0] = 14.2164926529;
    poses[8].pos[1] = -19.5606555939;
    poses[8].rot[0] = 0.873626148401;
    poses[8].rot[1] = -0.486597732045;

//Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[9].pos[0] = 1.59010624886;
    poses[9].pos[1] = -24.7894496918;
    poses[9].rot[0] = 0.943969258767;
    poses[9].rot[1] = 0.330033389983;

//Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[10].pos[0] = -3.1943116188;
    poses[10].pos[1] = -13.8743724823;
    poses[10].rot[0] = 0.526520305235;
    poses[10].rot[1] = 0.850162553971;

//Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[11].pos[0] = 10.4544706345;
    poses[11].pos[1] = -5.40554046631;
    poses[11].rot[0] = 0.190887267714;
    poses[11].rot[1] = 0.981611965608;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    for (int i=0; i<numPoses; ++i) {
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = poses[i].pos[0];
        goal.target_pose.pose.position.y = poses[i].pos[1];
        goal.target_pose.pose.orientation.z = poses[i].rot[0];
        goal.target_pose.pose.orientation.w = poses[i].rot[1];

        ROS_INFO("Sending goal #%d", i);
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal #%d reached", i);
        }
        else {
            ROS_INFO("Failed to reach goal #%d", i);
	    --i;
	    continue;
        }
    }

    return 0;
}

/*
---
header: 
  seq: 5
  stamp: 
    secs: 1507984204
    nsecs: 423257560
  frame_id: map
pose: 
  position: 
    x: 4.58767414093
    y: -25.4991645813
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.975034801782
    w: 0.222052100451
---
header: 
  seq: 6
  stamp: 
    secs: 1507984213
    nsecs: 775688651
  frame_id: map
pose: 
  position: 
    x: -0.372051239014
    y: -10.5009384155
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.307050199578
    w: 0.951693319793
---
header: 
  seq: 7
  stamp: 
    secs: 1507984224
    nsecs: 842439302
  frame_id: map
pose: 
  position: 
    x: 10.1185894012
    y: -6.00762653351
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.169557705202
    w: 0.985520260881
---

*/
