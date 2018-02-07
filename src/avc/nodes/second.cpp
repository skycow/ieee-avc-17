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

	int numPoses = 7;
	struct Poses* poses = new Poses[numPoses];

	//Position(12.840, -1.076, 0.000), Orientation(0.000, 0.000, -0.065, 0.998) = Angle: -0.131
    poses[0].pos[0] = 18.0435523987;
    poses[0].pos[1] = -2.92783951759;
    poses[0].rot[0] = 0.188467965353;
    poses[0].rot[1] = 0.982079337954;

    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[1].pos[0] = 31.9820270538;
    poses[1].pos[1] = 6.8506064415;
    poses[1].rot[0] = 0.725733909014;
    poses[1].rot[1] = 0.687975503421;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[2].pos[0] = 26.3127059937;
    poses[2].pos[1] = 15.7774839401;
    poses[2].rot[0] = 0.967754518871;
    poses[2].rot[1] = 0.251895198852;

    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[3].pos[0] = 13.7842350006; 
    poses[3].pos[1] = 11.0979137421;
    poses[3].rot[0] = -0.710321933622;
    poses[3].rot[1] = 0.703876942807;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[4].pos[0] = 14.4668140411;
    poses[4].pos[1] = -3.57088804245;
    poses[4].rot[0] = -0.656639986684;
    poses[4].rot[1] = 0.75420416857;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[5].pos[0] = 4.58767414093;
    poses[5].pos[1] = -25.4991645813;
    poses[5].rot[0] = 0.975034801782;
    poses[5].rot[1] = 0.222052100451;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[6].pos[0] = -0.372051239014;
    poses[6].pos[1] = -10.5009384155;
    poses[6].rot[0] = 0.307050199578;
    poses[6].rot[1] = 0.951693319793;

//Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[7].pos[0] = 10.1185894012;
    poses[7].pos[1] = -6.00762653351;
    poses[7].rot[0] = 0.169557705202;
    poses[7].rot[1] = 0.985520260881;

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
