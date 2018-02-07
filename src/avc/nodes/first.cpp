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
    poses[0].pos[0] = 24.3563041687;
    poses[0].pos[1] = -0.885554432869;
    poses[0].rot[0] = 0.302491396804;
    poses[0].rot[1] = 0.953152115278;

    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[1].pos[0] = 25.8810005188;
    poses[1].pos[1] = 16.332069397;
    poses[1].rot[0] = 0.98257302653;
    poses[1].rot[1] = 0.185876968812;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[2].pos[0] = 15.1311779022;
    poses[2].pos[1] = 11.1205072403;
    poses[2].rot[0] = -0.771845847604;
    poses[2].rot[1] = 0.635809710162;

    //poses[3].pos[0] = 14.8527202606;
    //poses[3].pos[1] = -7.09735488892;
    //poses[3].rot[0] = -0.689336248493;
    //poses[3].rot[1] = 0.724441534228;

    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[3].pos[0] = 14.9024143219; 
    poses[3].pos[1] = -20.9094676971;
    poses[3].rot[0] = 0.928598096117;
    poses[3].rot[1] = -0.371087019293;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[4].pos[0] = 3.01051425934;
    poses[4].pos[1] = -25.2211914062;
    poses[4].rot[0] = 0.947313260764;
    poses[4].rot[1] = 0.320308579311;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[5].pos[0] = -1.35394191742;
    poses[5].pos[1] = -10.465338707;
    poses[5].rot[0] = 0.33967970775;
    poses[5].rot[1] = 0.940541171955;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[6].pos[0] = 8.49039173126;
    poses[6].pos[1] = -6.0890007019;
    poses[6].rot[0] = 0.21437342001;
    poses[6].rot[1] = 0.976751778495;

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
