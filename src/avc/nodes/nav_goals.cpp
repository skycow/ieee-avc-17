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


    int numPoses = 11;
    struct Poses* poses = new Poses[numPoses];

    //Position(12.840, -1.076, 0.000), Orientation(0.000, 0.000, -0.065, 0.998) = Angle: -0.131
    poses[0].pos[0] =16.00661;
    poses[0].pos[1] = -2.09784;
    poses[0].rot[0] = -0.72195;
    poses[0].rot[1] = 0.69194;

    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[1].pos[0] = 19.94042;
    poses[1].pos[1] =  -7.04888;
    poses[1].rot[0] = -0.00943;
    poses[1].rot[1] = 0.99995;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[2].pos[0] =44.72389; 
    poses[2].pos[1] =-11.50415;
    poses[2].rot[0] =-0.69917;
    poses[2].rot[1] =0.71494;

    poses[3].pos[0] =45.41506;
    poses[3].pos[1] = -30.45268;
    poses[3].rot[0] =-0.01879;
    poses[3].rot[1] =0.99982;

    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[4].pos[0] = 55.71775; 
    poses[4].pos[1] =-31.06762;
    poses[4].rot[0]= -0.07175;
    poses[4].rot[1] =0.99742; 

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[5].pos[0] = 71.49681;
    poses[5].pos[1] = -30.36725;
    poses[5].rot[0] = 0.70864;
    poses[5].rot[1] = 0.70556;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[6].pos[0] = 71.85166;
    poses[6].pos[1] = -21.17044;
    poses[6].rot[0] = 0.67549;
    poses[6].rot[1] = 0.73736;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[7].pos[0] = 68.77900;
    poses[7].pos[1] = -8.89277;
    poses[7].rot[0] = 0.99912;
    poses[7].rot[1] = 0.04192;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[8].pos[0] = 40.76310;
    poses[8].pos[1] = -7.91019;
    poses[8].rot[0] = 0.99984;
    poses[8].rot[1] = 0.01765;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[9].pos[0] = 15.80878;
    poses[9].pos[1] = -4.09852;
    poses[9].rot[0] = 0.68877;
    poses[9].rot[1] = 0.72497;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[10].pos[0] = 4.80525;
    poses[10].pos[1] = 0.08393;
    poses[10].rot[0] = 0.99967;
    poses[10].rot[1] = 0.02559;

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
