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

    int numPoses = 13;
    struct Poses* poses = new Poses[numPoses];

    //Position(12.840, -1.076, 0.000), Orientation(0.000, 0.000, -0.065, 0.998) = Angle: -0.131
    poses[0].pos[0] =-20.338596344;
    poses[0].pos[1] = 0.318251729012;
    poses[0].rot[0] = 0.99996561246;
    poses[0].rot[1] = -0.0082930029599;

    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[1].pos[0] = -20.4522323608; 
    poses[1].pos[1] = -16.1060504913;
    poses[1].rot[0] = -0.722066081281;
    poses[1].rot[1] = 0.691824091994;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[2].pos[0] = -20.694278717;
    poses[2].pos[1] = -37.8949546814;
    poses[2].rot[0] = -0.00904238829264;
    poses[2].rot[1] = 0.999959116771;

    poses[3].pos[0] = 0.683166980743;
    poses[3].pos[1] = -38.3546142578;
    poses[3].rot[0] = 0.704831787704;
    poses[3].rot[1] = 0.709374478708;

    //Position(8.433, -14.722, 0.000), Orientation(0.000, 0.000, -0.732, 0.681) = Angle: -1.642
    poses[4].pos[0] = 0.993792951107;
    poses[4].pos[1] = -18.1544742584;
    poses[4].rot[0] = 0.68357581891;
    poses[4].rot[1] = 0.729879510468;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[5].pos[0] = 1.72852647305; 
    poses[5].pos[1] = 0.608638763428;
    poses[5].rot[0] = -0.00196158387799;
    poses[5].rot[1] = 0.999998076092;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[6].pos[0] = 16.8318042755; 
    poses[6].pos[1] = -0.364023208618;
    poses[6].rot[0] = -0.753875647615;
    poses[6].rot[1] = 0.657017129102;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[7].pos[0] = 16.4061927795;
    poses[7].pos[1] = -6.40832519531;
    poses[7].rot[0] = -0.0578732918403;
    poses[7].rot[1] = 0.998323936451;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[8].pos[0] = 41.1328086853;
    poses[8].pos[1] = -7.3299946785;
    poses[8].rot[0] = -0.0355253444859;
    poses[8].rot[1] = 0.999368775728;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[9].pos[0] = 73.1540374756;
    poses[9].pos[1] = -8.69867706299;
    poses[9].rot[0] = -0.736025566632;
    poses[9].rot[1] = 0.676953739383;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[10].pos[0] = 71.2664794922;
    poses[10].pos[1] = -31.9376869202;
    poses[10].rot[0] = 0.999977197629;
    poses[10].rot[1] = -0.00675308974953;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[11].pos[0] = 44.1098060608;
    poses[11].pos[1] = -7.25391292572;
    poses[11].rot[0] = 0.999993030897;
    poses[11].rot[1] = 0.00373338426931;

    //Position(8.202, -22.073, 0.000), Orientation(0.000, 0.000, -0.753, 0.658) = Angle: -1.706
    poses[12].pos[0] = 1.47721743584;
    poses[12].pos[1] = 0.52663564682;
    poses[12].rot[0] = 0.999999616344;
    poses[12].rot[1] = -0.000875963035274;

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
