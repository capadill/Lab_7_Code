
#include "../include/square.hpp"

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "square");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    //Write your code for following the square trajectory here.
     
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;

    joint_targets["elbow_joint"] = 0.8203;
    joint_targets["shoulder_lift_joint"] = -0.5236;
    joint_targets["shoulder_pan_joint"] = 0.8552;
    joint_targets["wrist_1_joint"] = 2.8449;
    joint_targets["wrist_2_joint"] = -0.8552;
    joint_targets["wrist_3_joint"] = 3.1416;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }
    std::string reference_frame = "world";
    // Create instance of cartesian plan
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;

    // Get the start Pose
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose edge_1= start_pose;
    edge_1.position.x = 0.2;
    edge_1.position.z = 1.3;
    edge_1.position.y = 0.4;
    geometry_msgs::Pose edge_2 = edge_1;
    edge_2.position.z -= 0.2;
    geometry_msgs::Pose edge_3 = edge_2;
    edge_3.position.x -= 0.2;
    //
    geometry_msgs::Pose edge_4 = edge_3;
    edge_4.position.z += 0.2;
    geometry_msgs::Pose edge_5 = edge_4;
    edge_5.position.x += 0.2;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(edge_1);
    waypoints.push_back(edge_2);
    waypoints.push_back(edge_3);
    waypoints.push_back(edge_4);
    waypoints.push_back(edge_5);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);

}