#include "../include/circle.hpp"

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "circle");
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

    //Write your code for following the circle trajectory here.
     //Vertical x-z plane
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = -1.8326;
    joint_targets["shoulder_lift_joint"] = -1.39626;
    joint_targets["shoulder_pan_joint"] = -1.93732;
    joint_targets["wrist_1_joint"] = 0.0;
    joint_targets["wrist_2_joint"] = 2.0;
    joint_targets["wrist_3_joint"] = 0.0;

    bool joint_plan_success;
    std::string reference_frame = "world";
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }


    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_1;

    // Get the start Pose
    geometry_msgs::Pose start_pose_1 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_1 = start_pose_1;
    end_pose_1.position.x += 0;
    end_pose_1.position.y -= 0;
    end_pose_1.position.z += 0.05;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(end_pose_1);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose_1, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);

    float r = 0.05;
    float phi = 0.314159;
    for(int i = 0; i < 20; i++){
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_1;
        
        // Get the start Pose
        geometry_msgs::Pose start_pose_1 = arm_move_group.getCurrentPose().pose;
        
        geometry_msgs::Pose end_pose_1 = start_pose_1;
        end_pose_1.position.x += 0 + r*cos(phi);
        end_pose_1.position.y -= 0; 
        end_pose_1.position.z += 0.05 + r*sin(phi);
        
        // Define waypoints for the cartesian path
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(end_pose_1);
        
        moveit_msgs::RobotTrajectory trajectory;
        trajectory = ArmController::planCartesianPath(start_pose_1, waypoints, reference_frame, arm_move_group);
        
        n.setParam("/record_pose", true);
        arm_move_group.execute(trajectory);
        n.setParam("/record_pose", false);
        
        phi += 0.314159;
    }
}