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
    planning_options.acceleration_scaling_factor = 0.15;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

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


    //Write your code for following the square trajectory here.
     // Create instance of pose target plan
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan_start;

    geometry_msgs::Pose pose_target_s;
    pose_target_s.position.x = 0.25;
    pose_target_s.position.y = 0.2;
    pose_target_s.position.z = 1.1;
    pose_target_s.orientation.x = -0.707;
    pose_target_s.orientation.y = 0.0;
    pose_target_s.orientation.z = 0.0;
    pose_target_s.orientation.w = 0.707;

    bool pose_plan_success;
    std::string reference_frame = "world";
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target_s, reference_frame, pose_plan_start);
    
    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan_start);
    }

/*
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_1;

    // Get the start Pose
    geometry_msgs::Pose start_pose_1 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_1 = start_pose_1;
    end_pose_1.position.x += 0;
    end_pose_1.position.y -= 0;
    end_pose_1.position.z += 0.1;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(end_pose_1);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose_1, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);
    






    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_2;

    // Get the start Pose
    geometry_msgs::Pose start_pose_2 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_2 = start_pose_2;
    end_pose_2.position.x += 0.1;
    end_pose_2.position.y -= 0;
    end_pose_2.position.z -= 0;

    // Define waypoints for the cartesian path
    waypoints.push_back(end_pose_2);

    trajectory = ArmController::planCartesianPath(start_pose_2, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);






    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_3;
*/
    // Get the start Pose
    geometry_msgs::Pose start_pose_3 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_3 = start_pose_3;
    end_pose_3.position.x += 0;
    end_pose_3.position.y -= 0;
    end_pose_3.position.z -= 0.1;

    // Define waypoints for the cartesian path
    waypoints.push_back(end_pose_3);

    trajectory = ArmController::planCartesianPath(start_pose_3, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);
/*
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_4;

    // Get the start Pose
    geometry_msgs::Pose start_pose_4 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_4 = start_pose_4;
    end_pose_4.position.x -= 0.1;
    end_pose_4.position.y -= 0;
    end_pose_4.position.z -= 0;

    // Define waypoints for the cartesian path
    waypoints.push_back(end_pose_4);

    trajectory = ArmController::planCartesianPath(start_pose_4, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);

/*
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan_1;

    geometry_msgs::Pose pose_target_1;
    pose_target_1.position.x = 0.25;
    pose_target_1.position.y = 0.2;
    pose_target_1.position.z = 1.0;
    pose_target_1.orientation.x = -0.707;
    pose_target_1.orientation.y = 0.0;
    pose_target_1.orientation.z = 0.0;
    pose_target_1.orientation.w = 0.707;

    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target_1, reference_frame, pose_plan_1);
    
    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan_1);
    }

    moveit::planning_interface::MoveGroupInterface::Plan pose_plan_2;

    geometry_msgs::Pose pose_target_2;
    pose_target_2.position.x = 0.15;
    pose_target_2.position.y = 0.2;
    pose_target_2.position.z = 1.0;
    pose_target_2.orientation.x = -0.707;
    pose_target_2.orientation.y = 0.0;
    pose_target_2.orientation.z = 0.0;
    pose_target_2.orientation.w = 0.707;

    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target_2, reference_frame, pose_plan_2);
    
    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan_2);
    }

    moveit::planning_interface::MoveGroupInterface::Plan pose_plan_4;

    geometry_msgs::Pose pose_target_4;
    pose_target_4.position.x = 0.15;
    pose_target_4.position.y = 0.2;
    pose_target_4.position.z = 1.1;
    pose_target_4.orientation.x = -0.707;
    pose_target_4.orientation.y = 0.0;
    pose_target_4.orientation.z = 0.0;
    pose_target_4.orientation.w = 0.707;

    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target_3, reference_frame, pose_plan_3);
    
    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan_3);
    }

    moveit::planning_interface::MoveGroupInterface::Plan pose_plan_4;

    geometry_msgs::Pose pose_target_4;
    pose_target_4.position.x = 0.25;
    pose_target_4.position.y = 0.2;
    pose_target_4.position.z = 1.1;
    pose_target_4.orientation.x = -0.707;
    pose_target_4.orientation.y = 0.0;
    pose_target_4.orientation.z = 0.0;
    pose_target_4.orientation.w = 0.707;

    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target_4, reference_frame, pose_plan_4);
    
    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan_4);
    }
    */
}