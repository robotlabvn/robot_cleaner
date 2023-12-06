#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_cpp");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // Move joint
   moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
   
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = 0.0;  // radians
  joint_group_positions[1] = -1.57;  // radians
  joint_group_positions[2] =  1.57;  // radians
  joint_group_positions[3] = -1.57;  // radians
  joint_group_positions[4] = -1.57;  // radians
  joint_group_positions[5] = -1.57;  // radians
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Move pose
  
  
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  

 
  
  // // Planning with Path Constraints
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Path constraints can easily be specified for a link on the robot.
  // // Let's specify a path constraint and a pose goal for our group.
  // // First define the path constraint.
  // moveit_msgs::OrientationConstraint ocm;
  // ocm.link_name = "wrist_3_link";
  // ocm.header.frame_id = "shoulder_link";
  // ocm.orientation.y = 1.0;
  // ocm.absolute_x_axis_tolerance = 0.1;
  // ocm.absolute_y_axis_tolerance = 0.1;
  // ocm.absolute_z_axis_tolerance = 0.1;
  // ocm.weight = 1.0;

  // // Now, set it as the path constraint for the group.
  // moveit_msgs::Constraints test_constraints;
  // test_constraints.orientation_constraints.push_back(ocm);
  // move_group.setPathConstraints(test_constraints);

  // // We will reuse the old goal that we had and plan to it.
  // // Note that this will only work if the current state already
  // // satisfies the path constraints. So, we need to set the start
  // // state to a new pose.
  // robot_state::RobotState start_state(*move_group.getCurrentState());
  // geometry_msgs::Pose start_pose2;
  // start_pose2.orientation.y = 1.0;
  // start_pose2.position.x = 0.55;
  // start_pose2.position.y = -0.05;
  // start_pose2.position.z = 0.8;
  // start_state.setFromIK(joint_model_group, start_pose2);
  // move_group.setStartState(start_state);
  // // Now we will plan to the earlier pose target from the new
  // // start state that we have just created.
  // move_group.setPoseTarget(target_pose1);

  // // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  // move_group.setPlanningTime(10.0);
  // move_group.move();
  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

   // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  
  


  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);

  geometry_msgs::Pose target_pose3 = target_pose1;

  target_pose3.position.x += 0.04;
  waypoints.push_back(target_pose3);  // down

  // target_pose3.position.y += 0.2;
  // waypoints.push_back(target_pose3);  // right

  // target_pose3.position.x += 0.2;
  // waypoints.push_back(target_pose3);  // up and left

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maximum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.2);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  
  // Set the time increment between waypoints
  double time_increment = 0.1;
  // Set the time_from_start field of the first waypoint to a non-zero value
  trajectory.joint_trajectory.points[0].time_from_start = ros::Duration(time_increment);
  
  // Loop through each subsequent waypoint and set its time_from_start value
  for (size_t i = 1; i <  trajectory.joint_trajectory.points.size(); ++i)
  {
       trajectory.joint_trajectory.points[i].time_from_start =
           trajectory.joint_trajectory.points[i-1].time_from_start + ros::Duration(time_increment);
      
  }
  my_plan.trajectory_ = trajectory;
  move_group.execute(my_plan);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

 
  ros::waitForShutdown();
}