#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <fstream>
#include <sstream>

#include "custom_interfaces/srv/add_three_ints.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning");
static std::shared_ptr<rclcpp::Node> motion_planning_node;

// Function to print the joint trajectory
void printTrajectory(const trajectory_msgs::msg::JointTrajectory &joint_trajectory)
{
    std::cout << "Joint Trajectory: " << std::endl;
    std::cout << "  Joint Names: ";
    for (const auto &joint_name : joint_trajectory.joint_names)
    {
        std::cout << joint_name << " ";
    }
    std::cout << std::endl;

    // Iterate through trajectory points
    for (size_t i = 0; i < joint_trajectory.points.size(); ++i)
    {
        const auto &point = joint_trajectory.points[i];
        std::cout << "  Point " << i + 1 << ":" << std::endl;
        std::cout << "    Positions: ";
        for (const auto &position : point.positions)
        {
            std::cout << position << " ";
        }
        std::cout << std::endl;

        std::cout << "    Velocities: ";
        for (const auto &velocity : point.velocities)
        {
            std::cout << velocity << " ";
        }
        std::cout << std::endl;

        std::cout << "    Accelerations: ";
        if (!point.accelerations.empty())
        {
            for (const auto &acceleration : point.accelerations)
            {
                std::cout << acceleration << " ";
            }
            std::cout << std::endl;
        }
        else
        {
            std::cout << "N/A" << std::endl;
        }

        std::cout << "    Time From Start: " << point.time_from_start.sec << "s " 
                  << point.time_from_start.nanosec << "ns" << std::endl;
    }
}

trajectory_msgs::msg::JointTrajectory getJointTrajectory(const planning_interface::MotionPlanResponse &res)
{
    const robot_trajectory::RobotTrajectoryPtr &robot_trajectory = res.trajectory_;
    moveit_msgs::msg::RobotTrajectory msg_robot_trajectory;
    robot_trajectory->getRobotTrajectoryMsg(msg_robot_trajectory);
    trajectory_msgs::msg::JointTrajectory joint_trajectory = msg_robot_trajectory.joint_trajectory;
    return joint_trajectory;
}

trajectory_msgs::msg::JointTrajectory planning_func(std::shared_ptr<rclcpp::Node> motion_planning_node, geometry_msgs::msg::Point eef_position, geometry_msgs::msg::Quaternion eef_orientation, sensor_msgs::msg::JointState joint_state) {
    const std::string PLANNING_GROUP = "panda_arm";
    robot_model_loader::RobotModelLoader robot_model_loader(motion_planning_node, "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;
    
    motion_planning_node->get_parameter("planning_plugin", planner_plugin_name);
    std::cout << "Planner plugin name: " << planner_plugin_name << std::endl;

    if (!motion_planning_node->get_parameter("planning_plugin", planner_plugin_name))
      RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, motion_planning_node,
                                        motion_planning_node->get_namespace()))
        RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
      RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (const auto& cls : classes)
        ss << cls << " ";
      RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                  ex.what(), ss.str().c_str());
    }

    // moveit::planning_interface::MoveGroupInterface move_group(motion_planning_node, PLANNING_GROUP);
    std::cout << "Move group interface created" << std::endl;

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "fr3_link0";
    // pose.pose.position.x = 0.3;
    // pose.pose.position.y = 0.4;
    // pose.pose.position.z = 0.75;
    pose.pose.position = eef_position;
    // std::cout << "Point(x: " << eef_position.x
    //           << ", y: " << eef_position.y
    //           << ", z: " << eef_position.z << ")" << std::endl;
    // std::cout << eef_position. << std::endl;
    // pose.pose.orientation.w = 1.0;
    pose.pose.orientation = eef_orientation;

    moveit::core::RobotState& current_state = planning_scene->getCurrentStateNonConst();
    // std::vector<double> joint_positions = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.0};
    // std::cout << joint_state.position << std::endl; 
    // const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
    // for (size_t i = 0; i < joint_names.size(); ++i) {
    //     RCLCPP_INFO(LOGGER, " - %s: %f", joint_names[i].c_str());
    // }
    // for (size_t i = 0; i < joint_state.name.size(); ++i)
    //     {
    //         RCLCPP_INFO(LOGGER, "Joint Name: %s, Position: %f",
    //                     joint_state.name[i].c_str(),
    //                     i < joint_state.position.size() ? joint_state.position[i] : 0.0);
    //     }
    // std::size_t expected_size = joint_model_group->getVariableCount();
    // RCLCPP_INFO(LOGGER, "Expected joint positions size: %zu", expected_size);

    current_state.setJointGroupPositions(joint_model_group, joint_state.position);

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("fr3_link7", pose, tolerance_pose, tolerance_angle);

    req.group_name = PLANNING_GROUP;
    req.goal_constraints.push_back(pose_goal);

    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
      return trajectory_msgs::msg::JointTrajectory();
    }
    else {
      RCLCPP_INFO(LOGGER, "Plan computed successfully");
    }

    // Get the joint trajectory
    trajectory_msgs::msg::JointTrajectory joint_trajectory = getJointTrajectory(res);

    // Print the trajectory
    // printTrajectory(joint_trajectory);
    return joint_trajectory;
}

void handle_plan_service(const std::shared_ptr<custom_interfaces::srv::AddThreeInts::Request> request,
         std::shared_ptr<custom_interfaces::srv::AddThreeInts::Response> response)
{
    trajectory_msgs::msg::JointTrajectory traj = planning_func(motion_planning_node, request->position, request->quaternion, request->joint_state);
    response->trajectory = traj;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
    //             request->a, request->b);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  motion_planning_node =
      rclcpp::Node::make_shared("MotionPlanningNode", node_options);

  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(motion_planning_node);
  // std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::Service<custom_interfaces::srv::AddThreeInts>::SharedPtr service =
        motion_planning_node->create_service<custom_interfaces::srv::AddThreeInts>("planning_service", &handle_plan_service);

  while (rclcpp::ok())
  {
    rclcpp::spin(motion_planning_node);
  }
  rclcpp::shutdown();
  return 0;
}
