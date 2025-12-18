#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <thread>

static void openGripper(moveit::planning_interface::MoveGroupInterface &hand_group,
                        const rclcpp::Logger &logger)
{
  auto joint_goal = hand_group.getCurrentJointValues();
  if (joint_goal.size() >= 2)
  {
    joint_goal[0] = 0.04;
    joint_goal[1] = 0.04;
    hand_group.setJointValueTarget(joint_goal);
    auto ec = hand_group.move();
    RCLCPP_INFO(logger, "Gripper open move() result = %d", (int)ec.val);
  }
  else
  {
    RCLCPP_WARN(logger, "Hand group joint size < 2, cannot open gripper.");
  }
}

static void closeGripper(moveit::planning_interface::MoveGroupInterface &hand_group,
                         const rclcpp::Logger &logger)
{
  auto joint_goal = hand_group.getCurrentJointValues();
  if (joint_goal.size() >= 2)
  {
    joint_goal[0] = 0.0;
    joint_goal[1] = 0.0;
    hand_group.setJointValueTarget(joint_goal);
    auto ec = hand_group.move();
    RCLCPP_INFO(logger, "Gripper close move() result = %d", (int)ec.val);
  }
  else
  {
    RCLCPP_WARN(logger, "Hand group joint size < 2, cannot close gripper.");
  }
}

static bool planAndExecutePose(moveit::planning_interface::MoveGroupInterface &arm_group,
                               const geometry_msgs::msg::Pose &target_pose,
                               const std::string &name,
                               const rclcpp::Logger &logger)
{
  RCLCPP_INFO(logger, "Planning to %s ...", name.c_str());

  arm_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = arm_group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Plan to %s FAILED, code=%d", name.c_str(), (int)plan_result.val);
    arm_group.clearPoseTargets();
    return false;
  }

  RCLCPP_INFO(logger, "Executing %s ...", name.c_str());
  auto exec_result = arm_group.execute(plan);
  arm_group.stop();
  arm_group.clearPoseTargets();

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Execute %s FAILED, code=%d", name.c_str(), (int)exec_result.val);
    return false;
  }

  RCLCPP_INFO(logger, "%s done.", name.c_str());
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 关键：要有 executor spin，不然 MoveIt 的 action/service 通信经常收不到回调
  auto node = rclcpp::Node::make_shared("panda_pick_node");
  auto logger = node->get_logger();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // MoveGroup 接口
  moveit::planning_interface::MoveGroupInterface arm_group(node, "panda_arm");
  moveit::planning_interface::MoveGroupInterface hand_group(node, "hand");

  arm_group.setPlanningTime(5.0);
  arm_group.setNumPlanningAttempts(10);
  arm_group.setMaxVelocityScalingFactor(0.2);
  arm_group.setMaxAccelerationScalingFactor(0.2);

  // 等待一会儿，让 joint_states / TF 进来，否则 getCurrentPose 可能拿不到真实值
  rclcpp::sleep_for(std::chrono::milliseconds(800));

  // 1) 张开夹爪
  RCLCPP_INFO(logger, "Step1: open gripper");
  openGripper(hand_group, logger);

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // 2) 读当前末端 pose，然后往下走 5cm
  RCLCPP_INFO(logger, "Step2: get current pose");
  auto current = arm_group.getCurrentPose();   // MoveIt 认为的当前末端位姿
  auto target_pose = current.pose;
  RCLCPP_INFO(logger, "Current z=%.3f", target_pose.position.z);

  target_pose.position.z -= 0.05;  // 下移 5cm
  RCLCPP_INFO(logger, "Target  z=%.3f", target_pose.position.z);

  if (!planAndExecutePose(arm_group, target_pose, "move_down_5cm", logger))
  {
    RCLCPP_ERROR(logger, "move_down failed.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // 3) 合上夹爪
  RCLCPP_INFO(logger, "Step3: close gripper");
  closeGripper(hand_group, logger);

  RCLCPP_INFO(logger, "Done.");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <thread>

static void openGripper(moveit::planning_interface::MoveGroupInterface &hand_group,
                        const rclcpp::Logger &logger)
{
  auto joint_goal = hand_group.getCurrentJointValues();
  if (joint_goal.size() >= 2)
  {
    joint_goal[0] = 0.04;
    joint_goal[1] = 0.04;
    hand_group.setJointValueTarget(joint_goal);
    auto ec = hand_group.move();
    RCLCPP_INFO(logger, "Gripper open move() result = %d", (int)ec.val);
  }
  else
  {
    RCLCPP_WARN(logger, "Hand group joint size < 2, cannot open gripper.");
  }
}

static void closeGripper(moveit::planning_interface::MoveGroupInterface &hand_group,
                         const rclcpp::Logger &logger)
{
  auto joint_goal = hand_group.getCurrentJointValues();
  if (joint_goal.size() >= 2)
  {
    joint_goal[0] = 0.0;
    joint_goal[1] = 0.0;
    hand_group.setJointValueTarget(joint_goal);
    auto ec = hand_group.move();
    RCLCPP_INFO(logger, "Gripper close move() result = %d", (int)ec.val);
  }
  else
  {
    RCLCPP_WARN(logger, "Hand group joint size < 2, cannot close gripper.");
  }
}

static bool planAndExecutePose(moveit::planning_interface::MoveGroupInterface &arm_group,
                               const geometry_msgs::msg::Pose &target_pose,
                               const std::string &name,
                               const rclcpp::Logger &logger)
{
  RCLCPP_INFO(logger, "Planning to %s ...", name.c_str());

  arm_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = arm_group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Plan to %s FAILED, code=%d", name.c_str(), (int)plan_result.val);
    arm_group.clearPoseTargets();
    return false;
  }

  RCLCPP_INFO(logger, "Executing %s ...", name.c_str());
  auto exec_result = arm_group.execute(plan);
  arm_group.stop();
  arm_group.clearPoseTargets();

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Execute %s FAILED, code=%d", name.c_str(), (int)exec_result.val);
    return false;
  }

  RCLCPP_INFO(logger, "%s done.", name.c_str());
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 关键：要有 executor spin，不然 MoveIt 的 action/service 通信经常收不到回调
  auto node = rclcpp::Node::make_shared("panda_pick_node");
  auto logger = node->get_logger();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // MoveGroup 接口
  moveit::planning_interface::MoveGroupInterface arm_group(node, "panda_arm");
  moveit::planning_interface::MoveGroupInterface hand_group(node, "hand");

  arm_group.setPlanningTime(5.0);
  arm_group.setNumPlanningAttempts(10);
  arm_group.setMaxVelocityScalingFactor(0.2);
  arm_group.setMaxAccelerationScalingFactor(0.2);

  // 等待一会儿，让 joint_states / TF 进来，否则 getCurrentPose 可能拿不到真实值
  rclcpp::sleep_for(std::chrono::milliseconds(800));

  // 1) 张开夹爪
  RCLCPP_INFO(logger, "Step1: open gripper");
  openGripper(hand_group, logger);

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // 2) 读当前末端 pose，然后往下走 5cm
  RCLCPP_INFO(logger, "Step2: get current pose");
  auto current = arm_group.getCurrentPose();   // MoveIt 认为的当前末端位姿
  auto target_pose = current.pose;
  RCLCPP_INFO(logger, "Current z=%.3f", target_pose.position.z);

  target_pose.position.z -= 0.05;  // 下移 5cm
  RCLCPP_INFO(logger, "Target  z=%.3f", target_pose.position.z);

  if (!planAndExecutePose(arm_group, target_pose, "move_down_5cm", logger))
  {
    RCLCPP_ERROR(logger, "move_down failed.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // 3) 合上夹爪
  RCLCPP_INFO(logger, "Step3: close gripper");
  closeGripper(hand_group, logger);

  RCLCPP_INFO(logger, "Done.");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}