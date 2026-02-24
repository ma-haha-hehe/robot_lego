#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>
#include <thread>
#include <fstream>

// 必须包含以下头文件以处理轨迹时间
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using GraspAction = franka_msgs::action::Grasp;

// 物理参数常量
const double GRIPPER_HEIGHT = 0.103;
const double ARM_VEL_DEFAULT = 0.30;
const double ARM_ACC_DEFAULT = 0.30;

struct Task {
    std::string name;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// ================= YAML 加载逻辑 =================
bool load_active_task(const std::string& file_path, Task& task) {
    if (!std::filesystem::exists(file_path)) return false;
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        task.name = config["name"].as<std::string>();

        auto fill_pose = [](YAML::Node node, geometry_msgs::msg::Pose& pose) {
            pose.position.x = node["pos"][0].as<double>();
            pose.position.y = node["pos"][1].as<double>();
            pose.position.z = node["pos"][2].as<double>();
            pose.orientation.x = node["orientation"][0].as<double>();
            pose.orientation.y = node["orientation"][1].as<double>();
            pose.orientation.z = node["orientation"][2].as<double>();
            pose.orientation.w = node["orientation"][3].as<double>();
        };

        fill_pose(config["pick"], task.pick_pose);
        fill_pose(config["place"], task.place_pose);
        return true;
    } catch (...) { return false; }
}

// ================= 夹爪控制逻辑 =================
bool grasp_with_force(rclcpp::Node::SharedPtr node, double target_width, double force) {
    auto client = rclcpp_action::create_client<GraspAction>(node, "/panda_gripper/grasp");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) return false;

    GraspAction::Goal goal_msg;
    goal_msg.width = target_width;
    goal_msg.speed = 0.05;
    goal_msg.force = force;
    goal_msg.epsilon.inner = 0.05;
    goal_msg.epsilon.outer = 0.05;

    RCLCPP_INFO(node->get_logger(), ">>> Grasping with %.1f N", force);
    (void)client->async_send_goal(goal_msg);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}

void control_gripper(moveit::planning_interface::MoveGroupInterface& hand, double width) {
    hand.setJointValueTarget("panda_finger_joint1", width / 2.0);
    hand.setJointValueTarget("panda_finger_joint2", width / 2.0);
    hand.move();
}

// ================= 核心修复：线性移动函数 =================
bool move_linear(moveit::planning_interface::MoveGroupInterface& arm,
                 double z_delta,
                 double vel_scale,
                 double acc_scale)
{
    // 1. 基础路径点生成
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target = arm.getCurrentPose().pose;
    target.position.z += z_delta;
    waypoints.push_back(target);

    // 2. 计算笛卡尔路径 (仅生成几何点)
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    double fraction = arm.computeCartesianPath(waypoints, 0.005, 0.0, trajectory_msg);

    if (fraction < 0.9) return false;

    // 3. 时间参数化 (使 vel_scale 和 acc_scale 生效)
    // 将消息转为 RobotTrajectory 对象进行处理
    robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), arm.getName());
    rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory_msg);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt, vel_scale, acc_scale);

    if (success) {
        rt.getRobotTrajectoryMsg(trajectory_msg);
        arm.execute(trajectory_msg); // 执行带速度约束的轨迹
        return true;
    }
    return false;
}

// ================= 核心修复：单任务执行流程 =================
void execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         moveit::planning_interface::MoveGroupInterface& hand,
                         const Task& task)
{
    RCLCPP_INFO(node->get_logger(), ">>> Processing Task: %s", task.name.c_str());

    // STEP 1: 移动到抓取点上方并同步旋转姿态 (Rotation Fix)
    geometry_msgs::msg::Pose h_pick = task.pick_pose; 
    h_pick.position.z += 0.15; // 预备高度 15cm
    
    arm.setMaxVelocityScalingFactor(ARM_VEL_DEFAULT);
    arm.setPoseTarget(h_pick);
    arm.move(); // 此时机械臂会旋转到 YAML 指定的 orientation

    // STEP 2: 开爪
    control_gripper(hand, 0.08);

    // STEP 3: 线性下降 (Speed Fix)
    move_linear(arm, -0.15, 0.20, 0.20); // 正常速度下降

    // STEP 4: 抓取并抬起
    grasp_with_force(node, 0.01, 40.0);
    move_linear(arm, 0.15, 0.30, 0.30); // 抬起稍快

    // STEP 5: 移动到放置点上方并调整姿态
    geometry_msgs::msg::Pose h_place = task.place_pose;
    h_place.position.z += 0.15;
    arm.setPoseTarget(h_place);
    arm.move();

    // STEP 6: 分段下降 (演示极慢速度)
    move_linear(arm, -0.12, 0.20, 0.20); // 快速接近
    move_linear(arm, -0.03, 0.02, 0.02); // 极慢触碰桌面

    // STEP 7: 释放
    control_gripper(hand, 0.08);
    move_linear(arm, 0.15, 0.30, 0.30);
}

// ================= 主函数 =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_executor_final");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand(node, "hand");

    const std::string yaml_path = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/active_task.yaml";

    while (rclcpp::ok()) {
        Task current_task;
        if (load_active_task(yaml_path, current_task)) {
            execute_single_task(node, arm, hand, current_task);
            std::filesystem::remove(yaml_path);
            RCLCPP_INFO(node->get_logger(), "Task Success. Waiting for next...");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}