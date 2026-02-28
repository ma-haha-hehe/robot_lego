#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>
#include <thread>

// 时间参数化相关头文件
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using GraspAction = franka_msgs::action::Grasp;
using GripperMoveAction = franka_msgs::action::Move;

// ================= 配置区域 =================
const std::string RESULT_FILE = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/active_task.yaml";
const std::string TASKS_YAML = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/tasks.yaml";
const double GRIPPER_HEIGHT = 0.103; 

struct Task {
    std::string name;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 1. YAML 监听逻辑 =================
bool wait_for_any_task(Task& current_task) {
    auto logger = rclcpp::get_logger("yaml_listener");
    while (rclcpp::ok()) {
        if (std::filesystem::exists(RESULT_FILE)) {
            try {
                YAML::Node res = YAML::LoadFile(RESULT_FILE);
                current_task.name = res["name"].as<std::string>();
                auto fill_pose = [](YAML::Node node, geometry_msgs::msg::Pose& pose) {
                    pose.position.x = node["pos"][0].as<double>();
                    pose.position.y = node["pos"][1].as<double>();
                    pose.position.z = node["pos"][2].as<double>();
                    pose.orientation.x = 1.0; pose.orientation.y = 0.0;
                    pose.orientation.z = 0.0; pose.orientation.w = 0.0;
                };
                fill_pose(res["pick"], current_task.pick_pose);
                fill_pose(res["place"], current_task.place_pose);
                return true;
            } catch (...) {}
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
}

// ================= 2. 持续力抓取 (Grasp Action) =================
bool grasp_with_force(rclcpp::Node::SharedPtr node, double target_width, double force) {
    auto client = rclcpp_action::create_client<GraspAction>(node, "/panda_gripper/grasp");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) return false;

    GraspAction::Goal goal_msg;
    goal_msg.width = target_width; // 设置为比物体窄的值以维持压力
    goal_msg.speed = 0.05;
    goal_msg.force = force;        // 持续夹紧力 (N)
    goal_msg.epsilon.inner = 0.05;
    goal_msg.epsilon.outer = 0.05;

    RCLCPP_INFO(node->get_logger(), ">>> 执行持续力抓取: %.1f N", force);
    auto future = client->async_send_goal(goal_msg);
    rclcpp::sleep_for(std::chrono::seconds(1)); // 给予硬件反应时间
    return true;
}

// 释放逻辑
bool release_gripper(rclcpp::Node::SharedPtr node, double width) {
    auto client = rclcpp_action::create_client<GripperMoveAction>(node, "/panda_gripper/move");
    if (!client->wait_for_action_server(std::chrono::seconds(2))) return false;
    GripperMoveAction::Goal goal_msg;
    goal_msg.width = width;
    goal_msg.speed = 0.1;
    client->async_send_goal(goal_msg);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}

// ================= 3. 线性移动函数 (含速度缩放) =================
bool move_linear(moveit::planning_interface::MoveGroupInterface& arm, 
                 double z_delta, double vel_scale, double acc_scale) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target = arm.getCurrentPose().pose;
    target.position.z += z_delta;
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    double fraction = arm.computeCartesianPath(waypoints, 0.005, 0.0, trajectory_msg);
    if (fraction < 0.9) return false;

    // 时间参数化：让速度缩放对直线运动生效
    robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), arm.getName());
    rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if (iptp.computeTimeStamps(rt, vel_scale, acc_scale)) {
        rt.getRobotTrajectoryMsg(trajectory_msg);
        arm.execute(trajectory_msg);
        return true;
    }
    return false;
}

// ================= 4. 单个积木执行逻辑 (修复返回值) =================
bool execute_single_task(rclcpp::Node::SharedPtr node,
    moveit::planning_interface::MoveGroupInterface& arm,
                         moveit::planning_interface::MoveGroupInterface& hand,
                         const Task& task) {
    RCLCPP_INFO(node->get_logger(), "### 正在处理积木: %s ###", task.name.c_str());
    geometry_msgs::msg::Pose h_pick = task.pick_pose;
    h_pick.position.z += GRIPPER_HEIGHT + 0.15;
    arm.setPoseTarget(h_pick);
    arm.move();
    hand.setJointValueTarget("panda_finger_joint1", 0.04);
    hand.setJointValueTarget("panda_finger_joint2", 0.04);
    hand.move();

    move_linear(arm, -0.15, 0.2, 0.2);
    grasp_with_force(node, 0.01, 40.0);
    //稳固等待
    RCLCPP_INFO(node->get_logger(), "抓取动作已经发送，等待2秒确保稳固");
    rclcpp::sleep_for(std::chrono::seconds(2));
    //执行抬起动作
    move_linear(arm, 0.15, 0.3, 0.3);
    geometry_msgs::msg::Pose h_place = task.place_pose;
    h_place.position.z += GRIPPER_HEIGHT + 0.15;
    arm.setPoseTarget(h_place);
    arm.move();
    move_linear(arm, -0.11,0.2,0.2);
    move_linear(arm, -0.04,0.02, 0.02);

    // --- 核心修复点：替换 hand.move() ---
    RCLCPP_INFO(node->get_logger(), ">>> 执行释放动作...");
    
    // 直接使用 Action 控制硬件，无视 MoveIt 的规划限制
    release_gripper(node, 0.08);
    move_linear(arm, 0.15, 0.3, 0.3);
    return true;
}


// ================= 5. MAIN =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_batch_executor");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand(node, "hand");
    moveit::planning_interface::PlanningSceneInterface psi;

    // --- 核心：取消碰撞相关的规划限制 ---
    arm.setPlanningTime(15.0);           // 增加规划时间，应对复杂路径
    arm.setNumPlanningAttempts(10);      // 增加尝试次数
    arm.setGoalPositionTolerance(0.01);  // 增加 1cm 容忍度
    arm.setGoalOrientationTolerance(0.1); // 增加姿态容忍度

    // --- 关键修改：清空所有场景物体以取消碰撞检测 ---
    RCLCPP_INFO(node->get_logger(), "⚠️ 正在清空规划场景，取消地面碰撞限制...");
    std::vector<std::string> object_ids = psi.getKnownObjectNames();
    psi.removeCollisionObjects(object_ids); 

    RCLCPP_INFO(node->get_logger(), ">>> 监听开始，等待视觉文件...");

    while (rclcpp::ok()) {
        Task current_task;
        if (wait_for_any_task(current_task)) {
            // 执行动作并捕获返回值
            bool success = execute_single_task(node, arm, hand, current_task);

            if (std::filesystem::exists(RESULT_FILE)) {
                std::filesystem::remove(RESULT_FILE);
                RCLCPP_INFO(node->get_logger(), "🎊 任务处理完毕，已清理信号。");
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}here is my cpp node, i want it no [ERROR] [1772300486.471161832] [move_group_interface]: MoveGroupInterface::move() failed or timeout reached , even it failed it go back to initial pose give me these code in copy paste view, no change 