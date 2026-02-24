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
#include <fstream>  // 必须添加这一行来修复编译错误

// 定义 Grasp Action 类型别名
using GraspAction = franka_msgs::action::Grasp;

const double GRIPPER_HEIGHT = 0.103; 

struct Task {
    std::string name;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 1. 读取单任务 YAML (适配图片中的扁平结构) =================
bool load_active_task(const std::string& file_path, Task& task) {
    if (!std::filesystem::exists(file_path)) return false;
    //强制刷新文件缓冲区，确保读取的是最新写入的内容
    std::ifstream fin(file_path);
    if (!fin.is_open()) return false;

    try {
        YAML::Node config = YAML::LoadFile(file_path);
        task.name = config["name"].as<std::string>();

        // 解析 Pick
        auto p_pos = config["pick"]["pos"];
        auto p_ori = config["pick"]["orientation"];
        task.pick_pose.position.x = p_pos[0].as<double>();
        task.pick_pose.position.y = p_pos[1].as<double>();
        task.pick_pose.position.z = p_pos[2].as<double>();
        task.pick_pose.orientation.x = p_ori[0].as<double>();
        task.pick_pose.orientation.y = p_ori[1].as<double>();
        task.pick_pose.orientation.z = p_ori[2].as<double>();
        task.pick_pose.orientation.w = p_ori[3].as<double>();

        // 解析 Place
        auto l_pos = config["place"]["pos"];
        auto l_ori = config["place"]["orientation"];
        task.place_pose.position.x = l_pos[0].as<double>();
        task.place_pose.position.y = l_pos[1].as<double>();
        task.place_pose.position.z = l_pos[2].as<double>();
        task.place_pose.orientation.x = l_ori[0].as<double>();
        task.place_pose.orientation.y = l_ori[1].as<double>();
        task.place_pose.orientation.z = l_ori[2].as<double>();
        task.place_pose.orientation.w = l_ori[3].as<double>();
        return true;
    } catch (...) { return false; }
}

// ================= 2. 持续力抓取函数 (让夹爪持续施压) =================
bool grasp_with_force(rclcpp::Node::SharedPtr node, double target_width, double force) {
    auto client = rclcpp_action::create_client<GraspAction>(node, "/panda_gripper/grasp");
    
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "无法连接到 Grasp Action Server");
        return false;
    }

    auto goal_msg = GraspAction::Goal();
    // target_width 必须小于积木实际宽度以维持压力
    goal_msg.width = target_width; 
    goal_msg.speed = 0.05;         
    goal_msg.force = force;        // 设置持续夹紧力（单位：牛顿）
    goal_msg.epsilon.inner = 0.05; 
    goal_msg.epsilon.outer = 0.05;

    RCLCPP_INFO(node->get_logger(), ">>> 正在执行持续力抓取: %.1f N", force);
    auto send_goal_future = client->async_send_goal(goal_msg);
    // 这里简单等待一秒以确保动作发起
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true; 
}

// ================= 3. 直线运动辅助函数 (Straight Move) =================
bool move_linear(moveit::planning_interface::MoveGroupInterface& arm, double z_delta) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose current_pose = arm.getCurrentPose().pose;
    
    // 设定目标高度
    current_pose.position.z += z_delta;
    waypoints.push_back(current_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    // 计算笛卡尔路径
    double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction >= 0.9) {
        arm.execute(trajectory);
        return true;
    }
    return false;
}

// ================= 4. 辅助夹爪控制 (普通位置控制，用于张开) =================
void control_gripper(moveit::planning_interface::MoveGroupInterface& hand, double width) {
    double pos = width / 2.0; 
    hand.setJointValueTarget("panda_finger_joint1", pos);
    hand.setJointValueTarget("panda_finger_joint2", pos);
    hand.setGoalJointTolerance(0.01);
    hand.move();
}

// ================= 5. 核心执行逻辑 =================
void execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         moveit::planning_interface::MoveGroupInterface& hand,
                         const Task& task)
{
    RCLCPP_INFO(node->get_logger(), ">>> 开始任务: %s", task.name.c_str());

    // --- STEP 1: 移动到抓取点上方 (普通规划) ---
    geometry_msgs::msg::Pose h_pick = task.pick_pose;
    h_pick.position.z += GRIPPER_HEIGHT + 0.15; // 预抓取高度 15cm
    arm.setPoseTarget(h_pick);
    arm.move();

    // --- STEP 2: 张开夹爪 ---
    control_gripper(hand, 0.08); // 张开至 8cm

    // --- STEP 3: 垂直直线下降 ---
    move_linear(arm, -0.15); 

    // --- STEP 4: 持续力抓取并垂直抬起 ---
    // 假设积木宽度约为 0.03m，设置 target 为 0.01m 强制施加压力
    grasp_with_force(node, 0.01, 40.0); 
    move_linear(arm, 0.15); // 直线抬起 15cm

    // --- STEP 5: 移动到放置点上方 ---
    geometry_msgs::msg::Pose h_place = task.place_pose;
    h_place.position.z += GRIPPER_HEIGHT + 0.15;
    arm.setPoseTarget(h_place);
    arm.move();

    // --- STEP 6: 直线下降放置 ---
    move_linear(arm, -0.15);

    // --- STEP 7: 释放并直线撤回 ---
    control_gripper(hand, 0.08);
    move_linear(arm, 0.15); 
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_executor_final");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand(node, "hand"); // 请确保与 SRDF 一致

    std::string yaml_path = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/active_task.yaml";

    while (rclcpp::ok()) {
        Task current_task;
        if (load_active_task(yaml_path, current_task)) {
            execute_single_task(node, arm, hand, current_task);
            std::filesystem::remove(yaml_path);
            RCLCPP_INFO(node->get_logger(), "任务成功完成，已删除 YAML。");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}