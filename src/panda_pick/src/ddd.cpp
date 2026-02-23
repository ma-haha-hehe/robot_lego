#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>

const double GRIPPER_HEIGHT = 0.103; 

struct Task {
    std::string name;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 1. 读取单任务 YAML =================
bool load_active_task(const std::string& file_path, Task& task) {
    if (!std::filesystem::exists(file_path)) return false;
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        task.name = config["name"].as<std::string>();

        auto p_pos = config["pick"]["pos"];
        auto p_ori = config["pick"]["orientation"];
        task.pick_pose.position.x = p_pos[0].as<double>();
        task.pick_pose.position.y = p_pos[1].as<double>();
        task.pick_pose.position.z = p_pos[2].as<double>();
        task.pick_pose.orientation.x = p_ori[0].as<double>();
        task.pick_pose.orientation.y = p_ori[1].as<double>();
        task.pick_pose.orientation.z = p_ori[2].as<double>();
        task.pick_pose.orientation.w = p_ori[3].as<double>();

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

// ================= 2. 直线运动辅助函数 (Straight Move) =================
bool move_linear(moveit::planning_interface::MoveGroupInterface& arm, double z_delta, double fraction_threshold = 0.9) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose current_pose = arm.getCurrentPose().pose;
    
    // 设定目标高度
    current_pose.position.z += z_delta;
    waypoints.push_back(current_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    // 计算笛卡尔路径
    double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction >= fraction_threshold) {
        arm.execute(trajectory);
        return true;
    }
    return false;
}

// ================= 3. 夹爪控制辅助函数 =================
void control_gripper(moveit::planning_interface::MoveGroupInterface& hand, double width) {
    // 单侧手指位置为总宽度的一半
    double pos = width / 2.0; 
    hand.setJointValueTarget("panda_finger_joint1", pos);
    hand.setJointValueTarget("panda_finger_joint2", pos);
    hand.setGoalJointTolerance(0.01); // 容忍微小误差，防止因夹住物体报错
    hand.move();
}

// ================= 4. 执行逻辑 =================
void execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         moveit::planning_interface::MoveGroupInterface& hand,
                         const Task& task)
{
    RCLCPP_INFO(node->get_logger(), ">>> 开始任务: %s", task.name.c_str());

    // --- STEP 1: 移动到抓取点上方 (普通规划) ---
    geometry_msgs::msg::Pose h_pick = task.pick_pose;
    h_pick.position.z += GRIPPER_HEIGHT + 0.1;
    arm.setPoseTarget(h_pick);
    arm.move();

    // --- STEP 2: 张开夹爪 ---
    control_gripper(hand, 0.08); // 张开至 8cm

    // --- STEP 3: 垂直下降 (直线运动) ---
    // 移动距离 = 负的悬停高度 (即下降 10cm)
    move_linear(arm, -0.1); 

    // --- STEP 4: 闭合夹爪并垂直抬起 ---
    control_gripper(hand, 0.01); // 尝试闭合至 1cm（会自动因夹住物体停止）
    move_linear(arm, 0.1); // 垂直抬起 10cm

    // --- STEP 5: 移动到放置点上方 (普通规划) ---
    geometry_msgs::msg::Pose h_place = task.place_pose;
    h_place.position.z += GRIPPER_HEIGHT + 0.1;
    arm.setPoseTarget(h_place);
    arm.move();

    // --- STEP 6: 垂直下降放置 (直线运动) ---
    // 计算放置点的下降距离
    double place_drop = task.place_pose.position.z - (h_place.position.z - GRIPPER_HEIGHT);
    move_linear(arm, place_drop);

    // --- STEP 7: 释放并垂直撤回 ---
    control_gripper(hand, 0.08);
    move_linear(arm, 0.1); // 撤回 10cm
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_executor_linear");

    // 多线程执行器必不可少
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    // 关键：确保组名与你实机一致，通常是 hand
    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand(node, "hand");

    std::string yaml_path = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/active_task.yaml";

    while (rclcpp::ok()) {
        Task current_task;
        if (load_active_task(yaml_path, current_task)) {
            execute_single_task(node, arm, hand, current_task);
            std::filesystem::remove(yaml_path);
            RCLCPP_INFO(node->get_logger(), "任务完成，等待下一个 YAML...");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}