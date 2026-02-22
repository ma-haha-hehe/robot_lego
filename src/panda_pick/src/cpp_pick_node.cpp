#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 必须包含
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <thread>

const double GRIPPER_HEIGHT = 0.103; 

struct Task {
    int id;
    std::string name;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// --- 1. 正确读取 active_task.yaml (包含四元数) ---
bool load_active_step(const std::string& path, Task& task) {
    if (!std::filesystem::exists(path)) return false;
    try {
        YAML::Node t = YAML::LoadFile(path);
        task.id = t["id"].as<int>();
        task.name = t["name"].as<std::string>();
        
        // 解析 Pick 坐标和方向
        auto p_pos = t["pick"]["pos"];
        task.pick_pose.position.x = p_pos[0].as<double>();
        task.pick_pose.position.y = p_pos[1].as<double>();
        task.pick_pose.position.z = p_pos[2].as<double>();

        auto p_ori = t["pick"]["orientation"];
        task.pick_pose.orientation.x = p_ori[0].as<double>();
        task.pick_pose.orientation.y = p_ori[1].as<double>();
        task.pick_pose.orientation.z = p_ori[2].as<double>();
        task.pick_pose.orientation.w = p_ori[3].as<double>();
        
        // 解析 Place 坐标和方向
        auto l_pos = t["place"]["pos"];
        task.place_pose.position.x = l_pos[0].as<double>();
        task.place_pose.position.y = l_pos[1].as<double>();
        task.place_pose.position.z = l_pos[2].as<double>();

        auto l_ori = t["place"]["orientation"];
        task.place_pose.orientation.x = l_ori[0].as<double>();
        task.place_pose.orientation.y = l_ori[1].as<double>();
        task.place_pose.orientation.z = l_ori[2].as<double>();
        task.place_pose.orientation.w = l_ori[3].as<double>();

        return true;
    } catch (...) { return false; }
}

// --- 2. 增强版移动函数 (增加重试和起始状态设置) ---
bool move_to(moveit::planning_interface::MoveGroupInterface& group, geometry_msgs::msg::Pose target) {
    group.setStartStateToCurrentState(); // 强制从当前实际位置开始规划
    group.setPoseTarget(target);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        group.execute(plan);
        return true;
    }
    return false;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_moveit_executor");
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand(node, "hand");
    moveit::planning_interface::PlanningSceneInterface psi;

    // 设置运动约束：限制速度以防真机动作过猛
    arm.setMaxVelocityScalingFactor(0.2);
    arm.setMaxAccelerationScalingFactor(0.2);

    std::string active_path = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/active_task.yaml";

    // --- 3. 环境碰撞建模 ---
    // 注意：如果机器人直接在桌面上，Z=0。你代码中 table.z = 0.38 意味着机器人底座在 0.4 处
    moveit_msgs::msg::CollisionObject table;
    table.id = "work_table";
    table.header.frame_id = "world";
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {1.5, 1.5, 0.02}; 
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.z = 0.38; // 保持你原有的 0.38 设置
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    psi.applyCollisionObject(table);

    while (rclcpp::ok()) {
        Task current_task;
        if (load_active_step(active_path, current_task)) {
            RCLCPP_INFO(node->get_logger(), "执行任务: %s", current_task.name.c_str());

            // --- 4. 核心：从上往下的运动流 ---
            
            // 步骤 A: 预抓取 (Hover) - 保持 YAML 中的朝下姿态，但在 $Z$ 轴抬高 15cm
            geometry_msgs::msg::Pose pre_pick = current_task.pick_pose;
            pre_pick.position.z += GRIPPER_HEIGHT + 0.15; 
            hand.setNamedTarget("open"); hand.move();
            move_to(arm, pre_pick);

            // 步骤 B: 垂直下降抓取
            geometry_msgs::msg::Pose pick_pos = current_task.pick_pose;
            pick_pos.position.z += GRIPPER_HEIGHT;
            move_to(arm, pick_pos);
            
            // 步骤 C: 闭合并建立逻辑连接
            hand.setNamedTarget("close"); hand.move();
            arm.attachObject(current_task.name, "panda_link8");
            
            // 步骤 D: 抬起积木 (Lift)
            move_to(arm, pre_pick);

            // 步骤 E: 移动到放置点上方
            geometry_msgs::msg::Pose pre_place = current_task.place_pose;
            pre_place.position.z += GRIPPER_HEIGHT + 0.15;
            move_to(arm, pre_place);

            // 步骤 F: 放置下降
            geometry_msgs::msg::Pose place_pos = current_task.place_pose;
            place_pos.position.z += GRIPPER_HEIGHT;
            move_to(arm, place_pos);

            // 步骤 G: 释放
            hand.setNamedTarget("open"); hand.move();
            arm.detachObject(current_task.name);
            
            // 步骤 H: 撤回
            move_to(arm, pre_place);

            std::filesystem::remove(active_path);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}