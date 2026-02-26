#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <vector>
#include <thread>

// 导入自定义的视觉服务接口
#include "my_robot_interfaces/srv/get_block_pose.hpp"

using GetBlockPose = my_robot_interfaces::srv::GetBlockPose;

// --- 全局物理参数配置 ---
const double GRIPPER_OPEN = 0.04;    // 夹爪打开宽度 (米)
const double GRIPPER_CLOSE = 0.01;   // 夹爪闭合宽度 (米)
// 装配区的原点坐标（根据你的实际桌面位置设置）
const Eigen::Vector3d ASSEMBLY_CENTER(0.45, -0.20, 0.02); 

struct TaskTemplate {
    std::string name;
    geometry_msgs::msg::Pose yaml_place_offset; // YAML中定义的放置偏移
};

// ================= 1. 简单的夹爪控制函数 =================
// 纯模拟方案下，通过控制关节值来模拟开关爪
void control_gripper(moveit::planning_interface::MoveGroupInterface& hand, double width) {
    hand.setJointValueTarget("panda_finger_joint1", width);
    hand.setJointValueTarget("panda_finger_joint2", width);
    hand.move();
}

// ================= 2. 核心：调用视觉纠偏服务 =================
geometry_msgs::msg::Pose call_vision_service(rclcpp::Node::SharedPtr node, std::string block_name) {
    auto client = node->create_client<GetBlockPose>("get_block_pose");
    
    // 等待视觉节点上线
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) return geometry_msgs::msg::Pose();
        RCLCPP_INFO(node->get_logger(), "正在等待 vision_node 提供服务...");
    }

    auto request = std::make_shared<GetBlockPose::Request>();
    request->block_name = block_name;

    auto result = client->async_send_request(request);
    
    // 阻塞等待视觉节点运行 7 秒并返回结果
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->success) {
            return result.get()->real_pose;
        }
    }
    
    RCLCPP_ERROR(node->get_logger(), "视觉识别失败或服务无响应: %s", block_name.c_str());
    return geometry_msgs::msg::Pose();
}

// ================= 3. 单个积木的完整执行逻辑 =================
void execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         moveit::planning_interface::MoveGroupInterface& hand,
                         const TaskTemplate& temp) {

    RCLCPP_INFO(node->get_logger(), "------------------------------------");
    RCLCPP_INFO(node->get_logger(), "开始处理积木: %s", temp.name.c_str());

    // --- STEP 1: 视觉定位 ---
    geometry_msgs::msg::Pose real_pick_pose = call_vision_service(node, temp.name);
    if (real_pick_pose.position.x == 0.0) return; // 识别失败则跳过

    // --- STEP 2: 抓取流程 ---
    // 移动到抓取点上方 15cm (预抓取点)
    geometry_msgs::msg::Pose pre_pick = real_pick_pose;
    pre_pick.position.z += 0.15;
    arm.setPoseTarget(pre_pick);
    arm.setMaxVelocityScalingFactor(0.4); // 设置较快速度
    arm.move();

    control_gripper(hand, GRIPPER_OPEN); // 开爪

    // 下降到真实抓取点
    arm.setPoseTarget(real_pick_pose);
    arm.setMaxVelocityScalingFactor(0.1); // 下降动作慢一点，保证安全
    arm.move();

    control_gripper(hand, GRIPPER_CLOSE); // 关爪

    // 抓取后抬起 15cm
    arm.setPoseTarget(pre_pick);
    arm.move();

    // --- STEP 3: 分段放置流程 (核心修改) ---
    // 计算最终放置点：装配中心 + YAML偏移
    geometry_msgs::msg::Pose real_place_pose;
    real_place_pose.position.x = ASSEMBLY_CENTER.x() + temp.yaml_place_offset.position.x;
    real_place_pose.position.y = ASSEMBLY_CENTER.y() + temp.yaml_place_offset.position.y;
    real_place_pose.position.z = ASSEMBLY_CENTER.z() + temp.yaml_place_offset.position.z;
    // 旋转姿态使用 YAML 预设的“正姿态”
    real_place_pose.orientation = temp.yaml_place_offset.orientation;

    // A. 移动到放置点上方 15cm (快速接近)
    geometry_msgs::msg::Pose place_stage_1 = real_place_pose;
    place_stage_1.position.z += 0.15;
    arm.setPoseTarget(place_stage_1);
    arm.setMaxVelocityScalingFactor(0.4); 
    arm.move();

    // B. 下降到上方 5cm (中速准备)
    geometry_msgs::msg::Pose place_stage_2 = real_place_pose;
    place_stage_2.position.z += 0.05;
    arm.setPoseTarget(place_stage_2);
    arm.setMaxVelocityScalingFactor(0.15); 
    arm.move();

    // C. 最后 5cm 极慢速下降 (保证插入稳定性)
    RCLCPP_INFO(node->get_logger(), ">>> 触发高精度插入动作：最后 5cm 极慢速下降...");
    arm.setPoseTarget(real_place_pose);
    
    // 设置极低速度系数：0.02 代表只使用 2% 的电机能力
    arm.setMaxVelocityScalingFactor(0.02); 
    arm.setMaxAccelerationScalingFactor(0.01); 
    
    arm.move(); // 执行最后的触碰动作

    // D. 释放与撤回
    control_gripper(hand, GRIPPER_OPEN); // 松开积木

    // 撤回：先慢速离开 3cm，防止带倒积木
    geometry_msgs::msg::Pose retreat = real_place_pose;
    retreat.position.z += 0.03;
    arm.setPoseTarget(retreat);
    arm.move();

    // 恢复正常速度配置，准备下一个任务
    arm.setMaxVelocityScalingFactor(0.4);
    arm.setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(node->get_logger(), "任务 %s 执行完毕。", temp.name.c_str());
}

// ================= 4. 主函数 =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_vision_executor");

    // 多线程执行器：允许 Service 和 MoveGroup 同时运行而互不干扰
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    // 初始化 MoveIt 接口
    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand(node, "hand");

    // 加载任务模板 (请将此处改为你的 tasks.yaml 绝对路径)
    const std::string yaml_path = "/home/aaa/robot/ros2_ws/src/panda_pick/src/tasks.yaml";
    
    // 简化的模板解析 (这里复用了你之前的 load_task_templates 逻辑)
    YAML::Node config = YAML::LoadFile(yaml_path);
    std::vector<TaskTemplate> templates;
    for (const auto& item : config["tasks"]) {
        TaskTemplate t;
        t.name = item["name"].as<std::string>();
        t.yaml_place_offset.position.x = item["place"]["pos"][0].as<double>();
        t.yaml_place_offset.position.y = item["place"]["pos"][1].as<double>();
        t.yaml_place_offset.position.z = item["place"]["pos"][2].as<double>();
        t.yaml_place_offset.orientation.x = item["place"]["orientation"][0].as<double>();
        t.yaml_place_offset.orientation.y = item["place"]["orientation"][1].as<double>();
        t.yaml_place_offset.orientation.z = item["place"]["orientation"][2].as<double>();
        t.yaml_place_offset.orientation.w = item["place"]["orientation"][3].as<double>();
        templates.push_back(t);
    }

    // 循环执行任务列表
    for (const auto& temp : templates) {
        if (!rclcpp::ok()) break;
        execute_single_task(node, arm, hand, temp);
        rclcpp::sleep_for(std::chrono::seconds(2)); // 每个任务间停顿 2s
    }

    rclcpp::shutdown();
    if (executor_thread.joinable()) executor_thread.join();
    return 0;
}