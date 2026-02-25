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

// 导入你的自定义服务接口
#include "my_robot_interfaces/srv/get_block_pose.hpp"

using GraspAction = franka_msgs::action::Grasp;
using GetBlockPose = my_robot_interfaces::srv::GetBlockPose;

// 物理参数常量
const double GRIPPER_HEIGHT = 0.103; 
// --- 核心业务参数 ---
const Eigen::Vector3d ASSEMBLY_CENTER(0.45, -0.20, 0.02); // 你的装配中心原点

struct TaskTemplate {
    std::string name;
    geometry_msgs::msg::Pose yaml_pick_offset;  // YAML里相对于物体的偏移
    geometry_msgs::msg::Pose yaml_place_offset; // YAML里相对于装配中心的偏移
};

// ================= 1. 加载任务模板 (从 YAML 读取相对偏移) =================
std::vector<TaskTemplate> load_task_templates(const std::string& file_path) {
    std::vector<TaskTemplate> templates;
    YAML::Node config = YAML::LoadFile(file_path);
    for (const auto& item : config["tasks"]) {
        TaskTemplate t;
        t.name = item["name"].as<std::string>();
        
        auto fill_pose = [](YAML::Node node, geometry_msgs::msg::Pose& p) {
            p.position.x = node["pos"][0].as<double>();
            p.position.y = node["pos"][1].as<double>();
            p.position.z = node["pos"][2].as<double>();
            p.orientation.x = node["orientation"][0].as<double>();
            p.orientation.y = node["orientation"][1].as<double>();
            p.orientation.z = node["orientation"][2].as<double>();
            p.orientation.w = node["orientation"][3].as<double>();
        };
        fill_pose(item["pick"], t.yaml_pick_offset);
        fill_pose(item["place"], t.yaml_place_offset);
        templates.push_back(t);
    }
    return templates;
}

// ================= 2. 核心：调用视觉服务获取真实抓取点 =================
geometry_msgs::msg::Pose call_vision_service(rclcpp::Node::SharedPtr node, std::string block_name) {
    auto client = node->create_client<GetBlockPose>("get_block_pose");
    
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "等待视觉服务中...");
    }

    auto request = std::make_shared<GetBlockPose::Request>();
    request->block_name = block_name;

    auto result = client->async_send_request(request);
    
    // 等待视觉节点运行那 7 秒钟
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->success) {
            return result.get()->real_pose;
        }
    }
    
    RCLCPP_ERROR(node->get_logger(), "视觉识别失败: %s", block_name.c_str());
    return geometry_msgs::msg::Pose(); // 返回空位姿（实际应加入异常处理）
}

// ================= 3. 执行逻辑 (单个积木) =================
void execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         moveit::planning_interface::MoveGroupInterface& hand,
                         const TaskTemplate& temp) {
    
    // A. 视觉纠偏：获取真实抓取点
    RCLCPP_INFO(node->get_logger(), ">>> 正在请求视觉纠偏: %s", temp.name.c_str());
    geometry_msgs::msg::Pose real_pick_pose = call_vision_service(node, temp.name);

    // B. 计算真实放置点 (装配中心 + YAML偏移)
    geometry_msgs::msg::Pose real_place_pose;
    real_place_pose.position.x = ASSEMBLY_CENTER.x() + temp.yaml_place_offset.position.x;
    real_place_pose.position.y = ASSEMBLY_CENTER.y() + temp.yaml_place_offset.position.y;
    real_place_pose.position.z = ASSEMBLY_CENTER.z() + temp.yaml_place_offset.position.z;
    // 姿态保持与抓取时一致（确保垂直）
    real_place_pose.orientation = real_pick_pose.orientation;

    // --- 开始 MoveIt 动作流水线 ---

    // STEP 1: 预抓取 (上方 15cm)
    geometry_msgs::msg::Pose h_pick = real_pick_pose;
    h_pick.position.z += 0.15; 
    arm.setPoseTarget(h_pick);
    arm.move();

    // STEP 2: 开爪
    // (此处调用你原来的 hand.move() 逻辑)

    // STEP 3: 下降并抓取
    // (此处调用你原来的 grasp_with_force 逻辑)

    // STEP 4: 抬起并前往放置点上方
    geometry_msgs::msg::Pose h_place = real_place_pose;
    h_place.position.z += 0.15;
    arm.setPoseTarget(h_place);
    arm.move();

    // STEP 5: 下降并释放
    // (此处调用你原来的释放逻辑)
    RCLCPP_INFO(node->get_logger(), "任务完成: %s", temp.name.c_str());
}

// ================= 主函数 =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_vision_executor");

    // 必须要用多线程执行器，否则 Service 回调会阻塞主线程
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand(node, "hand");

    // 1. 加载 YAML 模板（这里存的是相对偏移）
    const std::string yaml_path = ".../tasks.yaml";
    std::vector<TaskTemplate> templates = load_task_templates(yaml_path);

    // 2. 循环执行任务
    for (const auto& temp : templates) {
        execute_single_task(node, arm, hand, temp);
        rclcpp::sleep_for(std::chrono::seconds(2));
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}
