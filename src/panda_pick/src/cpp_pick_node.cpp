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
#include <franka_msgs/action/move.hpp>
using GripperMoveAction = franka_msgs::action::Move;

// 时间参数化相关头文件，确保速度缩放生效
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using GraspAction = franka_msgs::action::Grasp;

// ================= 配置区域 =================
const std::string RESULT_FILE = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/active_task.yaml";
const std::string TASKS_YAML = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/tasks.yaml";
const double GRIPPER_HEIGHT = 0.103; // 夹爪法兰到指尖的距离
const double ARM_VEL_DEFAULT = 0.4;
const double ARM_ACC_DEFAULT = 0.3;

struct Task {
    std::string name;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 1. YAML 监听与解析逻辑 =================
// 核心改动：不再检查名称，只要发现文件就解析并返回
bool wait_for_any_task(Task& current_task) {
    auto logger = rclcpp::get_logger("yaml_listener");
    while (rclcpp::ok()) {
        if (std::filesystem::exists(RESULT_FILE)) {
            try {
                YAML::Node res = YAML::LoadFile(RESULT_FILE);
                current_task.name = res["name"].as<std::string>();

                // 解析位姿并强制设定夹爪向下 (Quaternion: 1,0,0,0)
                auto fill_pose = [](YAML::Node node, geometry_msgs::msg::Pose& pose) {
                    pose.position.x = node["pos"][0].as<double>();
                    pose.position.y = node["pos"][1].as<double>();
                    pose.position.z = node["pos"][2].as<double>();
                    // 强制姿态向下
                    pose.orientation.x = 1.0;
                    pose.orientation.y = 0.0;
                    pose.orientation.z = 0.0;
                    pose.orientation.w = 0.0;
                };

                fill_pose(res["pick"], current_task.pick_pose);
                fill_pose(res["place"], current_task.place_pose);
                RCLCPP_INFO(logger, "✅ 发现任务文件 [%s]，开始执行...", current_task.name.c_str());
                return true;
            } catch (const std::exception& e) {
                // 文件可能正在写入，忽略并重试
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
}

// ================= 场景构建函数 (保持原样) =================
void setup_planning_scene(moveit::planning_interface::PlanningSceneInterface& psi) {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    moveit_msgs::msg::CollisionObject ground;
    ground.id = "ground";
    ground.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive ground_prim;
    ground_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    ground_prim.dimensions = {2.0, 2.0, 0.01};
    geometry_msgs::msg::Pose ground_pose;
    ground_pose.position.z = -0.005;
    ground.primitives.push_back(ground_prim);
    ground.primitive_poses.push_back(ground_pose);
    ground.operation = ground.ADD;
    collision_objects.push_back(ground);
    psi.applyCollisionObjects(collision_objects);
}

// ================= 2. 持续力抓取 (保持原样) =================
bool grasp_with_force(rclcpp::Node::SharedPtr node, double target_width, double force) {
    auto client = rclcpp_action::create_client<GraspAction>(node, "/panda_gripper/grasp");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) return false;
    GraspAction::Goal goal_msg;
    goal_msg.width = target_width;
    goal_msg.speed = 0.05;
    goal_msg.force = force;
    goal_msg.epsilon.inner = 0.05;
    goal_msg.epsilon.outer = 0.05;
    RCLCPP_INFO(node->get_logger(), ">>> 执行持续力抓取: %.1f N", force);
    auto future = client->async_send_goal(goal_msg);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}
// ================= 新增：使用 Action 强制释放夹爪 =================
bool release_gripper(rclcpp::Node::SharedPtr node, double width) {
    auto client = rclcpp_action::create_client<GripperMoveAction>(node, "/panda_gripper/move");
    
    if (!client->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node->get_logger(), "无法连接到夹爪 Move Action 服务器");
        return false;
    }

    auto goal_msg = GripperMoveAction::Goal();
    goal_msg.width = width;
    goal_msg.speed = 0.1;

    RCLCPP_INFO(node->get_logger(), ">>> 正在通过 Action 释放夹爪 (宽度: %.2f)...", width);
    auto future = client->async_send_goal(goal_msg);
    
    // 强制物理等待，确保夹爪动作完成
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}

// ================= 3. 线性移动函数 (保持原样) =================
bool move_linear(moveit::planning_interface::MoveGroupInterface& arm, 
                 double z_delta, double vel_scale, double acc_scale) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target = arm.getCurrentPose().pose;
    target.position.z += z_delta;
    waypoints.push_back(target);
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    double fraction = arm.computeCartesianPath(waypoints, 0.005, 0.0, trajectory_msg);
    if (fraction < 0.9) return false;
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

// ================= 4. 单个积木执行逻辑 (保持原样) =================
void execute_single_task(rclcpp::Node::SharedPtr node,
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
}

// ================= 5. MAIN 主函数 =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_batch_executor");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand(node, "hand");
    moveit::planning_interface::PlanningSceneInterface psi;

    RCLCPP_INFO(node->get_logger(), "正在初始化规划场景...");
    setup_planning_scene(psi);

    // 获取任务总数
    int total_tasks = 0;
    try {
        YAML::Node config = YAML::LoadFile(TASKS_YAML);
        total_tasks = config["tasks"].size();
    } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "无法加载 tasks.yaml 确定任务总数。");
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), ">>> 准备就绪。循环执行 %d 个任务...", total_tasks);

    for (int i = 0; i < total_tasks; ++i) {
        if (!rclcpp::ok()) break;

        Task current_task;
        // 1. 等待视觉节点写出 active_task.yaml
        if (wait_for_any_task(current_task)) {
            RCLCPP_INFO(node->get_logger(), "[任务进度 %d/%d]", i + 1, total_tasks);
            
            // 2. 执行动作
            execute_single_task(node, arm, hand, current_task);

            // 3. 握手清理：动作完成后删除文件，触发 Python 节点识别下一个
            if (std::filesystem::exists(RESULT_FILE)) {
                std::filesystem::remove(RESULT_FILE);
                RCLCPP_INFO(node->get_logger(), "🎊 积木 [%s] 任务完成，清理信号。", current_task.name.c_str());
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    RCLCPP_INFO(node->get_logger(), "### 所有同步任务已完成！ ###");
    rclcpp::shutdown();
    if (executor_thread.joinable()) executor_thread.join();
    return 0;
}