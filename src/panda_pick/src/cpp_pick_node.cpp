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
const double GRIPPER_HEIGHT = 0.103; 

struct Task {
    std::string name;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 辅助函数：安全返回初始位姿 =================
bool go_home(moveit::planning_interface::MoveGroupInterface& arm) {
    RCLCPP_INFO(rclcpp::get_logger("executor"), "🔄 任务受阻，正在返回初始位姿 (Ready) 以便重新规划...");
    arm.setNamedTarget("ready"); 
    auto result = arm.move();
    return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

// ================= 1. YAML 监听逻辑 =================
bool wait_for_any_task(Task& current_task) {
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
            } catch (...) {
                // 文件可能正在写入，稍后重试
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
}

// ================= 2. 夹爪控制 =================
bool grasp_with_force(rclcpp::Node::SharedPtr node, double target_width, double force) {
    auto client = rclcpp_action::create_client<GraspAction>(node, "/panda_gripper/grasp");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) return false;
    GraspAction::Goal goal;
    goal.width = target_width; goal.force = force; goal.speed = 0.05;
    goal.epsilon.inner = 0.05; goal.epsilon.outer = 0.05;
    client->async_send_goal(goal);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}

bool release_gripper(rclcpp::Node::SharedPtr node, double width) {
    auto client = rclcpp_action::create_client<GripperMoveAction>(node, "/panda_gripper/move");
    if (!client->wait_for_action_server(std::chrono::seconds(2))) return false;
    GripperMoveAction::Goal goal;
    goal.width = width; goal.speed = 0.1;
    client->async_send_goal(goal);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}

// ================= 3. 线性移动 (带失败检查) =================
bool move_linear_safe(moveit::planning_interface::MoveGroupInterface& arm, double z_delta) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target = arm.getCurrentPose().pose;
    target.position.z += z_delta;
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, traj);
    if (fraction < 0.9) return false;

    auto res = arm.execute(traj);
    return (res == moveit::core::MoveItErrorCode::SUCCESS);
}

// ================= 4. 执行逻辑 (含重试准备) =================
bool execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         const Task& task) {
    RCLCPP_INFO(node->get_logger(), "🚀 执行任务: %s", task.name.c_str());
    arm.setStartStateToCurrentState();

    // 定义错误处理闭包
    auto on_failure = [&](const std::string& msg) {
        RCLCPP_ERROR(node->get_logger(), "❌ %s，准备重试...", msg.c_str());
        go_home(arm);
        return false;
    };

    // STEP 1: 预抓取
    geometry_msgs::msg::Pose h_pick = task.pick_pose;
    h_pick.position.z += GRIPPER_HEIGHT + 0.15;
    arm.setPoseTarget(h_pick);
    if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) return on_failure("预抓取规划失败");

    // STEP 2: 下降并抓取
    release_gripper(node, 0.08);
    if (!move_linear_safe(arm, -0.15)) return on_failure("线性下降失败");
    grasp_with_force(node, 0.01, 40.0);
    rclcpp::sleep_for(std::chrono::seconds(2));

    // STEP 3: 抬起并前往放置点
    if (!move_linear_safe(arm, 0.15)) return on_failure("抬起动作失败");
    
    geometry_msgs::msg::Pose h_place = task.place_pose;
    h_place.position.z += GRIPPER_HEIGHT + 0.15;
    arm.setPoseTarget(h_place);
    if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) return on_failure("移动到放置点失败");

    // STEP 4: 放置并释放
    if (!move_linear_safe(arm, -0.15)) return on_failure("放置下降失败");
    release_gripper(node, 0.08);
    
    // STEP 5: 撤回
    move_linear_safe(arm, 0.15);
    return true; 
}

// ================= 5. MAIN (重试逻辑核心) =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_batch_executor");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::PlanningSceneInterface psi;

    // 优化参数
    arm.setPlanningTime(10.0);
    arm.setGoalPositionTolerance(0.01);

    // 清空场景避免碰撞误报
    std::vector<std::string> object_ids = psi.getKnownObjectNames();
    psi.removeCollisionObjects(object_ids);

    RCLCPP_INFO(node->get_logger(), ">>> 系统就绪，监听视觉信号...");

    while (rclcpp::ok()) {
        Task current_task;
        if (wait_for_any_task(current_task)) {
            // 尝试执行任务
            bool success = execute_single_task(node, arm, current_task);

            if (success) {
                // 只有成功才删除文件，进入下一个任务
                if (std::filesystem::exists(RESULT_FILE)) {
                    std::filesystem::remove(RESULT_FILE);
                    RCLCPP_INFO(node->get_logger(), "✅ 任务 [%s] 成功完成，清理信号。", current_task.name.c_str());
                }
            } else {
                // 失败不删除文件，下一轮循环会重新读取 RESULT_FILE 进行重试
                RCLCPP_WARN(node->get_logger(), "🔁 任务 [%s] 失败，信号文件已保留，即将重新规划重试...", current_task.name.c_str());
                rclcpp::sleep_for(std::chrono::seconds(2)); // 重试前的缓冲
            }
        }
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}