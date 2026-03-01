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
    RCLCPP_INFO(rclcpp::get_logger("executor"), "🔄 返回初始位姿 (Ready) 以便重新尝试当前步骤...");
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
            } catch (...) {}
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
}

//碰撞场景布置
void setup_planning_scene(moveit::planning_interface::PlanningSceneInterface& psi) {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // --- 定义桌子 ---
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "world"; // 确保这与你的机械臂基座坐标系一致

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {2.0, 2.0, 0.1}; // 宽2m, 深2m, 厚0.1m

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    // 关键：盒子厚0.1m，中心放在-0.05m，则盒子顶部表面恰好在 Z = 0
    table_pose.position.z = -0.051; // 稍微多往下放1mm，防止起始状态因浮点误差判定为碰撞

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;

    collision_objects.push_back(table);

    // 将桌子应用到场景
    psi.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(rclcpp::get_logger("executor"), "✅ 桌面碰撞约束已添加 (Z=0)");
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

// ================= 4. 执行逻辑 (分阶段重试) =================
bool execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         const Task& task) {
    RCLCPP_INFO(node->get_logger(), "🚀 开始任务流程: %s", task.name.c_str());

    // --- 阶段一：抓取循环 (Pick Stage) ---
    bool pick_finished = false;
    while (!pick_finished && rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "📍 [阶段: 抓取] 正在规划抓取路径...");
        arm.setStartStateToCurrentState();
        
        // 1.1 移动到抓取点上方
        geometry_msgs::msg::Pose h_pick = task.pick_pose;
        h_pick.position.z += GRIPPER_HEIGHT + 0.15;
        arm.setPoseTarget(h_pick);
        if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "❌ 预抓取规划失败，回原点重试抓取...");
            go_home(arm); continue; 
        }

        // 1.2 执行抓取动作
        release_gripper(node, 0.08);
        if (!move_linear_safe(arm, -0.15)) {
            RCLCPP_ERROR(node->get_logger(), "❌ 下降抓取失败，回原点重试抓取...");
            go_home(arm); continue;
        }
        grasp_with_force(node, 0.01, 40.0);
        rclcpp::sleep_for(std::chrono::seconds(2));
        
        pick_finished = true; // 抓取成功
    }

    // --- 阶段二：放置循环 (Place Stage) ---
    bool place_finished = false;
    while (!place_finished && rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "📍 [阶段: 放置] 正在规划放置路径...");
        arm.setStartStateToCurrentState();

        // 2.1 抬起动作 (如果是从 Home 重新开始，这步也会尝试执行)
        if (!move_linear_safe(arm, 0.15)) {
            RCLCPP_ERROR(node->get_logger(), "❌ 抬起失败，回原点重新规划放置...");
            go_home(arm); continue; 
        }

        // 2.2 移动到放置点上方
        geometry_msgs::msg::Pose h_place = task.place_pose;
        h_place.position.z += GRIPPER_HEIGHT + 0.15;
        arm.setPoseTarget(h_place);
        if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "❌ 放置点规划失败，回原点重新规划放置...");
            go_home(arm); continue; 
        }

        // 2.3 放置积木
        if (!move_linear_safe(arm, -0.15)) {
            RCLCPP_ERROR(node->get_logger(), "❌ 放置下降失败，回原点重新规划放置...");
            go_home(arm); continue; 
        }
        release_gripper(node, 0.08);
        move_linear_safe(arm, 0.15); // 撤回

        place_finished = true; // 放置成功
    }

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
    moveit::planning_interface::PlanningSceneInterface psi; //场景的布置

    arm.setPlanningTime(10.0);
    arm.setGoalPositionTolerance(0.01);

    // 初始清空场景
    //std::vector<std::string> object_ids = psi.getKnownObjectNames();
    // 1. 先彻底清理旧的残留物体
    std::vector<std::string> object_ids = psi.getKnownObjectNames();
    if (!object_ids.empty()) {
    psi.removeCollisionObjects(object_ids);}

    //添加桌面约束
    setup_planning_scene(psi);

    RCLCPP_INFO(node->get_logger(), ">>> 系统就绪，监听视觉信号...");

    while (rclcpp::ok()) {
        Task current_task;
        if (wait_for_any_task(current_task)) {
            // 执行任务（内部包含分阶段重试逻辑）
            if (execute_single_task(node, arm, current_task)) {
                // 整个任务（Pick+Place）全部成功后才删除信号文件
                if (std::filesystem::exists(RESULT_FILE)) {
                    std::filesystem::remove(RESULT_FILE);
                    RCLCPP_INFO(node->get_logger(), "✅ 任务 [%s] 已彻底完成。", current_task.name.c_str());
                }
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}