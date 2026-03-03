#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>
#include <thread>
#include <fstream>

// 时间参数化相关头文件
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

// ================= 0. 配置区域 =================
const std::string RESULT_FILE = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/active_task.yaml";
const double GRIPPER_HEIGHT = 0.103; 

struct Task {
    std::string name;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 1. 辅助函数：安全返回初始位姿 =================
bool go_home(moveit::planning_interface::MoveGroupInterface& arm) {
    RCLCPP_INFO(rclcpp::get_logger("executor"), "🔄 返回初始位姿 (Ready) 以便重新尝试当前步骤...");
    arm.setNamedTarget("ready"); 
    auto result = arm.move();
    return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

// ================= 2. YAML 监听逻辑 (严格阻塞等待版) =================
bool wait_for_any_task(Task& current_task) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "等待视觉节点发送信号 (active_task.yaml)...");

    // 只有看到文件才会返回，否则一直在这里 sleep 等待
    while (rclcpp::ok()) {
        if (std::filesystem::exists(RESULT_FILE)) {
            try {
                YAML::Node res = YAML::LoadFile(RESULT_FILE);
                current_task.name = res["name"].as<std::string>();

                auto fill_pose = [](YAML::Node node, geometry_msgs::msg::Pose& pose) {
                    pose.position.x = node["pos"][0].as<double>();
                    pose.position.y = node["pos"][1].as<double>();
                    pose.position.z = node["pos"][2].as<double>();
                    pose.orientation.x = node["orientation"][0].as<double>();
                    pose.orientation.y = node["orientation"][1].as<double>();
                    pose.orientation.z = node["orientation"][2].as<double>();
                    pose.orientation.w = node["orientation"][3].as<double>();
                };

                fill_pose(res["pick"], current_task.pick_pose);
                fill_pose(res["place"], current_task.place_pose);

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "✅ 收到任务: %s", current_task.name.c_str());

                // 消费信号文件
                std::filesystem::remove(RESULT_FILE);
                return true; 
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "解析 YAML 失败: %s", e.what());
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
}

// ================= 3. 碰撞场景布置 =================
void setup_planning_scene(moveit::planning_interface::PlanningSceneInterface& psi) {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    moveit_msgs::msg::CollisionObject table;
    table.id = "table"; table.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_primitive.dimensions = {2.0, 2.0, 0.1}; 
    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0; table_pose.position.z = -0.051; 
    table.primitives.push_back(table_primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    collision_objects.push_back(table);

    auto make_board = [&](std::string id, double y) {
        moveit_msgs::msg::CollisionObject b; b.id = id; b.header.frame_id = "world";
        shape_msgs::msg::SolidPrimitive bp; bp.type = bp.BOX; bp.dimensions = {2.0, 0.02, 1.0};
        geometry_msgs::msg::Pose p; p.orientation.w = 1.0; p.position.y = y; p.position.z = 0.5;
        b.primitives.push_back(bp); b.primitive_poses.push_back(p);
        return b;
    };
    collision_objects.push_back(make_board("left_board", 0.40));
    collision_objects.push_back(make_board("right_board", -0.40));

    psi.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(rclcpp::get_logger("executor"), "✅ 已成功添加桌面及左右侧限位板 (Y = ±0.4m)");
}

// ================= 4. 仿真夹爪动作 (Mock) =================
bool grasp_with_force(rclcpp::Node::SharedPtr node, double target_width, double force) {
    RCLCPP_INFO(node->get_logger(), "🦾 [仿真] 夹爪执行 GRASP: 宽度=%.3f, 力度=%.1f", target_width, force);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    return true;
}

bool release_gripper(rclcpp::Node::SharedPtr node, double width) {
    RCLCPP_INFO(node->get_logger(), "👐 [仿真] 夹爪执行 MOVE: 宽度=%.3f", width);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

// ================= 5. 线性移动工具 =================
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

// ================= 6. 核心执行逻辑 (保留原版所有打印) =================
bool execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         const Task& task) {
    RCLCPP_INFO(node->get_logger(), "🚀 开始任务流程: %s", task.name.c_str());

    const double OFFSET_Z = 0.15+GRIPPER_HEIGHT;
    const double NORMAL_SPEED = 0.2; 
    const double SLOW_SPEED = 0.01;  

    // --- 阶段一：抓取循环 (Pick Stage) ---
    bool pick_finished = false;
    while (!pick_finished && rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "📍 [阶段: 抓取] 正在移至抓取点上方...");
        arm.setStartStateToCurrentState();
        
        geometry_msgs::msg::Pose h_pick = task.pick_pose;
        h_pick.position.z += OFFSET_Z;
        
        arm.setPoseTarget(h_pick);
        if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "❌ 预抓取位姿规划失败，重试...");
            go_home(arm); continue; 
        }

        release_gripper(node, 0.08);

        RCLCPP_INFO(node->get_logger(), "⬇️ 线性下降中...");
        if (!move_linear(arm, -0.15, NORMAL_SPEED, NORMAL_SPEED)) {
            RCLCPP_ERROR(node->get_logger(), "❌ 线性下降失败，重试...");
            go_home(arm); continue;
        }

        grasp_with_force(node, 0.02, 40.0);
        rclcpp::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(node->get_logger(), "⬆️ 抓取后抬起...");
        move_linear(arm, OFFSET_Z, NORMAL_SPEED, NORMAL_SPEED);
        
        pick_finished = true; 
    }

    // --- 阶段二：放置循环 (Place Stage) ---
    bool place_finished = false;
    while (!place_finished && rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "📍 [阶段: 放置] 正在移至放置点上方...");
        arm.setStartStateToCurrentState();

        geometry_msgs::msg::Pose h_place = task.place_pose;
        h_place.position.z += OFFSET_Z;
        
        arm.setPoseTarget(h_place);
        if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "❌ 放置点上方规划失败，重试...");
            go_home(arm); continue; 
        }

        RCLCPP_INFO(node->get_logger(), "⬇️ 线性下降中...");
        if (!move_linear(arm, -0.10, NORMAL_SPEED, NORMAL_SPEED)) {
            RCLCPP_ERROR(node->get_logger(), "❌ 线性下降失败，重试...");
            go_home(arm); continue;
        }

        RCLCPP_INFO(node->get_logger(), "⬇️ 慢速下降放置中 (Speed: %.2f)...", SLOW_SPEED);
        if (!move_linear(arm, -0.05,SLOW_SPEED,SLOW_SPEED)){
            RCLCPP_ERROR(node->get_logger(), "❌ 放置下降失败，重试...");
            go_home(arm); continue; 
        }

        RCLCPP_INFO(node->get_logger(), "👐 释放积木...");
        release_gripper(node, 0.08);
        rclcpp::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(node->get_logger(), "⬆️ 放置完成，线性撤回...");
        move_linear(arm, OFFSET_Z, NORMAL_SPEED, NORMAL_SPEED);

        place_finished = true; 
    }

    RCLCPP_INFO(node->get_logger(), "✅ 任务 [%s] 执行成功", task.name.c_str());
    return true; 
}

// ================= 7. 主函数 =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_batch_executor");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::PlanningSceneInterface psi;

    arm.setPlanningTime(10.0);
    arm.setGoalPositionTolerance(0.01);

    setup_planning_scene(psi);

    RCLCPP_INFO(node->get_logger(), ">>> 仿真系统就绪，监听视觉信号...");

    while (rclcpp::ok()) {
        Task current_task;
        // wait_for_any_task 现在是阻塞的，只有拿到新任务才会返回 true
        if (wait_for_any_task(current_task)) {
            execute_single_task(node, arm, current_task);
            RCLCPP_INFO(node->get_logger(), "✅ 任务 [%s] 已彻底完成。", current_task.name.c_str());
        }
    }

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}