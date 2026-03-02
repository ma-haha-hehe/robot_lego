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

#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>

// ================= 1. YAML 监听逻辑 =================
bool wait_for_any_task(Task& current_task) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "等待视觉节点发送信号 (active_task.yaml)...");

    while (rclcpp::ok()) {
        if (std::filesystem::exists(RESULT_FILE)) {
            try {
                // 1. 加载文件
                YAML::Node res = YAML::LoadFile(RESULT_FILE);
                current_task.name = res["name"].as<std::string>();

                // 2. 修改 fill_pose：从 YAML 中动态读取四元数
                auto fill_pose = [](YAML::Node node, geometry_msgs::msg::Pose& pose) {
                    // 读取位置 (pos)
                    pose.position.x = node["pos"][0].as<double>();
                    pose.position.y = node["pos"][1].as<double>();
                    pose.position.z = node["pos"][2].as<double>();

                    // 读取方向 (orientation: [x, y, z, w])
                    // 注意：Python 的 scipy 导出的顺序通常是 [x, y, z, w]
                    pose.orientation.x = node["orientation"][0].as<double>();
                    pose.orientation.y = node["orientation"][1].as<double>();
                    pose.orientation.z = node["orientation"][2].as<double>();
                    pose.orientation.w = node["orientation"][3].as<double>();
                };

                fill_pose(res["pick"], current_task.pick_pose);
                fill_pose(res["place"], current_task.place_pose);

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "✅ 收到任务: %s", current_task.name.c_str());

                // 3. 【重要】消费掉该文件，实现握手信号机制
                // 这样 vision_node.py 就会停止等待，开始寻找下一个物体
                std::filesystem::remove(RESULT_FILE);
                
                return true;

            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "解析 YAML 失败: %s", e.what());
            }
        }
        // 降低 CPU 占用
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

// ================= 1. 核心工具：链式线性规划 (含速度缩放) =================
// 作用：基于一个“虚拟”的起始状态规划直线路径，并应用 IPTP 时间参数化
bool plan_linear_chain(moveit::planning_interface::MoveGroupInterface& arm,
                       const moveit::core::RobotState& start_state,
                       double z_delta, double vel_scale,
                       moveit_msgs::msg::RobotTrajectory& trajectory_msg) {
    
    // 创建虚拟起始状态
    moveit::core::RobotStatePtr virtual_start_state(new moveit::core::RobotState(start_state));
    
    // 计算目标位姿
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose start_pose;
    tf2::fromMsg(virtual_start_state->getGlobalLinkTransform(arm.getEndEffectorLink()), start_pose);
    
    geometry_msgs::msg::Pose target_pose = start_pose;
    target_pose.position.z += z_delta;
    waypoints.push_back(target_pose);

    // 计算笛卡尔路径 (必须 100% 成功，即 fraction == 1.0)
    // 这样如果下降会撞到 Z=0 的桌面或者撞到关节极限，规划就会直接返回 false
    double fraction = arm.computeCartesianPath(waypoints, 0.005, 0.0, trajectory_msg, true);
    if (fraction < 1.0) return false;

    // 应用你原有的 IPTP 时间参数化逻辑进行速度/加速度缩放
    robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), arm.getName());
    rt.setRobotTrajectoryMsg(*virtual_start_state, trajectory_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    
    return iptp.computeTimeStamps(rt, vel_scale, vel_scale) && (rt.getRobotTrajectoryMsg(trajectory_msg), true);
}

// ================= 2. 完善后的执行函数 (全流程闭环) =================
bool execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         const Task& task) {
    RCLCPP_INFO(node->get_logger(), "🚀 开始全序列模拟规划: %s", task.name.c_str());

    const double HOVER_Z = 0.15;
    const double SPEED_NORMAL = 0.2;
    const double SPEED_SLOW = 0.01; // 你要求的放置下降慢速

    // ------------------- 阶段一：Pick 序列 (验证 Hover->Down->Up) -------------------
    bool pick_ok = false;
    while (!pick_ok && rclcpp::ok()) {
        arm.setStartStateToCurrentState();
        
        // 1.1 规划 Hover 点
        geometry_msgs::msg::Pose h_pick = task.pick_pose;
        h_pick.position.z += HOVER_Z;
        arm.setPoseTarget(h_pick);
        moveit::planning_interface::MoveGroupInterface::Plan p1_hover;
        if (arm.plan(p1_hover) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "❌ Pick Hover 不可达，回 Ready 重试...");
            go_home(arm); continue;
        }

        // 模拟到达 Hover 后的状态
        moveit::core::RobotState hover_state(*arm.getCurrentState());
        hover_state.setJointGroupPositions(arm.getName(), p1_hover.trajectory_.joint_trajectory.points.back().positions);

        // 1.2 规划线性下降 (Normal Speed)
        moveit_msgs::msg::RobotTrajectory p2_down;
        if (!plan_linear_chain(arm, hover_state, -HOVER_Z, SPEED_NORMAL, p2_down)) {
            RCLCPP_ERROR(node->get_logger(), "❌ 下降路径检测到关节极限或桌面碰撞，重试路径...");
            continue; // 这里不 go_home，直接重新 plan p1 寻找新的关节解
        }

        // 1.3 规划线性抬起
        moveit::core::RobotState pick_state(hover_state);
        pick_state.setJointGroupPositions(arm.getName(), p2_down.joint_trajectory.points.back().positions);
        moveit_msgs::msg::RobotTrajectory p3_up;
        if (!plan_linear_chain(arm, pick_state, HOVER_Z, SPEED_NORMAL, p3_up)) continue;

        // 【Pick 执行】
        RCLCPP_INFO(node->get_logger(), "✅ Pick 序列验证通过。");
        release_gripper(node, 0.08);
        arm.execute(p1_hover);
        arm.execute(p2_down);
        grasp_with_force(node, 0.015, 40.0);
        rclcpp::sleep_for(std::chrono::seconds(1));
        arm.execute(p3_up);
        pick_ok = true;
    }

    // ------------------- 阶段二：Place 序列 (验证 Hover->Down->Up) -------------------
    bool place_ok = false;
    while (!place_ok && rclcpp::ok()) {
        arm.setStartStateToCurrentState();

        // 2.1 规划 Hover 点
        geometry_msgs::msg::Pose h_place = task.place_pose;
        h_place.position.z += HOVER_Z;
        arm.setPoseTarget(h_place);
        moveit::planning_interface::MoveGroupInterface::Plan p1_hover;
        if (arm.plan(p1_hover) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "❌ Place Hover 不可达，重试...");
            go_home(arm); continue;
        }

        moveit::core::RobotState hover_state(*arm.getCurrentState());
        hover_state.setJointGroupPositions(arm.getName(), p1_hover.trajectory_.joint_trajectory.points.back().positions);

        // 2.2 规划慢速下降 (SPEED_SLOW = 0.01)
        moveit_msgs::msg::RobotTrajectory p2_down;
        if (!plan_linear_chain(arm, hover_state, -HOVER_Z, SPEED_SLOW, p2_down)) {
            RCLCPP_ERROR(node->get_logger(), "❌ 放置下降路径受阻，重试...");
            continue;
        }

        // 2.3 规划抬起
        moveit::core::RobotState place_state(hover_state);
        place_state.setJointGroupPositions(arm.getName(), p2_down.joint_trajectory.points.back().positions);
        moveit_msgs::msg::RobotTrajectory p3_up;
        if (!plan_linear_chain(arm, place_state, HOVER_Z, SPEED_NORMAL, p3_up)) continue;

        // 【Place 执行】
        RCLCPP_INFO(node->get_logger(), "✅ Place 序列验证通过，开始慢速放置。");
        arm.execute(p1_hover);
        arm.execute(p2_down); // 执行 0.01 倍速的慢速下降
        release_gripper(node, 0.08);
        rclcpp::sleep_for(std::chrono::seconds(1));
        arm.execute(p3_up);
        place_ok = true;
    }

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