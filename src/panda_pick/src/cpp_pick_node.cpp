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

// 时间参数化相关头文件，确保速度缩放生效
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using GraspAction = franka_msgs::action::Grasp;

// 物理参数常量
const double GRIPPER_HEIGHT = 0.103; // 夹爪法兰到指尖的距离
const double ARM_VEL_DEFAULT = 0.4;
const double ARM_ACC_DEFAULT = 0.3;

struct Task {
    std::string name;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 1. 批量 YAML 加载逻辑 =================
// 专门适配你提供的 tasks: 列表结构
std::vector<Task> load_all_tasks(const std::string& file_path) {
    std::vector<Task> tasks;
    if (!std::filesystem::exists(file_path)) return tasks;

    try {
        YAML::Node config = YAML::LoadFile(file_path);
        for (const auto& item : config["tasks"]) {
            Task t;
            t.name = item["name"].as<std::string>();

            auto fill_pose = [](YAML::Node node, geometry_msgs::msg::Pose& pose) {
                pose.position.x = node["pos"][0].as<double>();
                pose.position.y = node["pos"][1].as<double>();
                pose.position.z = node["pos"][2].as<double>();
                pose.orientation.x = node["orientation"][0].as<double>();
                pose.orientation.y = node["orientation"][1].as<double>();
                pose.orientation.z = node["orientation"][2].as<double>();
                pose.orientation.w = node["orientation"][3].as<double>();
            };

            fill_pose(item["pick"], t.pick_pose);
            fill_pose(item["place"], t.place_pose);
            tasks.push_back(t);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("yaml_loader"), "YAML 解析失败: %s", e.what());
    }
    return tasks;
}

// ================= 场景构建函数 =================
void setup_planning_scene(moveit::planning_interface::PlanningSceneInterface& psi) {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // --- 1. 地面 (Ground Plane) ---
    moveit_msgs::msg::CollisionObject ground;
    ground.id = "ground";
    ground.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive ground_prim;
    ground_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    ground_prim.dimensions = {2.0, 2.0, 0.01}; // 2米见方，厚度1cm
    
    geometry_msgs::msg::Pose ground_pose;
    ground_pose.position.z = -0.005; // 稍微向下偏移，避免与机械臂底座z=0完全重合
    ground.primitives.push_back(ground_prim);
    ground.primitive_poses.push_back(ground_pose);
    ground.operation = ground.ADD;
    collision_objects.push_back(ground);

    // --- 2. 四周防护罩 (Square Cage) ---
    // 我们用 4 个薄板围成一个正方形
    auto create_wall = [&](std::string id, double x, double y, double dx, double dy, double dz) {
        moveit_msgs::msg::CollisionObject wall;
        wall.id = id;
        wall.header.frame_id = "world";
        shape_msgs::msg::SolidPrimitive wall_prim;
        wall_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        wall_prim.dimensions = {dx, dy, dz};
        
        geometry_msgs::msg::Pose wall_pose;
        wall_pose.position.x = x;
        wall_pose.position.y = y;
        wall_pose.position.z = dz / 2.0;
        wall.primitives.push_back(wall_prim);
        wall.primitive_poses.push_back(wall_pose);
        wall.operation = wall.ADD;
        return wall;
    };

    double cage_size = 1.2; // 罩子边长
    double wall_thickness = 0.02;
    double wall_height = 1.0;
    double offset = cage_size / 2.0;

    collision_objects.push_back(create_wall("wall_front",  offset, 0, wall_thickness, cage_size, wall_height));
    collision_objects.push_back(create_wall("wall_back",  -offset, 0, wall_thickness, cage_size, wall_height));
    collision_objects.push_back(create_wall("wall_left",   0,  offset, cage_size, wall_thickness, wall_height));
    collision_objects.push_back(create_wall("wall_right",  0, -offset, cage_size, wall_thickness, wall_height));

    // --- 3. 积木 (Blocks) ---
    // 这里我们把任务列表中的积木预先生成在场景中
    // 假设积木尺寸为 3cm x 3cm x 3cm
    // 注意：任务执行时需要 attach 它们，否则机械臂碰到积木会认为发生碰撞
    // 如果你只想避障而不 attach，请确保抓取姿态非常精准
    
    // 提交所有物体
    psi.applyCollisionObjects(collision_objects);
}

// ================= 2. 持续力抓取 (Grasp Action) =================
bool grasp_with_force(rclcpp::Node::SharedPtr node, double target_width, double force) {
    auto client = rclcpp_action::create_client<GraspAction>(node, "/panda_gripper/grasp");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) return false;

    GraspAction::Goal goal_msg;
    goal_msg.width = target_width; // 设置为比物体窄的值以维持压力
    goal_msg.speed = 0.05;
    goal_msg.force = force;        // 持续夹紧力 (N)
    goal_msg.epsilon.inner = 0.05;
    goal_msg.epsilon.outer = 0.05;

    RCLCPP_INFO(node->get_logger(), ">>> 执行持续力抓取: %.1f N", force);
    auto future = client->async_send_goal(goal_msg);
    rclcpp::sleep_for(std::chrono::seconds(1)); // 给予硬件反应时间
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

// ================= 4. 单个积木执行逻辑 =================
void execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         moveit::planning_interface::MoveGroupInterface& hand,
                         const Task& task) {
    RCLCPP_INFO(node->get_logger(), "### 正在处理积木: %s ###", task.name.c_str());

    // STEP 1: 预抓取 (移动到上方 15cm 并对齐姿态)
    geometry_msgs::msg::Pose h_pick = task.pick_pose;
    h_pick.position.z += GRIPPER_HEIGHT + 0.15;
    arm.setPoseTarget(h_pick);
    arm.move();

    // STEP 2: 开爪
    hand.setJointValueTarget("panda_finger_joint1", 0.04);
    hand.setJointValueTarget("panda_finger_joint2", 0.04);
    hand.move();

    // STEP 3: 直线下降 15cm
    move_linear(arm, -0.15, 0.2, 0.2);

    // STEP 4: 持续力抓持 (40N) 并直线抬起
    grasp_with_force(node, 0.01, 40.0);
    move_linear(arm, 0.15, 0.3, 0.3);

    // STEP 5: 移动到放置点上方 15cm
    geometry_msgs::msg::Pose h_place = task.place_pose;
    h_place.position.z += GRIPPER_HEIGHT + 0.15;
    arm.setPoseTarget(h_place);
    arm.move();

    // STEP 6: 慢速直线放置
    move_linear(arm, -0.12, 0.2, 0.2); // 快速下降
    move_linear(arm, -0.03, 0.02, 0.02); // 极慢触碰

    // STEP 7: 释放并撤回
    hand.setJointValueTarget("panda_finger_joint1", 0.04);
    hand.setJointValueTarget("panda_finger_joint2", 0.04);
    hand.move();
    move_linear(arm, 0.15, 0.3, 0.3);
}

// ================= 5. MAIN 主函数 =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lego_batch_executor");

    // 必须使用多线程执行器处理 Action 回调
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand(node, "hand"); // 请确保与 SRDF 一致

    //实例化场景接口
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // 1. 设置碰撞环境
    RCLCPP_INFO(node->get_logger(), "正在初始化规划场景（地面与防护罩）...");
    setup_planning_scene(planning_scene_interface);

    // 路径：请确保指向你存储 7 个任务的那个 tasks.yaml
    const std::string yaml_path = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/tasks.yaml";

    RCLCPP_INFO(node->get_logger(), ">>> 正在加载批量任务列表: %s", yaml_path.c_str());
    std::vector<Task> task_list = load_all_tasks(yaml_path);

    if (task_list.empty()) {
        RCLCPP_ERROR(node->get_logger(), "未找到任务列表或格式错误。程序退出。");
    } else {
        RCLCPP_INFO(node->get_logger(), "共加载 %zu 个任务。开始顺序执行...", task_list.size());
        
        for (size_t i = 0; i < task_list.size(); ++i) {
            if (!rclcpp::ok()) break;
            
            RCLCPP_INFO(node->get_logger(), "[任务进度 %zu/%zu]", i + 1, task_list.size());
            execute_single_task(node, arm, hand, task_list[i]);
            
            RCLCPP_INFO(node->get_logger(), "任务 %zu 完成。等待系统稳定...", i + 1);
            rclcpp::sleep_for(std::chrono::seconds(2)); // 每个积木间的安全停顿
        }
    }

    RCLCPP_INFO(node->get_logger(), "### 所有批量任务已完成！ ###");
    rclcpp::shutdown();
    if (executor_thread.joinable()) executor_thread.join();
    return 0;
}
