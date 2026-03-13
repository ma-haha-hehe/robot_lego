#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>
#include <thread>

// STL 与几何处理
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/msg/mesh.hpp>

// 时间参数化与轨迹处理
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

const double GRIPPER_HEIGHT = 0.104;
using GripperCommand = control_msgs::action::GripperCommand;

// ================= 配置 =================
const std::string RESULT_FILE = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/active_task.yaml";
const std::string MESH_PATH = "/home/i6user/Desktop/robot_lego/FoundationPose/meshes/";

struct Task {
    std::string name;
    std::string mesh_file;
    geometry_msgs::msg::Pose brick_pose;
    geometry_msgs::msg::Pose gripper_pick;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 1. STL 加载 =================
shape_msgs::msg::Mesh load_stl_mesh(const std::string& file_path) {
    std::string uri = (file_path.substr(0, 7) != "file://") ? "file://" + file_path : file_path;
    shapes::Mesh* m = shapes::createMeshFromResource(uri);
    if (m == nullptr) throw std::runtime_error("无法加载 STL: " + uri);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
    for (auto& vertex : mesh.vertices) {
        vertex.x *= 0.001; vertex.y *= 0.001; vertex.z *= 0.001;
    }
    delete m;
    return mesh;
}

// ================= 2. 夹爪控制 =================
bool driveGripperAction(rclcpp::Node::SharedPtr node, double pos) {
    auto client = rclcpp_action::create_client<GripperCommand>(node, "/panda_hand_controller/gripper_cmd");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) return false;
    auto goal = GripperCommand::Goal();
    goal.command.position = pos;
    goal.command.max_effort = 20.0;
    client->async_send_goal(goal);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    return true;
}

// ================= 3. 碰撞矩阵优化 =================
void allow_all_collisions(rclcpp::Node::SharedPtr node, const std::string& obj_id) {
    auto ps_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;
    std::vector<std::string> links = {obj_id, "table", "panda_leftfinger", "panda_rightfinger", "panda_hand", "panda_link8"};
    for (const auto& name : links) {
        ps.allowed_collision_matrix.default_entry_names.push_back(name);
        ps.allowed_collision_matrix.default_entry_values.push_back(true);
    }
    ps_pub->publish(ps);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

// ================= 4. YAML 解析 =================
bool wait_for_task(Task& t) {
    if (!std::filesystem::exists(RESULT_FILE)) return false;
    try {
        YAML::Node res = YAML::LoadFile(RESULT_FILE);
        t.name = res["name"].as<std::string>();
        t.mesh_file = (t.name.find("2x4") != std::string::npos || t.name.find("4x2") != std::string::npos) ? 
                      "LEGO_Duplo_brick_4x2.stl" : "LEGO_Duplo_brick_2x2.stl";

        auto parse_pose = [](const YAML::Node& n, geometry_msgs::msg::Pose& p) {
            p.position.x = n["pos"][0].as<double>();
            p.position.y = n["pos"][1].as<double>();
            p.position.z = n["pos"][2].as<double>();
            p.orientation.x = n["orientation"][0].as<double>();
            p.orientation.y = n["orientation"][1].as<double>();
            p.orientation.z = n["orientation"][2].as<double>();
            p.orientation.w = n["orientation"][3].as<double>();
        };

        parse_pose(res["brick_info"], t.brick_pose);
        t.gripper_pick.position = t.brick_pose.position;
        t.gripper_pick.orientation.x = res["robot_pick"]["orientation"][0].as<double>();
        t.gripper_pick.orientation.y = res["robot_pick"]["orientation"][1].as<double>();
        t.gripper_pick.orientation.z = res["robot_pick"]["orientation"][2].as<double>();
        t.gripper_pick.orientation.w = res["robot_pick"]["orientation"][3].as<double>();
        parse_pose(res["place"], t.place_pose);
        return true;
    } catch (...) { return false; }
}

// ================= 5. 直线运动 (TOTG 调速) =================
bool move_linear(moveit::planning_interface::MoveGroupInterface& arm, double z_delta, double speed_scale = 0.1) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target = arm.getCurrentPose().pose;
    target.position.z += z_delta;
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    double fraction = arm.computeCartesianPath(waypoints, 0.001, 0.0, trajectory_msg, false);
    
    if (fraction > 0.1) {
        robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), arm.getName());
        rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory_msg);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (totg.computeTimeStamps(rt, speed_scale, speed_scale)) {
            rt.getRobotTrajectoryMsg(trajectory_msg);
            arm.execute(trajectory_msg);
            return true;
        }
    }
    return false;
}

// ================= 6. 核心任务执行 (STOMP 模式) =================
bool execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         moveit::planning_interface::PlanningSceneInterface& psi,
                         const Task& task) {
    
    RCLCPP_INFO(node->get_logger(), "--- 启动 STOMP 规划任务: %s ---", task.name.c_str());

    // 1. 添加碰撞物体
    moveit_msgs::msg::CollisionObject brick;
    brick.id = task.name; brick.header.frame_id = arm.getPlanningFrame();
    brick.meshes.push_back(load_stl_mesh(MESH_PATH + task.mesh_file));
    brick.mesh_poses.push_back(task.brick_pose);
    brick.operation = brick.ADD;
    psi.applyCollisionObject(brick);
    allow_all_collisions(node, task.name);

    const double GRIPPER_OFFSET = 0.1234; 
    const double HOVER = 0.15;

    // --- [抓取阶段] ---
    // A. STOMP 快速接近 (PTP)
    // 💡 关键：这里会调用 STOMP 进行轨迹优化，路径会非常平滑
    RCLCPP_INFO(node->get_logger(), ">>> STOMP 规划：接近目标...");
    geometry_msgs::msg::Pose p_hover = task.gripper_pick;
    p_hover.position.z += (GRIPPER_OFFSET + HOVER);
    arm.setPoseTarget(p_hover);
    arm.move(); 

    driveGripperAction(node, 0.04); 

    // B. 直线下降 (笛卡尔插值)
    move_linear(arm, -0.155, 0.2); 

    // C. 抓取
    driveGripperAction(node, 0.02); 
    arm.attachObject(task.name, "panda_hand"); 
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // D. 抬起
    move_linear(arm, HOVER, 0.5);

    // --- [放置阶段] ---
    // E. STOMP 移动到放置点上方
    RCLCPP_INFO(node->get_logger(), ">>> STOMP 规划：移动至放置区...");
    geometry_msgs::msg::Pose p_place = task.place_pose;
    p_place.position.z += (GRIPPER_OFFSET + HOVER);
    
    allow_all_collisions(node, task.name);
    arm.setPoseTarget(p_place);
    arm.move(); // 再次使用 STOMP

    // F. 慢速放置
    move_linear(arm, -HOVER, 0.1); 

    // G. 释放
    arm.detachObject(task.name);   
    driveGripperAction(node, 0.04); 
    
    // H. 撤离
    move_linear(arm, HOVER, 0.5); 

    if (std::filesystem::exists(RESULT_FILE)) std::filesystem::remove(RESULT_FILE);
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("lego_stomp_executor", node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinning_thread([&executor]() { executor.spin(); });

    {
        moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
        moveit::planning_interface::PlanningSceneInterface psi;

        // 🔥🔥🔥 [关键修改]：指定使用 STOMP 规划流水线
        // 这要求你的 moveit 配置文件中已经包含了 stomp_planning.yaml
        arm.setPlanningPipelineId("stomp"); 
        
        arm.setPlanningTime(15.0);           
        arm.setMaxVelocityScalingFactor(0.4); 
        arm.setMaxAccelerationScalingFactor(0.2);

        // 初始化桌面碰撞
        moveit_msgs::msg::CollisionObject table;
        table.id = "table"; table.header.frame_id = "world";
        shape_msgs::msg::SolidPrimitive box; box.type = box.BOX; box.dimensions = {2.0, 2.0, 0.02};
        geometry_msgs::msg::Pose tp; tp.position.z = -0.011; tp.orientation.w = 1.0;
        table.primitives.push_back(box); table.primitive_poses.push_back(tp);
        table.operation = table.ADD;
        psi.applyCollisionObject(table);

        while (rclcpp::ok()) {
            Task current_task;
            if (wait_for_task(current_task)) {
                execute_single_task(node, arm, psi, current_task);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    rclcpp::shutdown();
    spinning_thread.join();
    return 0;
}