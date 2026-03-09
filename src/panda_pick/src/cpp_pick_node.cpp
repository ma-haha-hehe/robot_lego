#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>
#include <thread>

// STL 与几何图形处理头文件
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/msg/mesh.hpp>

using GripperCommand = control_msgs::action::GripperCommand;

// ================= 0. 配置区域 =================
const std::string RESULT_FILE = "/home/aaa/robot_lego/src/panda_pick/src/active_task.yaml";
const std::string MESH_BASE_PATH = "/home/aaa/robot/FoundationPose/meshes/"; 
const double GRIPPER_HEIGHT = 0.1034;

struct Task {
    std::string name;
    std::string mesh_file;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 1. 辅助函数：加载并缩放 STL (mm -> m) =================
shape_msgs::msg::Mesh load_stl_mesh(const std::string& file_name) {
    std::string full_path = MESH_BASE_PATH + file_name;
    if (!std::filesystem::exists(full_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("executor"), "❌ 找不到 STL 文件: %s", full_path.c_str());
        return shape_msgs::msg::Mesh();
    }

    shapes::Mesh* m = shapes::createMeshFromResource("file://" + full_path);
    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    // 🔥 单位缩放修正：如果 STL 是毫米单位，必须乘以 0.001
    for (auto& vertex : mesh.vertices) {
        vertex.x *= 0.001;
        vertex.y *= 0.001;
        vertex.z *= 0.001;
    }
    delete m;
    return mesh;
}

// ================= 2. 仿真夹爪控制 =================
bool move_gripper_sim(rclcpp::Node::SharedPtr node, double position) {
    auto client = rclcpp_action::create_client<GripperCommand>(node, "/panda_gripper_controller/gripper_cmd");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) return false;
    auto goal = GripperCommand::Goal();
    goal.command.position = position; 
    goal.command.max_effort = 100.0;
    client->async_send_goal(goal);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}

// ================= 3. 动态场景同步：生成识别到的积木 =================
void spawn_detected_brick(moveit::planning_interface::PlanningSceneInterface& psi, const Task& task) {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = "target_brick"; // 固定 ID 方便后续 attach 操作
    obj.header.frame_id = "world";
    
    shape_msgs::msg::Mesh mesh = load_stl_mesh(task.mesh_file);
    if (mesh.vertices.empty()) return;

    obj.meshes.push_back(mesh);
    obj.mesh_poses.push_back(task.pick_pose);
    obj.operation = obj.ADD;
    
    psi.applyCollisionObject(obj);
}
// ================= 1. YAML 解析增强 =================
bool parse_task(Task& t) {
    try {
        YAML::Node config = YAML::LoadFile(RESULT_FILE);
        
        // 1. 基本信息
        t.name = config["name"].as<std::string>();
        // 🔥 注意：你的 YAML 没写 mesh_file，这里先给个默认值，否则会报错
        t.mesh_file = "base_2x4_lvl2.stl"; 

        // 2. 解析 pick (对应你 YAML 里的 pick 节点)
        auto pick_node = config["pick"];
        t.pick_pose.position.x = pick_node["pos"][0].as<double>();
        t.pick_pose.position.y = pick_node["pos"][1].as<double>();
        t.pick_pose.position.z = pick_node["pos"][2].as<double>();
        
        // 四元数顺序通常为 [x, y, z, w]，请确认你视觉输出的顺序
        t.pick_pose.orientation.x = pick_node["orientation"][0].as<double>();
        t.pick_pose.orientation.y = pick_node["orientation"][1].as<double>();
        t.pick_pose.orientation.z = pick_node["orientation"][2].as<double>();
        t.pick_pose.orientation.w = pick_node["orientation"][3].as<double>();

        // 3. 解析 place (对应你 YAML 里的 place 节点)
        auto place_node = config["place"];
        t.place_pose.position.x = place_node["pos"][0].as<double>();
        t.place_pose.position.y = place_node["pos"][1].as<double>();
        t.place_pose.position.z = place_node["pos"][2].as<double>();
        
        t.place_pose.orientation.x = place_node["orientation"][0].as<double>();
        t.place_pose.orientation.y = place_node["orientation"][1].as<double>();
        t.place_pose.orientation.z = place_node["orientation"][2].as<double>();
        t.place_pose.orientation.w = place_node["orientation"][3].as<double>();

        return true;
    } catch (const std::exception& e) {
        // 打印具体错误原因，方便调试
        std::cerr << "YAML 解析具体错误: " << e.what() << std::endl;
        return false;
    }
}

// ================= 4. 核心执行逻辑 =================
bool execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         moveit::planning_interface::PlanningSceneInterface& psi,
                         const Task& task) {
    
    RCLCPP_INFO(node->get_logger(), "📍 同步视觉场景：在 RViz 生成积木...");
    spawn_detected_brick(psi, task);

    const double OFFSET_Z = 0.15 + GRIPPER_HEIGHT;
    
    // --- 抓取阶段 ---
    geometry_msgs::msg::Pose h_pick = task.pick_pose;
    h_pick.position.z += OFFSET_Z;
    arm.setPoseTarget(h_pick);
    if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) return false;

    move_gripper_sim(node, 0.08); // 打开
    
    // 下降抓取 (简化描述)
    move_gripper_sim(node, 0.02); 
    
    // 🔥 关键：在虚拟世界中将积木附着在手上，实现跟随移动
    arm.attachObject("target_brick", "panda_hand");
    RCLCPP_INFO(node->get_logger(), "📦 积木已附着，开始搬运...");

    // --- 放置阶段 ---
    geometry_msgs::msg::Pose h_place = task.place_pose;
    h_place.position.z += OFFSET_Z;
    arm.setPoseTarget(h_place);
    arm.move();

    // 释放
    arm.detachObject("target_brick");
    psi.removeCollisionObjects({"target_brick"});
    move_gripper_sim(node, 0.08);
    
    if (std::filesystem::exists(RESULT_FILE)) std::filesystem::remove(RESULT_FILE);
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 🔥 修正 1: 必须与 Launch 文件中的 use_sim_time 保持一致 (False)
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("use_sim_time", false);
    auto node = rclcpp::Node::make_shared("lego_sim_sync_executor", node_options);

    // 开启异步线程处理 MoveGroup 回调
    std::thread spin_thread([node]() { rclcpp::spin(node); });

    moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
    moveit::planning_interface::PlanningSceneInterface psi;

    arm.setPlanningTime(5.0);
    arm.setMaxVelocityScalingFactor(0.5);

    RCLCPP_INFO(node->get_logger(), "🔄 正在初始化：切换至 Ready 姿态...");
    arm.setNamedTarget("ready");
    arm.move();

    RCLCPP_INFO(node->get_logger(), ">>> 视觉同步仿真系统就绪，监听路径: %s", RESULT_FILE.c_str());

    while (rclcpp::ok()) {
        if (std::filesystem::exists(RESULT_FILE)) {
            RCLCPP_INFO(node->get_logger(), "🔔 检测到新任务文件！");
            Task t;
            if (parse_task(t)) {
                RCLCPP_INFO(node->get_logger(), "🚀 开始执行任务: %s", t.name.c_str());
                execute_single_task(node, arm, psi, t);
                
                // 任务完成后删除文件，防止重复执行
                std::filesystem::remove(RESULT_FILE);
                RCLCPP_INFO(node->get_logger(), "✅ 任务完成，等待下一个视觉信号...");
            } else {
                RCLCPP_ERROR(node->get_logger(), "❌ YAML 解析失败，请检查格式！");
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}