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

// 时间参数化相关头文件
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

const double GRIPPER_HEIGHT = 0.104;

using GripperCommand = control_msgs::action::GripperCommand;

// ================= 0. 配置 =================
const std::string RESULT_FILE = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/active_task.yaml";
const std::string MESH_PATH = "/home/i6user/Desktop/robot_lego/FoundationPose/meshes/";

// 🔥 结构体更新：分离物体显示姿态与抓取姿态
struct Task {
    std::string name;
    std::string mesh_file;
    geometry_msgs::msg::Pose brick_pose;   // 物体在场景中生成的真实姿态
    geometry_msgs::msg::Pose gripper_pick; // 机械臂前往抓取的姿态
    geometry_msgs::msg::Pose place_pose;   // 放置姿态
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

// ================= 2. 夹爪 Action =================
bool driveGripperAction(rclcpp::Node::SharedPtr node, double pos) {
    auto client = rclcpp_action::create_client<GripperCommand>(node, "/panda_hand_controller/gripper_cmd");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) return false;

    auto goal = GripperCommand::Goal();
    goal.command.position = pos;
    goal.command.max_effort = 40.0;
    client->async_send_goal(goal);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    return true;
}
void allow_all_collisions(rclcpp::Node::SharedPtr node, const std::string& obj_id) {
    auto ps_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;
    
    // 强制允许以下所有部件之间的互相碰撞
    std::vector<std::string> links = {obj_id, "table", "panda_leftfinger", "panda_rightfinger", "panda_hand", "panda_link8"};
    
    for (const auto& name : links) {
        ps.allowed_collision_matrix.default_entry_names.push_back(name);
        ps.allowed_collision_matrix.default_entry_values.push_back(true);
    }

    ps_pub->publish(ps);
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 增加等待时间确保生效
}
// ================= 5. YAML 解析逻辑 (位置复用 + 旋转独立) =================
bool wait_for_task(Task& t) {
    if (!std::filesystem::exists(RESULT_FILE)) return false;
    try {
        YAML::Node res = YAML::LoadFile(RESULT_FILE);
        
        // 1. 解析任务名称
        std::string name = res["name"].as<std::string>();
        t.name = name;

        // 💡 动态选择 STL 模型
        if (name.find("2x4") != std::string::npos || name.find("4x2") != std::string::npos) {
            t.mesh_file = "LEGO_Duplo_brick_4x2.stl";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "📦 匹配到 4x2 积木模型");
        } 
        else if (name.find("2x2") != std::string::npos) {
            t.mesh_file = "LEGO_Duplo_brick_2x2.stl";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "📦 匹配到 2x2 积木模型");
        } 
        else {
            t.mesh_file = "LEGO_Duplo_brick_4x2.stl"; // 默认
        }

        // 2. 解析 brick_info (物体在场景中的位姿)
        t.brick_pose.position.x = res["brick_info"]["pos"][0].as<double>();
        t.brick_pose.position.y = res["brick_info"]["pos"][1].as<double>();
        t.brick_pose.position.z = res["brick_info"]["pos"][2].as<double>();
        
        t.brick_pose.orientation.x = res["brick_info"]["orientation"][0].as<double>();
        t.brick_pose.orientation.y = res["brick_info"]["orientation"][1].as<double>();
        t.brick_pose.orientation.z = res["brick_info"]["orientation"][2].as<double>();
        t.brick_pose.orientation.w = res["brick_info"]["orientation"][3].as<double>();

        // 3. 设定抓取位姿 (Robot Pick)
        // 💡 关键：Position 直接复用 brick_info 的坐标
        t.gripper_pick.position = t.brick_pose.position;

        // 💡 关键：Orientation 读取 robot_pick 专属的四元数
        t.gripper_pick.orientation.x = res["robot_pick"]["orientation"][0].as<double>();
        t.gripper_pick.orientation.y = res["robot_pick"]["orientation"][1].as<double>();
        t.gripper_pick.orientation.z = res["robot_pick"]["orientation"][2].as<double>();
        t.gripper_pick.orientation.w = res["robot_pick"]["orientation"][3].as<double>();

        // 4. 解析放置位姿 (Place)
        t.place_pose.position.x = res["place"]["pos"][0].as<double>();
        t.place_pose.position.y = res["place"]["pos"][1].as<double>();
        t.place_pose.position.z = res["place"]["pos"][2].as<double>();
        
        t.place_pose.orientation.x = res["place"]["orientation"][0].as<double>();
        t.place_pose.orientation.y = res["place"]["orientation"][1].as<double>();
        t.place_pose.orientation.z = res["place"]["orientation"][2].as<double>();
        t.place_pose.orientation.w = res["place"]["orientation"][3].as<double>();

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "YAML 解析深度错误: %s", e.what());
        return false;
    }
}


bool move_linear(moveit::planning_interface::MoveGroupInterface& arm, double z_delta) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target = arm.getCurrentPose().pose;
    target.position.z += z_delta;
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory traj;
    // 参数：路径点, 步长(1mm), 跳变阈值(禁用), 轨迹输出, 避障(关闭)
    double fraction = arm.computeCartesianPath(waypoints, 0.001, 0.0, traj, false);
    
    if (fraction > 0.1) {
        arm.execute(traj);
        return true;
    }
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "直线移动失败！");
    return false;
}


bool execute_single_task(rclcpp::Node::SharedPtr node,
                         moveit::planning_interface::MoveGroupInterface& arm,
                         moveit::planning_interface::PlanningSceneInterface& psi,
                         const Task& task) {
    
    RCLCPP_INFO(node->get_logger(), "🚀 执行任务（防止掉落模式）: %s", task.name.c_str());

    const double GRIPPER_OFFSET = 0.1234; // 法兰盘到指尖的高度
    const double HOVER = 0.15;           // 15cm 悬停
    const double SAFE_PICK_Z = 0.015;    // 🔥 稍微调高到 3.5cm，防止手指尖打翻积木

    // 1. 生成并彻底豁免碰撞
    moveit_msgs::msg::CollisionObject brick;
    brick.id = task.name; brick.header.frame_id = arm.getPlanningFrame();
    brick.meshes.push_back(load_stl_mesh(MESH_PATH + task.mesh_file));
    brick.mesh_poses.push_back(task.brick_pose); 
    brick.operation = brick.ADD;
    psi.applyCollisionObject(brick);
    
    allow_all_collisions(node, task.name);

    // --- [抓取序列] ---
    // A. 到达 15cm 悬停位
    geometry_msgs::msg::Pose p_hover = task.gripper_pick;
    p_hover.position.z += (GRIPPER_OFFSET + HOVER);
    arm.setPoseTarget(p_hover);
    arm.move();

    driveGripperAction(node, 0.04); // 完全张开
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 💡 额外等待：确保手指完全张开

    // B. 下降到物体上方 3.5cm (15 - 3.5 = 11.5cm)
    RCLCPP_INFO(node->get_logger(), "📍 下降中...");
    move_linear(arm, -0.155); 

    // C. 关键步骤：先 Attach，再闭合，防止物理引擎判定积木掉落
    // 定义哪些部分可以接触积木
    std::vector<std::string> touch_links = {"panda_leftfinger", "panda_rightfinger", "panda_hand", "panda_link8"};
    
    RCLCPP_INFO(node->get_logger(), "📍 正在闭合夹爪并锁定物体...");
    driveGripperAction(node, 0.01); // 闭合到 2cm
    arm.attachObject(task.name, "panda_hand", touch_links); // 逻辑绑定
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));//给物理引擎缓冲时间

    // D. 抬起 (抬起 15cm)
    move_linear(arm, HOVER); 

    // --- [放置序列] ---
    RCLCPP_INFO(node->get_logger(), "📍 移动到放置位...");
    geometry_msgs::msg::Pose p_place = task.place_pose;
    p_place.position.z += (GRIPPER_OFFSET + HOVER);
    
    // 再次强制豁免，防止搬运途中撞到其他积木
    allow_all_collisions(node, task.name);
    
    arm.setPoseTarget(p_place);
    arm.move();

    // 下降到目标高度 (15cm)
    move_linear(arm, -HOVER); 

    // 释放
    RCLCPP_INFO(node->get_logger(), "📍 释放物体");
    arm.detachObject(task.name);   
    driveGripperAction(node, 0.04); 
    
    // 撤离
    move_linear(arm, HOVER); 

    if (std::filesystem::exists(RESULT_FILE)) std::filesystem::remove(RESULT_FILE);
    RCLCPP_INFO(node->get_logger(), "✅ 任务完成");
    return true;
}
int main(int argc, char** argv) {
    // 1. 初始化 ROS 2
    rclcpp::init(argc, argv);
    
    // 2. 配置节点选项 (允许参数覆盖)
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("lego_batch_executor", node_options);

    // 💡 关键：使用多线程执行器
    // MoveIt 2 需要在后台持续接收机器人状态（Joint States）和 TF 变换。
    // 如果主循环阻塞了，机械臂将无法获取最新位姿，导致移动失败。
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // 在独立线程中启动 Spin
    std::thread spinning_thread([&executor]() {
        executor.spin();
    });

    RCLCPP_INFO(node->get_logger(), ">>> [START] LEGO 批量执行节点已启动");

    // 3. 使用大括号限定 MoveIt 接口的作用域
    // 这样可以确保在调用 rclcpp::shutdown() 之前，MoveGroup 等对象已经先被销毁。
    // 这能有效避免你之前遇到的 "Attempting to unload library while objects exist" 报错。
    {
        // 4. 初始化 MoveIt 接口
        moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
        moveit::planning_interface::PlanningSceneInterface psi;

        // 设置全局运动参数
        arm.setPlanningTime(20.0);           // 增加规划时间，应对复杂环境
        arm.setMaxVelocityScalingFactor(0.5); // 限制全局最大速度
        arm.setMaxAccelerationScalingFactor(0.5);

        // 5. 初始化环境场景：添加桌面
        // 建议在循环开始前添加一次即可
        moveit_msgs::msg::CollisionObject table;
        table.id = "table";
        table.header.frame_id = "world";
        
        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions = {2.0, 2.0, 0.02}; // 2米见方的大桌子
        
        geometry_msgs::msg::Pose table_pose;
        table_pose.position.x = 0.0;
        table_pose.position.y = 0.0;
        table_pose.position.z = -0.011;    // 桌面高度设为 -0.011 (假设原点在桌面厚度中心)
        table_pose.orientation.w = 1.0;
        
        table.primitives.push_back(box);
        table.primitive_poses.push_back(table_pose);
        table.operation = table.ADD;

        RCLCPP_INFO(node->get_logger(), ">>> 正在添加桌面碰撞体...");
        psi.applyCollisionObject(table);

        // 6. 主循环：监听并执行任务
        RCLCPP_INFO(node->get_logger(), ">>> [READY] 正在监听 YAML 任务文件: %s", RESULT_FILE.c_str());
        
        while (rclcpp::ok()) {
            Task current_task;
            
            // 轮询：检查是否有新的任务产生
            if (wait_for_task(current_task)) {
                RCLCPP_INFO(node->get_logger(), "🔔 收到新任务: %s", current_task.name.c_str());
                
                // 执行抓取与放置序列 (含 15cm->3cm 下降及慢速放置)
                try {
                    execute_single_task(node, arm, psi, current_task);
                    RCLCPP_INFO(node->get_logger(), "✅ 任务执行成功。");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node->get_logger(), "❌ 任务执行异常: %s", e.what());
                }
            }

            // 频率控制：每 500ms 检查一次文件是否存在
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    } // arm 和 psi 在这里被析构

    // 7. 优雅关闭
    RCLCPP_INFO(node->get_logger(), ">>> [SHUTDOWN] 节点正在关闭...");
    rclcpp::shutdown();
    
    if (spinning_thread.joinable()) {
        spinning_thread.join();
    }

    return 0;
}