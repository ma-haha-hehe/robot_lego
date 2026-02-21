#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp_action/rclcpp_action.hpp>
//#include <franka_msgs/action/gripper_command.hpp>
//#include <franka_msgs/action/grasp.hpp>
//#include <franka_msgs/action/move.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
const double GRIPPER_HEIGHT = 0.103; // Panda Gripper 从手腕到夹爪底部的高度
// ================= 任务结构体 =================
struct Task {
    int id;
    std::string name; // 必须与 spawn_objects 中的 ID 对应 (例如 "leg_left")
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 1. 读取 YAML =================
std::vector<Task> load_tasks_from_yaml(const std::string& file_path) {
    std::vector<Task> tasks;
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        if (!config["tasks"]) {
            RCLCPP_ERROR(rclcpp::get_logger("yaml_loader"), "YAML file has no 'tasks' node!");
            return tasks;
        }

        for (const auto& t : config["tasks"]) {
            Task task;
            task.id = t["id"].as<int>();
            task.name = t["name"].as<std::string>();

            // Pick
            auto pick_p = t["pick"]["pos"];
            auto pick_o = t["pick"]["orientation"];
            task.pick_pose.position.x = pick_p[0].as<double>();
            task.pick_pose.position.y = pick_p[1].as<double>();
            task.pick_pose.position.z = pick_p[2].as<double>();
            task.pick_pose.orientation.x = pick_o[0].as<double>();
            task.pick_pose.orientation.y = pick_o[1].as<double>();
            task.pick_pose.orientation.z = pick_o[2].as<double>();
            task.pick_pose.orientation.w = pick_o[3].as<double>();

            // Place
            auto place_p = t["place"]["pos"];
            auto place_o = t["place"]["orientation"];
            task.place_pose.position.x = place_p[0].as<double>();
            task.place_pose.position.y = place_p[1].as<double>();
            task.place_pose.position.z = place_p[2].as<double>();
            task.place_pose.orientation.x = place_o[0].as<double>();
            task.place_pose.orientation.y = place_o[1].as<double>();
            task.place_pose.orientation.z = place_o[2].as<double>();
            task.place_pose.orientation.w = place_o[3].as<double>();

            tasks.push_back(task);
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("yaml_loader"), "Error reading YAML: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("yaml_loader"), "File not found or other error: %s", e.what());
    }
    return tasks;
}

// ================= 2. 在 RViz 中生成场景物体 =================
void spawn_objects_in_rviz(moveit::planning_interface::PlanningSceneInterface& psi) {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // --- 创建桌子 ---
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "world";
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {0.6, 1.0, 0.04}; // 桌子尺寸
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.4;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.4;
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    collision_objects.push_back(table);

    // --- 创建积木零件 Lambda ---
    auto add_block = [&](std::string name, double x, double y, double dx, double dy, double dz) {
        moveit_msgs::msg::CollisionObject block;
        block.id = name;
        block.header.frame_id = "world";
        block.primitives.resize(1);
        block.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        block.primitives[0].dimensions = {dx, dy, dz};
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.42 + (dz / 2.0); // 放在桌面上 (0.4 + 0.02 + half_height)
        pose.orientation.w = 1.0;
        block.primitive_poses.push_back(pose);
        block.operation = block.ADD;
        collision_objects.push_back(block);
    };

    // 生成物体 (必须与 YAML 中的 name 一致)
    add_block("leg_left",  0.55,  0.25, 0.03, 0.03, 0.09);
    add_block("leg_right", 0.55,  0.15, 0.03, 0.03, 0.09);
    add_block("hips",      0.55,  -0.3,  0.03, 0.09, 0.03);
    add_block("torso",     0.55, -0.15, 0.03, 0.15, 0.03);
    add_block("head",      0.62,  0.0,  0.03, 0.03, 0.09);
    add_block("hand_left", 0.62,  0.15, 0.03, 0.03, 0.03);
    add_block("hand_right",0.62, -0.15, 0.03, 0.03, 0.03);

    // 提交到 RViz
    psi.applyCollisionObjects(collision_objects);
}

// ================= 3. 禁用特定碰撞检测 (ACM) =================
void disable_collision_checks(rclcpp::Node::SharedPtr node, const std::vector<Task>& tasks) {
    auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
    RCLCPP_INFO(node->get_logger(), "Updating Collision Matrix...");
    rclcpp::sleep_for(1s);

    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true; // 重要：只更新我们改变的部分

    // 定义机器人本体 Link
    std::vector<std::string> body_links = {
        "panda_link0", "panda_link1", "panda_link2", "panda_link3",
        "panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_link8",
        "panda_hand", "panda_leftfinger", "panda_rightfinger"
    };

    // 收集所有需要设置的物体名称
    std::vector<std::string> all_names = body_links;
    all_names.push_back("table");
    for(const auto& t : tasks) {
        all_names.push_back(t.name);
    }

    // 初始化 ACM
    planning_scene.allowed_collision_matrix.entry_names = all_names;
    planning_scene.allowed_collision_matrix.entry_values.resize(all_names.size());

    // 构建逻辑
    for (size_t i = 0; i < all_names.size(); ++i) {
        // 默认设为 true (开启碰撞检测，即不允许碰撞)
        planning_scene.allowed_collision_matrix.entry_values[i].enabled.resize(all_names.size(), true);
        
        for (size_t j = 0; j < all_names.size(); ++j) {
            std::string A = all_names[i];
            std::string B = all_names[j];

            // 检查 A 或 B 是否是特定类型
            bool A_is_body = std::find(body_links.begin(), body_links.end(), A) != body_links.end();
            bool B_is_body = std::find(body_links.begin(), body_links.end(), B) != body_links.end();
            bool A_is_table = (A == "table");
            bool B_is_table = (B == "table");
            
            bool A_is_block = false;
            bool B_is_block = false;
            for(const auto& t : tasks) {
                if (t.name == A) A_is_block = true;
                if (t.name == B) B_is_block = true;
            }

            // 规则 1: 允许 Body 和 Table 碰撞 (enabled = false)
            if ((A_is_body && B_is_table) || (A_is_table && B_is_body)) {
                planning_scene.allowed_collision_matrix.entry_values[i].enabled[j] = false;
            }
            // 规则 2: 允许 积木 和 桌子 碰撞 (enabled = false) - 否则抓起来时会认为碰桌子
            if ((A_is_block && B_is_table) || (A_is_table && B_is_block)) {
                planning_scene.allowed_collision_matrix.entry_values[i].enabled[j] = false;
            }
            // 规则 3: 允许 积木 和 积木 碰撞 (可选，防止堆叠报错)
            //if (A_is_block && B_is_block) {
            //    planning_scene.allowed_collision_matrix.entry_values[i].enabled[j] = false;
            //}
     }
    }

    // 多次发布以确保 MoveGroup 收到
    for(int i=0; i<5; i++) {
        scene_pub->publish(planning_scene);
        rclcpp::sleep_for(100ms);
    }
    RCLCPP_WARN(node->get_logger(), ">>> COLLISION MATRIX UPDATED <<<");
}

// ================= 4. 辅助移动函数 =================
bool move_to_pose(moveit::planning_interface::MoveGroupInterface& group,
                  geometry_msgs::msg::Pose target,
                  rclcpp::Logger logger,
                  std::string label)
{
    group.setPoseTarget(target);
    // 增加规划时间，处理复杂场景
    group.setPlanningTime(5.0); 
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // 尝试最多 3 次
    for(int i=0; i<3; ++i) {
        if (group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            group.execute(my_plan);
            RCLCPP_INFO(logger, ">>> %s: Success", label.c_str());
            return true;
        }
        RCLCPP_WARN(logger, ">>> %s: Planning failed, retrying (%d/3)...", label.c_str(), i+1);
        rclcpp::sleep_for(200ms);
    }
    RCLCPP_ERROR(logger, ">>> %s: FAILED", label.c_str());
    return false;
}

// ================= 5. 执行单个任务 =================
void execute_task(rclcpp::Node::SharedPtr node,
                  moveit::planning_interface::MoveGroupInterface& arm_group,
                  moveit::planning_interface::MoveGroupInterface& hand_group,
                  const Task& task)
{
    RCLCPP_INFO(node->get_logger(), "=== EXECUTING TASK: %s ===", task.name.c_str());

    // --- PICK ---
    // 1. Hover Pick
    geometry_msgs::msg::Pose hover_pick = task.pick_pose;
    hover_pick.position.z += GRIPPER_HEIGHT + 0.05; // +0.05
    if (!move_to_pose(arm_group, hover_pick, node->get_logger(), "Hover Pick")) return;

    // 2. Open Hand
    hand_group.setJointValueTarget("panda_finger_joint1", 0.04);
    hand_group.setJointValueTarget("panda_finger_joint2", 0.04);

    // the error distance
    hand_group.setGoalJointTolerance(0.01);
    RCLCPP_INFO(node->get_logger(),"Attempting to grasp...");
    hand_group.move();

    // 3. Descend
    geometry_msgs::msg::Pose grasp = task.pick_pose;
    grasp.position.z += GRIPPER_HEIGHT; // 抓取高度 0.015
    if (!move_to_pose(arm_group, grasp, node->get_logger(), "Descend Pick")) return;

    // 4. Grasp (Attach)
    // 注意：Attach 会修改 PlanningScene，告诉系统这个物体现在连在机器人手上
    
    hand_group.setJointValueTarget("panda_finger_joint1", 0.006); // 尝试闭合
    hand_group.setJointValueTarget("panda_finger_joint2", 0.006);
    // 使用 move 而不是 grasp action，因为我们模拟的是简单抓取
    hand_group.move(); 
    arm_group.attachObject(task.name, "panda_link8"); 
    rclcpp::sleep_for(std::chrono::seconds(1));


    // 5. Lift
    if (!move_to_pose(arm_group, hover_pick, node->get_logger(), "Lift Pick")) return;

    // --- PLACE ---
    // 6. Hover Place
    geometry_msgs::msg::Pose hover_place = task.place_pose;
    hover_place.position.z += GRIPPER_HEIGHT + 0.10;
    if (!move_to_pose(arm_group, hover_place, node->get_logger(), "Hover Place")) return;

    // 7. Descend Place
    geometry_msgs::msg::Pose place_target = task.place_pose;
    place_target.position.z += GRIPPER_HEIGHT; 
    if (!move_to_pose(arm_group, place_target, node->get_logger(), "Descend Place")) return;

    // 8. Release (Detach)
    

    hand_group.setJointValueTarget("panda_finger_joint1", 0.04);
    hand_group.setJointValueTarget("panda_finger_joint2", 0.04);
    hand_group.move();
    arm_group.detachObject(task.name);
    rclcpp::sleep_for(500ms);

    // 9. Retract
    if (!move_to_pose(arm_group, hover_place, node->get_logger(), "Retract")) return;
}

// ================= 6. MAIN =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 更改节点选项以允许参数声明（如果是为了 launch 文件）
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("auto_build_node", node_options);

    // 🔥 使用 MultiThreadedExecutor，防止 MoveIt 动作卡死 🔥
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    // 初始化 MoveIt
    moveit::planning_interface::MoveGroupInterface arm_group(node, "panda_arm");
    moveit::planning_interface::MoveGroupInterface hand_group(node, "hand");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    arm_group.setMaxVelocityScalingFactor(0.5);
    arm_group.setMaxAccelerationScalingFactor(0.5);

    // 1. 加载任务
    // 建议：不要硬编码，使用参数或相对路径。这里为了保持你代码逻辑，仍使用硬编码，请根据实际情况修改。
    std::string yaml_path = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/task.yaml";
    RCLCPP_INFO(node->get_logger(), "Loading tasks from: %s", yaml_path.c_str());
    
    std::vector<Task> task_list = load_tasks_from_yaml(yaml_path);
    if (task_list.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Task list empty. Stopping.");
        rclcpp::shutdown();
        return 1;
    }

    // 2. 初始化场景
    RCLCPP_INFO(node->get_logger(), "Setting up planning scene...");
    spawn_objects_in_rviz(planning_scene_interface);
    // 等待物体在 Planning Scene 中注册完毕
    rclcpp::sleep_for(2s);

    // 3. 修改碰撞矩阵 (Critical Step)
    disable_collision_checks(node, task_list);
    rclcpp::sleep_for(1s);

    // 4. 循环执行任务
    RCLCPP_INFO(node->get_logger(), "Starting Execution Loop...");
    for (const auto& task : task_list) {
        if(!rclcpp::ok()) break;
        execute_task(node, arm_group, hand_group, task);
        rclcpp::sleep_for(1s);
    }

    RCLCPP_INFO(node->get_logger(), "ALL DONE.");
    
    // 退出
    rclcpp::shutdown();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    return 0;
}