#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <thread>
#include <vector>

// ================= 配置区域 =================
// 必须与 Vision Node 中的 RESULT_FILE 路径完全一致
const std::string RESULT_FILE = "/Users/Zhuanz/Downloads/RWTH/robot_lego/src/panda_pick/src/vision_result.yaml";
const std::string TASKS_YAML = "/home/aaa/robot/ros2_ws/src/panda_pick/src/tasks.yaml";

struct TaskTemplate {
    std::string name;
};

class LegoAssemblyExecutor : public rclcpp::Node {
public:
    LegoAssemblyExecutor() : Node("lego_assembly_executor") {
        arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");
        hand = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "hand");
        
        arm->setMaxVelocityScalingFactor(0.3);
        arm->setMaxAccelerationScalingFactor(0.2);
        
        load_task_names();
        RCLCPP_INFO(this->get_logger(), "✅ 执行器准备就绪，等待视觉信号...");
    }

    void run_assembly_sequence() {
        for (const auto &task : task_list) {
            if (!rclcpp::ok()) break;

            RCLCPP_INFO(this->get_logger(), "▶️ 正在处理积木: %s", task.name.c_str());

            // 1. 握手：等待视觉节点生成已转换好的机械臂坐标文件
            auto [pick_pose, place_pose] = wait_for_vision_data(task.name);

            // 2. 执行抓取 (直接使用视觉传来的机械臂系坐标)
            execute_move_sequence(pick_pose, true);

            // 3. 执行放置 (直接使用视觉传来的机械臂系坐标)
            execute_move_sequence(place_pose, false);

            // 4. 【核心握手逻辑】删除文件，触发视觉节点进行下一个积木识别
            if (std::filesystem::exists(RESULT_FILE)) {
                std::filesystem::remove(RESULT_FILE);
                RCLCPP_INFO(this->get_logger(), "🎊 %s 放置完成，已清理信号文件。", task.name.c_str());
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_INFO(this->get_logger(), "🏁 所有任务执行完毕！");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand;
    std::vector<TaskTemplate> task_list;

    // 只需加载名称，位姿计算交由视觉节点根据 tasks.yaml 处理
    void load_task_names() {
        YAML::Node config = YAML::LoadFile(TASKS_YAML);
        for (const auto& item : config["tasks"]) {
            TaskTemplate t;
            t.name = item["name"].as<std::string>();
            task_list.push_back(t);
        }
    }

    // 监听文件并解析成对的位姿
    std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose> wait_for_vision_data(const std::string& target_name) {
        while (rclcpp::ok()) {
            if (std::filesystem::exists(RESULT_FILE)) {
                try {
                    YAML::Node res = YAML::LoadFile(RESULT_FILE);
                    if (res["block_name"].as<std::string>() == target_name) {
                        return {parse_pose(res["pick_pose"]), parse_pose(res["place_pose"])};
                    }
                } catch (...) {}
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        return {};
    }

    geometry_msgs::msg::Pose parse_pose(const YAML::Node& node) {
        geometry_msgs::msg::Pose p;
        p.position.x = node["x"].as<double>();
        p.position.y = node["y"].as<double>();
        p.position.z = node["z"].as<double>();
        p.orientation.x = node["qx"].as<double>();
        p.orientation.y = node["qy"].as<double>();
        p.orientation.z = node["qz"].as<double>();
        p.orientation.w = node["qw"].as<double>();
        return p;
    }

    void control_gripper(double width) {
        hand->setJointValueTarget("panda_finger_joint1", width);
        hand->setJointValueTarget("panda_finger_joint2", width);
        hand->move();
    }

    // 通用的移动序列：预到达 -> 目标点 -> 动作 -> 撤回
    void execute_move_sequence(const geometry_msgs::msg::Pose& target, bool is_pick) {
        geometry_msgs::msg::Pose pre = target;
        pre.position.z += 0.1; // 上方 10cm 预到达

        arm->setPoseTarget(pre); arm->move();

        if (is_pick) control_gripper(0.04); // 抓取前开爪

        arm->setMaxVelocityScalingFactor(0.05); // 慢速接触
        arm->setPoseTarget(target); arm->move();

        if (is_pick) control_gripper(0.01); // 抓取关爪
        else control_gripper(0.04);         // 放置开爪

        arm->setMaxVelocityScalingFactor(0.3); // 恢复速度
        arm->setPoseTarget(pre); arm->move();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegoAssemblyExecutor>();
    std::thread([node]() {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        node->run_assembly_sequence();
    }).detach();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}