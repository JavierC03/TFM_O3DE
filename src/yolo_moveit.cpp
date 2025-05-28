#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <yolo_msgs/msg/detection_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class YoloMoveitController : public rclcpp::Node{
public:
    YoloMoveitController() : Node("yolo_listener"){
        
    }

    void init() {
        auto node_ptr = this->shared_from_this();
        move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "panda_arm");
        gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "hand");
        subscription_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
            "/yolo/detections_3d", 10, std::bind(&YoloMoveitController::topic_callback, this, std::placeholders::_1));
    }
private:

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr subscription_;

    double target_x = 0.0;
    double target_y = 0.0;
    double target_z = 0.0;

    std::string target_class_;
    bool class_selected_ = false;
    
    void descend_straight(const geometry_msgs::msg::Pose& start_pose) {
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z = 0.15;  // Ajusta la altura de descenso según sea necesario

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(start_pose);
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_interface->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
        if (fraction > 0.9){
            RCLCPP_INFO(this->get_logger(), "Descending.");
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            move_group_interface->execute(cartesian_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to descend.");
        }
    }

    geometry_msgs::msg::Pose approach_target(double x, double y, double z) {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(M_PI, 0, M_PI/4);  // orientación hacia abajo
        target_pose.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose above_pose = target_pose;
        above_pose.position.z += 0.10; 
        move_group_interface->setPoseTarget(above_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_interface->plan(plan)) {
            RCLCPP_INFO(this->get_logger(), "Moved to target position.");
            move_group_interface->execute(plan);
            return above_pose;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to target position.");
            return geometry_msgs::msg::Pose();  // Retorna una pose vacía en caso de error
        }
    }

    void open_gripper() {
        std::vector<double> open_gripper_joints = {0.04, 0.04};
        gripper_group->setJointValueTarget(open_gripper_joints);
        gripper_group->setMaxVelocityScalingFactor(0.5);
        if (gripper_group->move()) {
            RCLCPP_INFO(this->get_logger(), "Pinza abierta correctamente.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error al abrir la pinza.");
        }
    }

    void close_gripper() {
        std::vector<double> close_gripper_joints = {0.0, 0.0};
        gripper_group->setJointValueTarget(close_gripper_joints);
        gripper_group->setMaxVelocityScalingFactor(0.1); 
        if(gripper_group->move()){
            RCLCPP_INFO(this->get_logger(), "Pinza cerrada correctamente.");
        }else{
            RCLCPP_ERROR(this->get_logger(), "Error al cerrar la pinza.");
        }
    }

    std::string get_target_class(const std::vector<std::string>& detected_classes) {
        std::set<std::string> unique_classes(detected_classes.begin(), detected_classes.end());
        
        if(unique_classes.empty()){
            RCLCPP_WARN(this->get_logger(), "No objects detected.");
            return "";
        }

        std::cout << "\nSe han detectado los siguientes objetos:\n";
        for (const auto& cls : unique_classes) {
            std::cout << "- " << cls << "\n";
        }

        std::string selected;
        while (selected.empty()) {
            std::cout << "Seleccione un objetivo de la lista: ";
            std::getline(std::cin, selected);

            if(unique_classes.count(selected)){
                RCLCPP_INFO(this->get_logger(), "Objetivo seleccionado: %s", selected.c_str());
                return selected;
            } else {
                RCLCPP_WARN(this->get_logger(), "Objetivo no válido. Intente de nuevo.\n");
                selected.clear();
            }
        }
        return "";
    }

    void topic_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
        if(!class_selected_){
            std::vector<std::string> all_classes;
            for(const auto& detection : msg->detections) {
                all_classes.push_back(detection.class_name);
            }
            target_class_ = get_target_class(all_classes);
            class_selected_ = true;
        }

        for (const auto& detection : msg->detections) {
            if (detection.class_name == target_class_) {
                target_x = detection.bbox3d.center.position.x;
                target_y = detection.bbox3d.center.position.y;
                target_z = detection.bbox3d.center.position.z + 0.1;

                open_gripper();
                auto pose_after_approach = approach_target(target_x, target_y, target_z);
                descend_straight(pose_after_approach);
                close_gripper();
                rclcpp::shutdown();
                return;
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloMoveitController>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}