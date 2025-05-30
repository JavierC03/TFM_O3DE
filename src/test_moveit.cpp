#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;

  // Grupo para controlar la pinza
  auto gripper_group = MoveGroupInterface(node, "hand");

  std::vector<double> open_gripper_joints = {0.04, 0.04};
  gripper_group.setJointValueTarget(open_gripper_joints);

  if(gripper_group.move()) {
    RCLCPP_INFO(logger, "Pinza abierta correctamente.");
  } else {
    RCLCPP_ERROR(logger, "Error al abrir la pinza.");
  }

  // Grupo para controlar el brazo
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");
  move_group_interface.setPoseReferenceFrame("world");
  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  
  // Pose del objeto detectado
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.3058234669808529;
  target_pose.position.y = 0.015369474099220265;
  target_pose.position.z = 0.05148127278362813;

  tf2::Quaternion q;
  q.setRPY(M_PI, 0, 0);  // orientación hacia abajo
  target_pose.orientation = tf2::toMsg(q);

  // Fase 1: ir a una posición por encima del objeto
  geometry_msgs::msg::Pose above_pose = target_pose;
  above_pose.position.z += 0.2;  // 10 cm por encima del objeto

  move_group_interface.setPoseTarget(above_pose);

  auto const [success1, plan1] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    bool ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success1) {
    RCLCPP_INFO(logger, "Movimiento hacia arriba del objeto exitoso.");
    move_group_interface.execute(plan1);
  } else {
    RCLCPP_ERROR(logger, "Fallo al planificar movimiento sobre el objeto.");
    rclcpp::shutdown();
    return 1;
  }

  // Fase 2: descenso en línea recta hacia el objeto
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(above_pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_interface.computeCartesianPath(
    waypoints,
    0.01,  // resolución (distancia entre puntos)
    0.0,   // jump_threshold
    trajectory
  );

  if (fraction > 0.9) {
    RCLCPP_INFO(logger, "Trayectoria cartesiana generada correctamente.");
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;
    move_group_interface.execute(cartesian_plan);
  } else {
    RCLCPP_ERROR(logger, "Falló la trayectoria cartesiana.");
    rclcpp::shutdown();
    return 1;
  }

  // Fase 3: cerrar la pinza
  std::vector<double> close_gripper_joints = {0.0, 0.0};
  gripper_group.setJointValueTarget(close_gripper_joints);
  gripper_group.setMaxVelocityScalingFactor(0.1); 

  if(gripper_group.move()){
    RCLCPP_INFO(logger, "Pinza cerrada correctamente.");
  }else{
    RCLCPP_ERROR(logger, "Error al cerrar la pinza.");
  }

  rclcpp::shutdown();
  return 0;
}
