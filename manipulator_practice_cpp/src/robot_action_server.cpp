#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include "manipulator_practice_cpp/srv/move_to_pose.hpp"
#include <Eigen/Geometry>

class RobotActionServer : public rclcpp::Node
{
public:
  RobotActionServer() : Node("robot_action_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // Service
    srv_ = this->create_service<manipulator_practice_cpp::srv::MoveToPose>(
        "move_to_pose",
        std::bind(&RobotActionServer::handle_move, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "RobotActionServer started. Waiting for MoveGroup initialization...");
  }

  void init_move_group() {
      // Assuming 'dsr01' or 'manipulator' - standard is usually 'manipulator' or 'arm'
      // I'll assume 'manipulator' which is common for generated moveit configs.
      // Ideally this is a parameter.
      std::string group_name = "dsr01"; // Doosan often uses dsr01 or manipulator
      // Let's try to declare it
      this->declare_parameter("move_group_name", "dsr01");
      group_name = this->get_parameter("move_group_name").as_string();

      try {
          move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), group_name);
          move_group_->setMaxVelocityScalingFactor(0.1);
          move_group_->setMaxAccelerationScalingFactor(0.1);
          RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for group: %s", group_name.c_str());
      } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to init MoveGroup: %s", e.what());
      }
  }

private:
  rclcpp::Service<manipulator_practice_cpp::srv::MoveToPose>::SharedPtr srv_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  void handle_move(const std::shared_ptr<manipulator_practice_cpp::srv::MoveToPose::Request> request,
                   std::shared_ptr<manipulator_practice_cpp::srv::MoveToPose::Response> response)
  {
      if (!move_group_) {
          response->success = false;
          response->message = "MoveGroup not initialized";
          RCLCPP_ERROR(this->get_logger(), "MoveGroup not initialized");
          return;
      }

      RCLCPP_INFO(this->get_logger(), "Received Move Request");

      // Parse Matrix
      Eigen::Matrix4f mat;
      // Safety check for size? request->pose_matrix is std::array<float, 16>
      for(int i=0; i<16; ++i) mat(i/4, i%4) = request->pose_matrix[i];
      
      Eigen::Matrix3f R = mat.block<3,3>(0,0);
      Eigen::Vector3f T = mat.block<3,1>(0,3);
      
      Eigen::Quaternionf q(R);
      
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = T.x();
      target_pose.position.y = T.y();
      target_pose.position.z = T.z();
      target_pose.orientation.x = q.x();
      target_pose.orientation.y = q.y();
      target_pose.orientation.z = q.z();
      target_pose.orientation.w = q.w();
      
      move_group_->setPoseTarget(target_pose);
      
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if(success) {
          RCLCPP_INFO(this->get_logger(), "Planning success. Executing...");
          move_group_->execute(my_plan);
          response->success = true;
          response->message = "Moved successfully";
      } else {
          RCLCPP_ERROR(this->get_logger(), "Planning failed");
          response->success = false;
          response->message = "Planning failed";
      }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotActionServer>();
  
  // Init MoveGroup after node is created
  node->init_move_group();

  // Use MultiThreadedExecutor for MoveIt
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
