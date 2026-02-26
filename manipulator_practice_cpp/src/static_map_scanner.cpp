#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fstream>
#include <sstream>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// ROS2 Paths
#include <ament_index_cpp/get_package_share_directory.hpp>

// TF2 includes
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class StaticMapScanner : public rclcpp::Node
{
public:
  StaticMapScanner() : Node("static_map_scanner")
  {
    // Parameters
    this->declare_parameter("map_file_path", "workspace_map.bt"); // 상대경로로 수정이
    this->declare_parameter("robot_base_frame", "base_link");
    this->declare_parameter("robot_tip_frame", "link_6"); 
    this->declare_parameter("min_depth", 0.2);
    this->declare_parameter("max_depth", 1.0);

    // Load Calibration (표준 경로 방식 적용)
    load_hand_eye_calibration();

    // OctoMap init (Smart Pointer 사용)
    tree_ = std::make_unique<octomap::OcTree>(0.025);
    tree_->setOccupancyThres(0.7); 
    tree_->setProbHit(0.75);
    tree_->setProbMiss(0.15);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers & Publishers
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera/color/camera_info", 10,
      std::bind(&StaticMapScanner::info_callback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/aligned_depth_to_color/image_raw", 10,
      std::bind(&StaticMapScanner::depth_callback, this, std::placeholders::_1));

    octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_full", 1);

    save_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "save_and_exit",
      std::bind(&StaticMapScanner::save_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "StaticMapScanner initialized with standard path resolution.");
  }

private:
  bool got_cam_info_ = false;
  float fx_, fy_, cx_, cy_;
  
  Eigen::Isometry3d T_hand_eye_; // Affine보다 Isometry 권장
  std::unique_ptr<octomap::OcTree> tree_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  void load_hand_eye_calibration()
  {
    try {
      // 패키지 설치 경로를 기준으로 config 파일 탐색
      std::string pkg_path = ament_index_cpp::get_package_share_directory("manipulator_practice_python");
      std::string csv_path = pkg_path + "/config/result_park.csv";

      std::ifstream file(csv_path);
      if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV: %s. Using Identity.", csv_path.c_str());
        T_hand_eye_ = Eigen::Isometry3d::Identity();
        return;
      }
      
      Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
      std::string line;
      int row = 0;
      while (std::getline(file, line) && row < 4) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (std::getline(ss, cell, ',') && col < 4) {
          double val = std::stod(cell);
          if (col == 3 && row < 3) val /= 1000.0; // mm -> m
          mat(row, col) = val;
          col++;
        }
        row++;
      }
      T_hand_eye_ = Eigen::Isometry3d(mat);
      RCLCPP_INFO(this->get_logger(), "Hand-Eye Calibration Loaded from: %s", csv_path.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error finding package: %s", e.what());
      T_hand_eye_ = Eigen::Isometry3d::Identity();
    }
  }

  void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (got_cam_info_) return;
    fx_ = msg->k[0]; fy_ = msg->k[4];
    cx_ = msg->k[2]; cy_ = msg->k[5];
    got_cam_info_ = true;
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!got_cam_info_) return;

    geometry_msgs::msg::TransformStamped t_stamped;
    try {
      t_stamped = tf_buffer_->lookupTransform(
        this->get_parameter("robot_base_frame").as_string(),
        this->get_parameter("robot_tip_frame").as_string(),
        msg->header.stamp, rclcpp::Duration::from_seconds(0.05)); 
    } catch (tf2::TransformException &ex) { return; }

    Eigen::Isometry3d T_base_flange = tf2::transformToEigen(t_stamped);
    Eigen::Isometry3d T_base_cam = T_base_flange * T_hand_eye_;

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) { return; }

    cv::Mat& depth_img = cv_ptr->image;
    
    // --- [노이즈 제거: 필터링 로직] ---
    cv::Mat mask_x, mask_y, edge_mask;
    cv::absdiff(depth_img(cv::Rect(0, 0, depth_img.cols-1, depth_img.rows)), 
                depth_img(cv::Rect(1, 0, depth_img.cols-1, depth_img.rows)), mask_x);
    cv::absdiff(depth_img(cv::Rect(0, 0, depth_img.cols, depth_img.rows-1)), 
                depth_img(cv::Rect(0, 1, depth_img.cols, depth_img.rows-1)), mask_y);

    cv::threshold(mask_x, mask_x, 90, 255, cv::THRESH_BINARY);
    cv::threshold(mask_y, mask_y, 90, 255, cv::THRESH_BINARY);

    edge_mask = cv::Mat::zeros(depth_img.size(), CV_8UC1);
    mask_x.copyTo(edge_mask(cv::Rect(0, 0, mask_x.cols, mask_x.rows)));
    cv::bitwise_or(edge_mask, edge_mask, edge_mask); // 단순 합치기 예시 (사용자 로직 유지)

    cv::dilate(edge_mask, edge_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15)));
    depth_img.setTo(0, edge_mask);

    // --- [PCL 처리: 투영 및 변환 최적화] ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam(new pcl::PointCloud<pcl::PointXYZ>);
    double min_d = this->get_parameter("min_depth").as_double();
    double max_d = this->get_parameter("max_depth").as_double();

    for (int v = 0; v < msg->height; v += 2) {
      for (int u = 0; u < msg->width; u += 2) {
        uint16_t d_raw = depth_img.at<uint16_t>(v, u);
        if (d_raw == 0) continue;
        float z_c = d_raw / 1000.0f;
        if (z_c < min_d || z_c > max_d) continue;

        cloud_cam->push_back(pcl::PointXYZ((u - cx_) * z_c / fx_, (v - cy_) * z_c / fy_, z_c));
      }
    }

    if (cloud_cam->empty()) return;

    // 카메라 좌표계 -> 월드 좌표계 일괄 변환 (Isometry 사용으로 연산 효율 상승)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_cam, *cloud_world, T_base_cam.matrix().cast<float>());

    // Voxel + SOR 필터링
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_world);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_world);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_world);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_world);

    // --- [OctoMap 업데이트] ---
    octomap::Pointcloud octo_cloud;
    for (const auto& p : cloud_world->points) octo_cloud.push_back(p.x, p.y, p.z);
    
    Eigen::Vector3d sensor_origin = T_base_cam.translation();
    tree_->insertPointCloud(octo_cloud, octomap::point3d(sensor_origin.x(), sensor_origin.y(), sensor_origin.z()));

    // RViz 발행
    octomap_msgs::msg::Octomap map_msg;
    map_msg.header.frame_id = this->get_parameter("robot_base_frame").as_string();
    map_msg.header.stamp = msg->header.stamp;
    if (octomap_msgs::fullMapToMsg(*tree_, map_msg)) octomap_pub_->publish(map_msg);
  }

  void save_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    tree_->updateInnerOccupancy(); 
    std::string path = this->get_parameter("map_file_path").as_string();
    
    if (tree_->writeBinary(path)) {
        RCLCPP_INFO(this->get_logger(), "Map saved to %s", path.c_str());
        res->success = true;
        res->message = "Success";
        rclcpp::shutdown();
    } else {
        res->success = false;
        res->message = "Fail";
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticMapScanner>());
  rclcpp::shutdown();
  return 0;
}

