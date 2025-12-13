#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <mutex>
#include <thread>

// -- ROS 2
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

// PCL
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
namespace sm = sensor_msgs::msg;
typedef pcl::PointXYZ point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;

#include <filesystem>

// -- agilicious
#include "dodgelib/base/parameter_base.hpp"
#include "dodgelib/simulator/model_init.hpp"
#include "dodgelib/simulator/model_motor.hpp"
#include "dodgelib/simulator/model_rigid_body.hpp"
#include "dodgelib/simulator/model_thrust_torque_simple.hpp"
#include "dodgelib/simulator/quadrotor_simulator.hpp"
#include "dodgelib/utils/timer.hpp"
#include "dodgeros/ros_pilot.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// flightlib
#include "flightlib/envs/vision_env/vision_env.hpp"

// Messages
#include "dodgeros_msgs/msg/quad_state.hpp"
#include "envsim_msgs/msg/obstacle_array.hpp"

namespace agi {

class VisionSim : public rclcpp::Node {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VisionSim();
  ~VisionSim();
  
  bool init();
  
  // Getter for RosPilot node (needed for executor)
  std::shared_ptr<RosPilot> getRosPilot() const { return ros_pilot_; }

 private:
  void resetCallback(const std_msgs::msg::Empty::SharedPtr msg);

  void simLoop();
  void publishState(const QuadState& state);
  void publishImages(const QuadState& state);
  void publishObstacles(const QuadState& state);
  Eigen::Vector3d get_covariance_matrix(const Eigen::Vector3d& depth_point) const;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<dodgeros_msgs::msg::QuadState>::SharedPtr state_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;

  rclcpp::Publisher<envsim_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher opticalflow_pub_;

  Quadrotor quad_;
  QuadrotorSimulator simulator_;
  std::shared_ptr<RosPilot> ros_pilot_;
  Scalar camera_dt_ = 0.04;  // 20 Hz. Should be a multiple of sim_dt_
  Scalar sim_dt_ = 0.01;
  int render_every_n_steps_ = camera_dt_ / sim_dt_;
  int step_counter_ = 0;
  Scalar real_time_factor_ = 1.0;
  bool render_ = false;
  rclcpp::Time t_start_;

  std::string agi_param_directory_;
  std::string ros_param_directory_;
  std::string world_frame_name_;
  std::string vehicle_frame_name_;

  // flightmare vision environment
  std::unique_ptr<flightlib::VisionEnv> vision_env_ptr_;
  flightlib::FrameID frame_id_;

  // -- Race tracks
  Vector<3> start_pos_;
  Vector<3> goal_pos_;

  std::mutex sim_mutex_;
  std::thread sim_thread_;
  std::thread render_thread_;

  // covariance parameters
  std::vector<double> cov_coeffs;
};

}  // namespace agi