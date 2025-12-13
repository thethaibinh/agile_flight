#include <rclcpp/rclcpp.hpp>

#include "dodgeros_msgs/msg/quad_state.hpp"
#include "envsim/visionsim.hpp"
#include "envsim_msgs/msg/obstacle_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "dodgeros/ros_eigen.hpp"

using namespace agi;

VisionSim::VisionSim()
  : rclcpp::Node("visionsim_node"), frame_id_(0) {
}

bool VisionSim::init() {
  // Logic subscribers
  reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "reset_sim", 1, std::bind(&VisionSim::resetCallback, this, std::placeholders::_1));

  // Publishers
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
  odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("groundtruth/odometry", 1);
  state_pub_ = this->create_publisher<dodgeros_msgs::msg::QuadState>("groundtruth/state", 1);

  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

  obstacle_pub_ = this->create_publisher<envsim_msgs::msg::ObstacleArray>("groundtruth/obstacles", 1);
  pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("groundtruth/pcl", 1);
  image_pub_ = it_->advertise("unity/image", 1);
  depth_pub_ = it_->advertise("unity/depth", 1);
  opticalflow_pub_ = it_->advertise("unity/opticalflow", 1);

  // TF broadcaster
  tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create RosPilot as a separate node
  ros_pilot_ = std::make_shared<RosPilot>();
  if (!ros_pilot_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create RosPilot instance");
    return false;
  }
  
  ros_pilot_->init();  // init() returns void
  
  if (!ros_pilot_->getQuadrotor(&quad_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get quadrotor from RosPilot");
    return false;
  }
  simulator_.updateQuad(quad_);
  simulator_.addModel(ModelInit{quad_});
  simulator_.addModel(ModelMotor{quad_});

  // Declare and get parameters
  this->declare_parameter<bool>("render", false);
  this->declare_parameter<std::string>("agi_param_dir", "");
  this->declare_parameter<std::string>("ros_param_dir", "");
  this->declare_parameter<double>("real_time_factor", 1.0);
  
  this->get_parameter("render", render_);
  this->get_parameter("agi_param_dir", agi_param_directory_);
  this->get_parameter("ros_param_dir", ros_param_directory_);
  this->get_parameter("real_time_factor", real_time_factor_);

  simulator_.addModel(ModelThrustTorqueSimple{quad_});
  simulator_.addModel(ModelRigidBody{quad_});

  const char* flightmare_path = getenv("FLIGHTMARE_PATH");
  if (!flightmare_path) {
    RCLCPP_ERROR(this->get_logger(), "Environment variable FLIGHTMARE_PATH is not set");
    return false;
  }
  std::string env_cfg_file =
    std::string(flightmare_path) +
    std::string("/flightpy/configs/vision/config.yaml");
  if (!(std::filesystem::exists(env_cfg_file))) {
    RCLCPP_ERROR(this->get_logger(), "Configuration file [%s] does not exist.",
              env_cfg_file.c_str());
    return false;
  }

  const char* planner_path = getenv("PLANNER_PATH");
  if (!planner_path) {
    RCLCPP_ERROR(this->get_logger(), "Environment variable PLANNER_PATH is not set");
    return false;
  }
  const std::string planner_cfg_file = std::string(planner_path) + "/configs/sim.yaml";
  if (!(std::filesystem::exists(planner_cfg_file))) {
    RCLCPP_ERROR(this->get_logger(), "Planning configuration file [%s] does not exist.",
              planner_cfg_file.c_str());
    return false;
  }

  // Load the config file
  YAML::Node env_cfg_node = YAML::LoadFile(env_cfg_file);
  YAML::Node planner_cfg_node = YAML::LoadFile(planner_cfg_file);
  // Get the frame names
  world_frame_name_ = planner_cfg_node["world_frame_name"].as<std::string>();
  vehicle_frame_name_ = planner_cfg_node["vehicle_frame_name"].as<std::string>();

  // Initialize depth covariance coefficients from config
  if (env_cfg_node["depth_covariance_coeffs"]) {
    cov_coeffs = env_cfg_node["depth_covariance_coeffs"].as<std::vector<double>>();
    if (cov_coeffs.size() != 6) {
      RCLCPP_WARN(this->get_logger(), "Expected 6 covariance coefficients, got %zu. Using defaults.", cov_coeffs.size());
      cov_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "depth_covariance_coeffs not found in config. Using default values.");
    cov_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  vision_env_ptr_ = std::make_unique<flightlib::VisionEnv>(env_cfg_file, 0);
  if (!vision_env_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create VisionEnv");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "VisionEnv created successfully");
  
  // Check if quadrotor was created in VisionEnv
  auto unity_quad = vision_env_ptr_->getQuadrotor();
  if (!unity_quad) {
    RCLCPP_ERROR(this->get_logger(), "VisionEnv quadrotor is null");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "VisionEnv quadrotor exists, cameras: %zu", unity_quad->getCameras().size());
  
  if (render_) {
    std::string camera_config = ros_param_directory_ + "/camera_config.yaml";
    RCLCPP_INFO(this->get_logger(), "Loading camera config from: %s", camera_config.c_str());
    if (!(std::filesystem::exists(camera_config))) {
      RCLCPP_ERROR(this->get_logger(), "Configuration file [%s] does not exist.",
                camera_config.c_str());
      return false;
    }
    
    // Wait for Unity to fully start before attempting connection
    RCLCPP_INFO(this->get_logger(), "Waiting 5 seconds for Unity to initialize...");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    try {
      YAML::Node cfg_node = YAML::LoadFile(camera_config);
      // Note: VisionEnv::init() already calls configCamera, so skip reconfiguring
      // vision_env_ptr_->configCamera(cfg_node);  
      RCLCPP_INFO(this->get_logger(), "Setting Unity render mode...");
      if (!vision_env_ptr_->setUnity(render_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set Unity mode - bridge may already exist or render is false");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Unity bridge created, connecting to Unity (make sure Flightmare is running)...");
      if (!vision_env_ptr_->connectUnity()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to Unity! Make sure Flightmare is running.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully connected to Unity");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during Unity setup: %s", e.what());
      return false;
    }
  }

  // Short wait to ensure Unity connection is stable
  if (render_) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  t_start_ = this->now();
  sim_thread_ = std::thread(&VisionSim::simLoop, this);
  
  RCLCPP_INFO(this->get_logger(), "VisionSim initialized successfully");
  return true;
}

VisionSim::~VisionSim() {
  if (sim_thread_.joinable()) sim_thread_.join();
  if (render_thread_.joinable()) render_thread_.join();
}

void VisionSim::resetCallback(const std_msgs::msg::Empty::SharedPtr msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "Resetting simulator!");
  QuadState reset_state;
  {
    const std::lock_guard<std::mutex> lock(sim_mutex_);
    simulator_.reset(false);
    simulator_.setCommand(Command(0.0, 0.0, Vector<3>::Zero()));
    simulator_.getState(&reset_state);
  }

  reset_state.t += t_start_.seconds();
}

void VisionSim::simLoop() {
  while (rclcpp::ok()) {
    rclcpp::Time t_start_sim = this->now();
    QuadState quad_state;
    {
      const std::lock_guard<std::mutex> lock(sim_mutex_);
      simulator_.getState(&quad_state);
    }

    // we add an offset to have realistic timestamps
    Scalar sim_time = quad_state.t;
    quad_state.t += t_start_.seconds();

    rosgraph_msgs::msg::Clock curr_time;
    curr_time.clock = rclcpp::Time(static_cast<int64_t>(quad_state.t * 1e9));
    clock_pub_->publish(curr_time);

    // sleep for 1ms
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    publishState(quad_state);

    ros_pilot_->getPilot().odometryCallback(quad_state);

    Command cmd = ros_pilot_->getCommand();
    cmd.t -= t_start_.seconds();
    if (cmd.valid()) {
      {
        const std::lock_guard<std::mutex> lock(sim_mutex_);
        simulator_.setCommand(cmd);
      }
    } else {
      Command zero_cmd;
      zero_cmd.t = sim_time;  // quad_state.t;
      zero_cmd.thrusts.setZero();
      {
        const std::lock_guard<std::mutex> lock(sim_mutex_);
        simulator_.setCommand(zero_cmd);
      }
    }
    {
      const std::lock_guard<std::mutex> lock(sim_mutex_);
      if (!simulator_.run(sim_dt_))
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Simulation failed!");
    }
    // Render here stuff
    if (render_) {
      if ((step_counter_ + 1) % render_every_n_steps_ == 0) {
        publishImages(quad_state);
        step_counter_ = 0;
      } else {
        step_counter_ += 1;
      }
    }

    // simulate dynamic obstacles (only if vision_env_ptr_ is valid)
    if (vision_env_ptr_) {
      std::vector<std::shared_ptr<flightlib::UnityObject>> dynamic_objects =
        vision_env_ptr_->getDynamicObjects();
      for (int i = 0; i < int(dynamic_objects.size()); i++) {
        dynamic_objects[i]->run(sim_dt_);
      }
      publishObstacles(quad_state);
    }

    Scalar sleep_time = 1.0 / real_time_factor_ * sim_dt_ -
                        (this->now() - t_start_sim).seconds();
    if (sleep_time > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
    }
  }
}

void VisionSim::publishState(const QuadState &state) {
  dodgeros_msgs::msg::QuadState msg_state;
  msg_state.header.frame_id = world_frame_name_;
  msg_state.header.stamp = rclcpp::Time(static_cast<int64_t>(state.t * 1e9));
  msg_state.t = state.t;
  msg_state.pose.position = toRosPoint(state.p);
  msg_state.pose.orientation = toRosQuaternion(state.q());
  msg_state.velocity.linear = toRosVector(state.v);
  msg_state.velocity.angular = toRosVector(state.w);
  msg_state.acceleration.linear = toRosVector(state.a);
  msg_state.acceleration.angular = toRosVector(state.tau);

  nav_msgs::msg::Odometry msg_odo;
  msg_odo.header.frame_id = world_frame_name_;
  msg_odo.header.stamp = rclcpp::Time(static_cast<int64_t>(state.t * 1e9));
  msg_odo.pose.pose = msg_state.pose;
  msg_odo.twist.twist = msg_state.velocity;

  odometry_pub_->publish(msg_odo);
  state_pub_->publish(msg_state);

  // Publish transform
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = world_frame_name_;
  transformStamped.child_frame_id = vehicle_frame_name_;
  transformStamped.transform.translation.x = state.p(0);
  transformStamped.transform.translation.y = state.p(1);
  transformStamped.transform.translation.z = state.p(2);
  transformStamped.transform.rotation.x = state.q().x();
  transformStamped.transform.rotation.y = state.q().y();
  transformStamped.transform.rotation.z = state.q().z();
  transformStamped.transform.rotation.w = state.q().w();
  transformStamped.header.stamp = rclcpp::Time(static_cast<int64_t>(state.t * 1e9));
  tfb_->sendTransform(transformStamped);
}

void VisionSim::publishObstacles(const QuadState &state) {
  flightlib::QuadState unity_quad_state;
  unity_quad_state.setZero();
  unity_quad_state.p = state.p.cast<flightlib::Scalar>();
  unity_quad_state.qx = state.qx.cast<flightlib::Scalar>();

  vision_env_ptr_->getQuadrotor()->setState(unity_quad_state);

  envsim_msgs::msg::ObstacleArray obstacle_msg;
  obstacle_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(state.t * 1e9));
  obstacle_msg.t = state.t;
  obstacle_msg.num = vision_env_ptr_->getNumDetectedObstacles();

  flightlib::Vector<> obstacle_state;
  const int obstacle_obs_dim = 4;
  obstacle_state.resize(obstacle_obs_dim * obstacle_msg.num);
  vision_env_ptr_->getObstacleState(obstacle_state);

  for (int i = 0; i < obstacle_msg.num; i++) {
    envsim_msgs::msg::Obstacle single_obstacle;
    single_obstacle.position.x = obstacle_state[obstacle_obs_dim * i];
    single_obstacle.position.y = obstacle_state[obstacle_obs_dim * i + 1];
    single_obstacle.position.z = obstacle_state[obstacle_obs_dim * i + 2];
    single_obstacle.scale = obstacle_state[obstacle_obs_dim * i + 3];

    obstacle_msg.obstacles.push_back(single_obstacle);
  }
  obstacle_pub_->publish(obstacle_msg);
}


void VisionSim::publishImages(const QuadState &state) {
  if (!vision_env_ptr_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "VisionEnv is not initialized, cannot publish images");
    return;
  }
  
  sensor_msgs::msg::Image::SharedPtr rgb_msg;
  frame_id_ += 1;
  // render the frame
  flightlib::QuadState unity_quad_state;
  unity_quad_state.setZero();
  unity_quad_state.p = state.p.cast<flightlib::Scalar>();
  unity_quad_state.qx = state.qx.cast<flightlib::Scalar>();

  std::shared_ptr<flightlib::Quadrotor> unity_quad =
    vision_env_ptr_->getQuadrotor();
  if (!unity_quad) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to get quadrotor from vision environment");
    return;
  }
  unity_quad->setState(unity_quad_state);


  vision_env_ptr_->updateUnity(frame_id_);

  // Warning, delay
  cv::Mat img, depth;

  // RGB Image
  auto cameras = unity_quad->getCameras();
  if (cameras.empty() || !cameras[0]) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Camera not available");
    return;
  }
  cameras[0]->getRGBImage(img);
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(static_cast<int64_t>(state.t * 1e9));
  rgb_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
  image_pub_.publish(rgb_msg);

  // Depth Image
  cameras[0]->getDepthMap(depth);
  if (depth.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Depth map is empty");
    return;
  }
  sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(header, "32FC1", depth).toImageMsg();
  depth_pub_.publish(depth_msg);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Published depth image");

  // Publish point cloud
  pointcloud_type* cloud (new pointcloud_type() );
  if (!cloud) {
    RCLCPP_ERROR(this->get_logger(), "Failed to allocate point cloud");
    return;
  }
  cloud->header.stamp     = rclcpp::Time(depth_msg->header.stamp).nanoseconds() / 1000;
  cloud->header.frame_id  = vehicle_frame_name_;
  cloud->is_dense         = false; //single point of view, 2d rasterized

  cloud->height = depth_msg->height;
  cloud->width = depth_msg->width;
  cloud->points.resize (cloud->height * cloud->width);
  
  auto intrinsic = cameras[0]->getIntrinsic();
  const double fx = intrinsic(0, 0);
  const double fy = intrinsic(1, 1);
  const double cx = intrinsic(0, 2);
  const double cy = intrinsic(1, 2);
  const float* depth_buffer = reinterpret_cast<const float*>(&depth_msg->data[0]);
  int depth_idx = 0;
  pointcloud_type::iterator pt_iter = cloud->begin ();
  for (int v = 0; v < (int)cloud->height; ++v)
  {
    for (int u = 0; u < (int)cloud->width; ++u, ++depth_idx, ++pt_iter)
    {
      point_type& pt = *pt_iter;
      float Z = depth_buffer[depth_idx];
      // Check for invalid measurements
      if (std::isnan (Z))
      {
        pt.x = pt.y = pt.z = Z;
      }
      else // Fill in XYZ
      {
        pt.y = -(u - cx) * Z / fx;
        pt.z = -(v - cy) * Z / fy;
        pt.x = Z;
      }
    }
  }
  sensor_msgs::msg::PointCloud2 cloudMessage;
  pcl::toROSMsg(*cloud, cloudMessage);
  pcl_pub_->publish(cloudMessage);
}

Eigen::Vector3d VisionSim::get_covariance_matrix(const Eigen::Vector3d& depth_point) const {
    double ca0 = cov_coeffs[0];
    double ca1 = cov_coeffs[1];
    double ca2 = cov_coeffs[2];
    double cl0 = cov_coeffs[3];
    double cl1 = cov_coeffs[4];
    double cl2 = cov_coeffs[5];

    double sigma_a = ca0 + ca1 * depth_point.z() + ca2 * depth_point.z() * depth_point.z();
    double sigma_lx = cl0 + cl1 * depth_point.z() + cl2 * depth_point.x();
    double sigma_ly = cl0 + cl1 * depth_point.z() + cl2 * depth_point.y();
    return Eigen::Vector3d(sigma_lx, sigma_ly, sigma_a);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<VisionSim>();
  if (!node->init()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize VisionSim");
    rclcpp::shutdown();
    return 1;
  }

  // Use MultiThreadedExecutor to handle both VisionSim and RosPilot nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node->getRosPilot());
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}