#include <unistd.h>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include "RectangularPyramidPlanner/SteeringPlanner.hpp"
#include "RapidQuadcopterTrajectories/SteeringTrajectoryGenerator.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include "dodgeros_msgs/QuadState.h"
#include "CommonMath/Rotation.hpp"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

// ROS TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"

//this is a global definition of the points to be used
//changes to omit color would need adaptations in 
//the visualization too
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <visualization_msgs/Marker.h>
namespace sm = sensor_msgs;
typedef pcl::PointXYZ point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;

using namespace sensor_msgs;
using namespace message_filters;
using namespace CommonMath;
using namespace RapidQuadrocopterTrajectoryGenerator;
using namespace RectangularPyramidPlanner;

class PlannerNode
{
public:
  PlannerNode() :
    to_world_tf2(to_world_buffer),
    to_camera_tf2(to_camera_buffer),  
    camera_frame("camera"), 
    world_frame("world"),
    steering_sent(false),
    sampling_mode(0),
    last_generated_time(0)
  {
    trajectoty_pub = n_.advertise<geometry_msgs::Pose>("/trajectory", 1);
    point_cloud_pub = n_.advertise<sm::PointCloud2>("/cloud_out", 10);
    visual_pub = n_.advertise<visualization_msgs::Marker>("/visualization", 10);
    image_sub = n_.subscribe("/kingfisher/dodgeros_pilot/unity/depth", 1, &PlannerNode::msgCallback, this);
    sampling_mode_sub = n_.subscribe("/sampling_mode", 1, &PlannerNode::sampling_mode_callback, this);
    state_sub = n_.subscribe("/kingfisher/dodgeros_pilot/state", 1, &PlannerNode::state_callback, this);
  }

  void sampling_mode_callback(const std_msgs::Int8::ConstPtr& msg)
  {
    sampling_mode = msg->data;
  }

  void state_callback(const dodgeros_msgs::QuadState& state)
  {
    _state = state;
  }

  // Callback for planning when a new depth image comes
  void msgCallback(const sensor_msgs::ImageConstPtr& depth_msg) 
  {
    cv_bridge::CvImageConstPtr cv_img_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
    cv::Mat depth_mat;
    // RAPPIDS use depth images in 16UC1 format
    cv_img_ptr->image.convertTo(depth_mat, CV_16UC1, 65535);
    
    double width = depth_mat.cols; 
    double height = depth_mat.rows;
    double fov = 90.0f;
    double cx = width / 2.0f;
    double cy = height / 2.0f;
    // double fx = (width / 2.0f) / tan((M_PI * fov / 180.0f) / 2.0f);
    // double fy = (height / 2.0f) / tan((M_PI * fov / 180.0f) / 2.0f);
    // We use camera intrinsics matrix value from Flightmare 
    // with FoV 90 degrees and resolution 320x240
    double fx, fy;
    fx = fy = 130.839769;

    // pixelValues = depth / depth_scale
    // e.g. 1 meter = 1000 pixel value for depth_scale = 0.001
    // depth_scale in 32FC1 is 100, 
    // then in 16UC1, depth_scale is 100/(2^16-1) = 0.00152590218f
    SteeringPlanner planner(depth_mat, 0.0015259f,
                              fx, cx,
                              cy,
                              0.26f,
                              0.55f,
                              0.65f);

    // Rotation matrix to express a vector in body (camera-fixed) frame 
    // to world frame. The matrix is formed from the quaternion 
    // that express the body attitude in world frame. 
    // The transformation is simply follow:
    // vector_in_world_frame = rotation * vector_in_body_frame
    // vector_in_body_frame = rotation / vector_in_world_frame
    Vec3 vel, accel;
    {
        const std::lock_guard<std::mutex> lock(state_mutex_);
        Rotation rotation(_state.pose.orientation.w, 
                      _state.pose.orientation.x, 
                      _state.pose.orientation.y, 
                      _state.pose.orientation.z);
        Vec3 temp = rotation / Vec3(_state.velocity.linear.x,
                                    _state.velocity.linear.y,
                                    _state.velocity.linear.z);
        vel = Vec3(-temp.y, -temp.z, temp.x);
        temp = rotation / Vec3(_state.acceleration.linear.x,
                                _state.acceleration.linear.y,
                                _state.acceleration.linear.z);
        accel = Vec3(-temp.y, -temp.z, temp.x);
    }

    SteeringTrajectoryGenerator traj(
        Vec3(0,0,0),
        vel,
        accel,
        // Vec3(0,0,0),
        // Vec3(0,0,0),
        Vec3(0, 9.8066f, 0));
    
    // Lookup for transforms in the TF2 transforming tree
    geometry_msgs::TransformStamped transform_to_camera, transform_to_world;
    try{
      transform_to_world = to_world_buffer.lookupTransform(world_frame,
                                        camera_frame,
                                        ros::Time(0));
      transform_to_camera = to_camera_buffer.lookupTransform(camera_frame,
                                        world_frame,
                                        ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    
    // Transform the coordinate of goal_in_world_frame to 
    // the coordinate of goal_in_camera_frame
    geometry_msgs::PointStamped goal_in_camera_frame, goal_in_world_frame;
    goal_in_world_frame.header.frame_id = world_frame;
    goal_in_world_frame.header.stamp = ros::Time::now();
    goal_in_world_frame.point.x = 18; 
    goal_in_world_frame.point.y = 0; 
    goal_in_world_frame.point.z = 5;
    try 
    {
      tf2::doTransform(goal_in_world_frame, 
                      goal_in_camera_frame,
                      transform_to_camera);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
    
    // Build exploration_vector from the coordinate of goal_in_camera_frame
    Vec3 exploration_vector(-goal_in_camera_frame.point.y, 
                              -goal_in_camera_frame.point.z, 
                              goal_in_camera_frame.point.x);
    
    // Stop planning when the goal is less than 1.5m close.
    if (exploration_vector.GetNorm2() < 2)
    {
      // Publish generated trajectory                                         
      geometry_msgs::Pose trajectory_msg;
      trajectory_msg.position.x = goal_in_world_frame.point.x;
      trajectory_msg.position.y = goal_in_world_frame.point.y;
      trajectory_msg.position.z = goal_in_world_frame.point.z;
      trajectory_msg.orientation.w = 1;
      // orientation.x = 0 means we are sending conventional trajectories
      // orientation.z = 0 means the steering value = 0
      trajectory_msg.orientation.x = 0;
      trajectory_msg.orientation.z = 0;
      trajectoty_pub.publish(trajectory_msg);
      steering_sent = false;
      last_generated_time = depth_msg->header.stamp.toSec();
      return;
    }
    
    // Find the fastest trajectory candidate
    if (!planner.FindFastestTrajRandomCandidates(sampling_mode, 
                                                traj, 0.003,
                                                exploration_vector))
    {
      // We only sent steering commands when we could not find 
      // any feasible trajectory for 1 second in a row.
      if (((depth_msg->header.stamp.toSec() - last_generated_time) < 1)
          || steering_sent)
      {
        return;
      }
      // Publish generated trajectory                                         
      geometry_msgs::Pose trajectory_msg;
      geometry_msgs::PointStamped traj_in_world_frame, traj_in_camera_frame;
      // Build traj_in_camera_frame from fastest trajectory found (traj)
      // and transforming it into traj_in_world_frame
      traj_in_camera_frame.header.frame_id = camera_frame;
      traj_in_camera_frame.header.stamp = ros::Time::now();
      traj_in_camera_frame.point.x = traj.GetPosition(traj.GetFinalTime()).z; 
      traj_in_camera_frame.point.y = -traj.GetPosition(traj.GetFinalTime()).x; 
      traj_in_camera_frame.point.z = -traj.GetPosition(traj.GetFinalTime()).y;
      try 
      {
        tf2::doTransform(traj_in_camera_frame, 
                          traj_in_world_frame,
                          transform_to_world);
      }
      catch (tf2::TransformException &ex) 
      {
        ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
      }
      // Building ROS trajectory_msg from transformed traj_in_world_frame
      trajectory_msg.position.x = traj_in_world_frame.point.x;
      trajectory_msg.position.y = traj_in_world_frame.point.y;
      trajectory_msg.position.z = traj_in_world_frame.point.z;
      // For convenience purposes, orientation.x = 1 means we are sending steering commands
      // we use orientation.z for steering value
      trajectory_msg.orientation.w = 1;
      trajectory_msg.orientation.x = 1;
      trajectory_msg.orientation.z = planner.get_steering();
      trajectoty_pub.publish(trajectory_msg);
      steering_sent = true;
      return;
    }
    // Publish generated trajectory                                         
    geometry_msgs::Pose trajectory_msg;
    geometry_msgs::PointStamped traj_in_world_frame, traj_in_camera_frame;
    traj_in_camera_frame.header.frame_id = camera_frame;
    traj_in_camera_frame.header.stamp = ros::Time::now();
    traj_in_camera_frame.point.x = traj.GetPosition(traj.GetFinalTime()).z; 
    traj_in_camera_frame.point.y = -traj.GetPosition(traj.GetFinalTime()).x; 
    traj_in_camera_frame.point.z = -traj.GetPosition(traj.GetFinalTime()).y;
    try 
    {
      tf2::doTransform(traj_in_camera_frame, 
                        traj_in_world_frame,
                        transform_to_world);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
    trajectory_msg.position.x = traj_in_world_frame.point.x;
    trajectory_msg.position.y = traj_in_world_frame.point.y;
    trajectory_msg.position.z = traj_in_world_frame.point.z;
    trajectory_msg.orientation.w = 1;
    // orientation.x = 0 means we are sending conventional trajectories
    // orientation.z = 0 means the steering value = 0
    trajectory_msg.orientation.x = 0;
    trajectory_msg.orientation.z = 0;
    trajectoty_pub.publish(trajectory_msg);
    steering_sent = false;
    last_generated_time = depth_msg->header.stamp.toSec();
  }

private:
  std::string camera_frame, world_frame;
  tf2_ros::Buffer to_world_buffer, to_camera_buffer;
  tf2_ros::TransformListener to_world_tf2, to_camera_tf2;
  ros::NodeHandle n_;
  ros::Publisher trajectoty_pub, point_cloud_pub, visual_pub;
  ros::Subscriber sampling_mode_sub, image_sub, state_sub;
  dodgeros_msgs::QuadState _state;
  bool steering_sent; 
  int8_t sampling_mode;
  double last_generated_time;
  std::mutex state_mutex_;
};
