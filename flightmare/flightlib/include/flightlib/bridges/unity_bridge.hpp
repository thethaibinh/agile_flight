//
// This bridge was originally from FlightGoggles.
// We made several changes on top of it.
//
#pragma once

// std libs
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <unordered_map>


// opencv
#include <opencv2/imgproc/types_c.h>

// Include ZMQ bindings for communications with Unity.
#include <zmqpp/zmqpp.hpp>

// flightlib
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/math.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/unity_camera.hpp"
#include "flightlib/objects/unity_object.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using json = nlohmann::json;

namespace flightlib {

class UnityBridge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // constructor & destructor
  UnityBridge();
  ~UnityBridge(){};

  // connect function
  bool connectUnity(const SceneID scene_id);
  bool disconnectUnity(void);

  // public get functions
  bool getRender(const FrameID sent_frame_id);
  FrameID handleOutput(const FrameID sent_frame_id);
  bool getPointCloud(PointCloudMessage_t &pointcloud_msg,
                     Scalar time_out = 600.0);

  // public set functions
  bool setScene(const SceneID &scene_id);
  bool setObjectCSV(const std::string &csv_file);

  // add object
  bool addQuadrotor(std::shared_ptr<Quadrotor> quad);
  bool addCamera(std::shared_ptr<UnityCamera> unity_camera);
  bool addStaticObject(std::shared_ptr<UnityObject> unity_object);
  bool addDynamicObject(std::shared_ptr<UnityObject> unity_object);
  void setRenderOffset(const Ref<Vector<3>> render_offset);

  // public auxiliary functions
  inline void setPubPort(const std::string &pub_port) { pub_port_ = pub_port; };
  inline void setSubPort(const std::string &sub_port) { sub_port_ = sub_port; };

  // create unity bridge
  static std::shared_ptr<UnityBridge> getInstance(void) {
    static std::shared_ptr<UnityBridge> bridge_ptr =
      std::make_shared<UnityBridge>();
    return bridge_ptr;
  };

 private:
  bool initializeConnections(void);

  //
  SettingsMessage_t settings_;
  PubMessage_t pub_msg_;
  Logger logger_{"UnityBridge"};

  std::vector<std::shared_ptr<Quadrotor>> unity_quadrotors_;
  std::vector<std::shared_ptr<RGBCamera>> rgb_cameras_;
  std::vector<std::shared_ptr<UnityObject>> static_objects_;
  std::vector<std::shared_ptr<UnityObject>> dynamic_objects_;

  // ZMQ variables and functions
  std::string client_address_;
  std::string pub_port_;
  std::string sub_port_;
  zmqpp::context context_;
  zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
  zmqpp::socket sub_{context_, zmqpp::socket_type::subscribe};
  bool sendInitialSettings(void);
  bool handleSettings(void);

  // timing variables
  int64_t num_frames_;
  int64_t last_downloaded_utime_;
  int64_t last_download_debug_utime_;
  int64_t u_packet_latency_;

  // axuiliary variables
  const int max_output_request_{100};
  const Scalar unity_connection_time_out_{60.0};
  bool unity_ready_{false};
};

}  // namespace flightlib