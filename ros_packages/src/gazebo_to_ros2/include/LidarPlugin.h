#ifndef SYSTEM_PLUGIN_LIDARPLUGIN_HH_
#define SYSTEM_PLUGIN_LIDARPLUGIN_HH_

#include <chrono>
#include <gz/sim/System.hh>
#include <iostream>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"

namespace lidar_plugin
{
  struct Pose {
    float time;
    float x;
    float y;
    float z;
  };

  class LidarPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public: LidarPlugin();

    public: ~LidarPlugin() override;
    public: void Configure(const gz::sim::Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) final;
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
    private: gz::sim::Entity entity;  // lidar entity
    std::string lidar_name;
    Pose cur_lidar_pos;
    private: std::shared_ptr<rclcpp::Node> node;
    int counter = 0;
    float last_update = 0;
    public: void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    public: void threah_exec();
    public:std::thread spin_thread;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription;
  };
}

#endif