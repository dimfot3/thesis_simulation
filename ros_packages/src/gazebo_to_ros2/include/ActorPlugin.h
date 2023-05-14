#ifndef SYSTEM_PLUGIN_COMMANDACTOR_HH_
#define SYSTEM_PLUGIN_COMMANDACTOR_HH_

#include <chrono>
#include <gz/sim/System.hh>
#include <iostream>
#include <algorithm>
#include <flann/flann.hpp>
#include <gz/sim/components/Actor.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"

namespace command_actor
{
  struct Pose {
    float time;
    float x;
    float y;
    float rz;
  };

  class CommandActor:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public: CommandActor();

    public: ~CommandActor() override;
    public: void Configure(const gz::sim::Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) final;
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
    private: std::shared_ptr<rclcpp::Node> node;
    private: std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    private: void publish_actor_frame(Pose &pose);

    private: gz::sim::Entity entity;  // the actor's entity
    std::vector<Pose> poses;        // saved the trajectroy poses
    std::vector<float> time_arr;    // used to create a KDtree of times 
    private: double started_sec;    // used for loop trajectory
    private: double last_update;    // used to get time between updates
    flann::Index<flann::L2<float>> *ktree; // time kdtree for fast search
    float velocity, angular_velocity, pace;
    std::string actor_name;
    float anim_t;
    bool anim_bool;
  };
}

#endif
