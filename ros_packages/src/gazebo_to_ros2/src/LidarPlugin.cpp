#include <iostream>
#include <fstream>
#include "LidarPlugin.h"
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Util.hh>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_msgs/msg/tf_message.hpp"
#include <random>

// Initialize random number generator with a seed (e.g. current time)
std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());

using namespace gz;
using namespace sim;
using namespace systems;


GZ_ADD_PLUGIN(
    lidar_plugin::LidarPlugin,
    gz::sim::System,
    lidar_plugin::LidarPlugin::ISystemConfigure,
    lidar_plugin::LidarPlugin::ISystemPreUpdate)
using namespace lidar_plugin;


LidarPlugin::LidarPlugin()
{
	
}

LidarPlugin::~LidarPlugin()
{
}

void LidarPlugin::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
	std::string frame = msg->transforms[0].child_frame_id.c_str();
	if(frame == this->lidar_name)
	{
		cur_lidar_pos.time = msg->transforms[0].header.stamp.sec + msg->transforms[0].header.stamp.nanosec * 1e-9;
		cur_lidar_pos.x = msg->transforms[0].transform.translation.x;
		cur_lidar_pos.y = msg->transforms[0].transform.translation.y;
		cur_lidar_pos.z = msg->transforms[0].transform.translation.z;
	}
		
}

void LidarPlugin::threah_exec()
{
	rclcpp::spin(this->node);
}	

void LidarPlugin::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
	// Load trajectory path
	this->entity = _entity;
	auto sdfClone = _sdf->Clone();
	this->lidar_name = sdfClone->GetElement("info")->Get<std::string>("name");
	if(!rclcpp::ok())
		rclcpp::init(0, 0);
	node = std::make_shared<rclcpp::Node>("lidar_tf_sub_" + lidar_name);
	this->tf_subscription =
  	node->create_subscription<tf2_msgs::msg::TFMessage>("/tf",
    10, std::bind(&LidarPlugin::tf_callback, this, std::placeholders::_1));
	spin_thread = std::thread(std::bind(&LidarPlugin::threah_exec, this));
}


void LidarPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
	float sec = (float) ((double) std::chrono::duration_cast<std::chrono::milliseconds>(
		_info.simTime).count() / 1000.0);
	float dur = sec - this->last_update;
	last_update = sec;
	if(dur == 0)
		return;
	auto poseComp = _ecm.Component<gz::sim::components::Pose>(
        this->entity);
	auto poseData = poseComp->Data();
	float epsilon = 0.000001;
	if(std::abs(poseData.Pos().X() - cur_lidar_pos.x) > epsilon | 
	std::abs(poseData.Pos().Y() - cur_lidar_pos.y) > epsilon | std::abs(poseData.Pos().Z() - cur_lidar_pos.z) > epsilon)
	{
		poseData.Pos().X(cur_lidar_pos.x);
		poseData.Pos().Y(cur_lidar_pos.y);
		poseData.Pos().Z(cur_lidar_pos.z);
		*poseComp = gz::sim::components::Pose(poseData);
		_ecm.SetChanged(this->entity,
			components::Pose::typeId, ComponentState::OneTimeChange);
	}
}
