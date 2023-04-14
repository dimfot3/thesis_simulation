#include <iostream>
#include <fstream>
#include "ActorPlugin.h"
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/World.hh>
#include <flann/flann.hpp>
#include <gz/sim/Util.hh>
#include <tf2/LinearMath/Quaternion.h>
using namespace gz;
using namespace sim;
using namespace systems;


GZ_ADD_PLUGIN(
    command_actor::CommandActor,
    gz::sim::System,
    command_actor::CommandActor::ISystemConfigure,
    command_actor::CommandActor::ISystemPreUpdate)
using namespace command_actor;


CommandActor::CommandActor()
{
}

CommandActor::~CommandActor()
{
}

void CommandActor::publish_message(Pose &pose)
{
	tf2::Quaternion quat;
    quat.setRPY(0, 0, pose.rz); // Set rotation in Roll-Pitch-Yaw
	auto message = geometry_msgs::msg::PoseStamped();
	message.header.stamp = this->node->now();
	message.header.frame_id = "world";
	message.pose.position.x = pose.x;
	message.pose.position.y = pose.y;
	message.pose.position.z = 1.0;
	message.pose.orientation.x = quat.x();
	message.pose.orientation.y = quat.y();
	message.pose.orientation.z = quat.z();
	message.pose.orientation.w = quat.w();
	this->publisher->publish(message);
}

void CommandActor::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
	// Load trajectory path
	this->entity = _entity;
	gzmsg << "Command actor for entity [" << _entity << "]" << std::endl;
	// enableComponent<gz::sim::components::WorldPose>(_ecm, this->entity, true);
	auto sdfClone = _sdf->Clone();
	std::string traj_path = sdfClone->GetElement("traj")->Get<std::string>("path");
	this->velocity = sdfClone->GetElement("traj")->Get<float>("velocity");
	std::ifstream file(traj_path, std::ios::in | std::ios::binary);
	Pose temp_pose;
	while (file.read((char*)&temp_pose, sizeof(temp_pose))) {
		poses.push_back(temp_pose);
		time_arr.push_back(temp_pose.time);
	}
	gzmsg << "Loaded trajectory path: " << traj_path<<std::endl;
	// Create time Kdtree
	flann::Matrix<float> dataset(time_arr.data(), time_arr.size(), 1);
	ktree = new flann::Index<flann::L2<float>>(dataset, flann::KDTreeIndexParams(4));
	ktree->buildIndex();

	// set up custom animation time from this plugin
	auto actorComp = _ecm.Component<gz::sim::components::Actor>(_entity);
	auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(_entity);
	if (nullptr == animTimeComp)
	{
		_ecm.CreateComponent(_entity, gz::sim::components::AnimationTime());
	}

	// Initialize the pose and override SDF trajectory path
	math::Pose3d initialPose;
	auto poseComp = _ecm.Component<gz::sim::components::Pose>(_entity);
	if (nullptr == poseComp)
	{
		_ecm.CreateComponent(_entity, gz::sim::components::Pose(
			gz::math::Pose3d::Zero));
	}
	else
	{
		initialPose = poseComp->Data();
		auto newPose = initialPose;
		newPose.Pos().X(0);
		newPose.Pos().Y(0);
		*poseComp = gz::sim::components::Pose(newPose);
	}
	auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(_entity);
	if (nullptr == trajPoseComp)
	{
		initialPose.Pos().Z(1.0);
		_ecm.CreateComponent(_entity, components::TrajectoryPose(initialPose));
	}
	// initilize trajectory time and time from last update
	started_sec = 0;
	last_update = 0;
	// initilize ROS2
	rclcpp::init(0, 0);
	node = std::make_shared<rclcpp::Node>("pose_publisher");
	publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("humans_poses", 10);
}


void CommandActor::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
	// geting duration from last update and update time
	float sec = (float) ((double) std::chrono::duration_cast<std::chrono::milliseconds>(
		_info.simTime).count() / 1000.0);
	float dur = sec - this->last_update;
	last_update = sec;
	float query_time = sec - started_sec; 
	
	// getting next pose
	flann::Matrix<float> query_mat(&query_time, 1, 1);
	flann::Matrix<int> indices(new int[1], 1, 1);
	flann::Matrix<float> dists(new float[1], 1, 1);
	ktree->knnSearch(query_mat, indices, dists, 1, flann::SearchParams(128));
	gz::math::Pose3d next_pose(poses[indices[0][0]].x, poses[indices[0][0]].y, 0, 0, 0, poses[indices[0][0]].rz);
	if(indices[0][0] == poses.size() - 1)
		started_sec = sec;

	// world pose
	auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(this->entity);
	auto actorPose = trajPoseComp->Data();
	auto initialPose = actorPose;
	// Current target
	auto targetPose = next_pose;
	// Direction to target
	auto dir = targetPose.Pos() - actorPose.Pos();
	dir.Z(0);
	dir.Normalize();	//unit vector to the target direction
	// Towards target
	math::Angle yaw = atan2(dir.Y(), dir.X());
	yaw.Normalize();
	actorPose.Pos() += dir * this->velocity * dur;
	actorPose.Rot() = math::Quaterniond(0, 0, yaw.Radian());
	// Distance traveled is used to coordinate motion with the walking animation
	double distanceTraveled = (actorPose.Pos() - initialPose.Pos()).Length();
	// Update actor root pose
	*trajPoseComp = components::TrajectoryPose(actorPose);
	// Mark as a one-time-change so that the change is propagated to the GUI
	_ecm.SetChanged(this->entity,
		components::TrajectoryPose::typeId, ComponentState::OneTimeChange);
	// Coordinate animation with trajectory
	auto animTimeComp = _ecm.Component<components::AnimationTime>(
		this->entity);
	auto animTime = animTimeComp->Data() +
	std::chrono::duration_cast<std::chrono::steady_clock::duration>(
	std::chrono::duration<double>(distanceTraveled /
	this->velocity));
	*animTimeComp = components::AnimationTime(animTime);
	// Mark as a one-time-change so that the change is propagated to the GUI
	_ecm.SetChanged(this->entity,
		components::AnimationTime::typeId, ComponentState::OneTimeChange);
	// publish new pose to ROS2 topic
	command_actor::Pose msgPose{(float)sec, (float)actorPose.Pos().X(), (float)actorPose.Pos().Y(), (float)actorPose.Rot().Z()};
	this->publish_message(msgPose);
}