#pragma once

#include <ros/ros.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/plugin_utils.h>
#include <mujoco_ros/mujoco_env.h>

#include <random>

namespace mujoco_ros::sensors {

struct WrenchConfig
{
public:
	WrenchConfig() : frame_id(""){};
	WrenchConfig(std::string frame_id) : frame_id(std::move(frame_id)){};

	void setFrameId(const std::string &frame_id) { this->frame_id = frame_id; };
	void registerPub(const ros::Publisher &pub) { value_pub = pub; };

	std::string frame_id;
	ros::Publisher value_pub;
};

using WrenchConfigPtr = std::unique_ptr<WrenchConfig>;

class MujocoWrenchPlugin : public mujoco_ros::MujocoPlugin
{
public:
	~MujocoWrenchPlugin() override;

	// Overload entry point
	bool load(const mjModel *m, mjData *d) override;

	void reset() override;

	void lastStageCallback(const mjModel *model, mjData *data) override;

private:
	ros::NodeHandle sensors_nh_;
	void initSensors(const mjModel *model, mjData *data);
	std::mt19937 rand_generator = std::mt19937(std::random_device{}());
	std::normal_distribution<double> noise_dist;
	ros::Time last_publish_; 
	ros::Duration skip_duration_;

	std::map<std::string, WrenchConfigPtr> sensor_map_;

	ros::ServiceServer register_noise_model_server_;
};

const char *SENSOR_STRING[1];

} // namespace mujoco_ros::sensors
