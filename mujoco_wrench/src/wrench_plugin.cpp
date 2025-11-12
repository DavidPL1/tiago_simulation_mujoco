/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2023, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/* Authors: David P. Leins */

#include <mujoco_wrench/wrench_plugin.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/WrenchStamped.h>
#include <mujoco_ros_msgs/ScalarStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mujoco_ros/mujoco_env.h>

namespace mujoco_ros::sensors {

MujocoWrenchPlugin::~MujocoWrenchPlugin()
{
	sensor_map_.clear();
}

bool MujocoWrenchPlugin::load(const mjModel *model, mjData *data)
{
	last_publish_ = ros::Time(0);
	ROS_INFO_NAMED("wrench", "Loading wrench plugin ...");
	SENSOR_STRING[mjSENS_FORCE]          = "force";

	std::string sensors_namespace;
	if (rosparam_config_.hasMember("namespace")) {
		sensors_namespace = static_cast<std::string>(rosparam_config_["namespace"]);
	}
	if (rosparam_config_.hasMember("hz")) {
		int hz = static_cast<int>(rosparam_config_["hz"]);
		skip_duration_ = ros::Duration(1.0 / hz);
	} else {
		skip_duration_ = ros::Duration(0.01);
	}
	ROS_INFO_STREAM_NAMED("wrench", "Skipping" << skip_duration_);


	sensors_nh_ = ros::NodeHandle("/" + sensors_namespace);

	noise_dist = std::normal_distribution<double>(0.0, 1.0);
	initSensors(model, data);
	ROS_INFO_NAMED("wrench", "All sensors initialized");

	return true;
}

void MujocoWrenchPlugin::lastStageCallback(const mjModel *model, mjData *data)
{
	std::string sensor_name;

	if (ros::Time::now() < ( last_publish_ + skip_duration_)) {
		return;
	}

	int adr, type, noise_idx;
	mjtNum cutoff;
	double noise = 0.0;

	for (int n = 0; n < model->nsensor; n++) {
		adr       = model->sensor_adr[n];
		type      = model->sensor_type[n];
		cutoff    = (model->sensor_cutoff[n] > 0 ? model->sensor_cutoff[n] : 1);
		noise_idx = 0;

		if (model->names[model->name_sensoradr[n]]) {
			sensor_name = mj_id2name(const_cast<mjModel *>(model), mjOBJ_SENSOR, n);
		} else {
			continue;
		}

		if (sensor_map_.find(sensor_name) == sensor_map_.end())
			continue;

		WrenchConfigPtr &config = sensor_map_[sensor_name];

		if (type == mjSENS_FORCE) {
			geometry_msgs::WrenchStamped msg;
			msg.header.frame_id = config->frame_id;
			msg.header.stamp    = ros::Time::now();

			msg.wrench.force.x = static_cast<float>(data->sensordata[adr] / cutoff);
			msg.wrench.force.y = static_cast<float>(data->sensordata[adr + 1] / cutoff);
			msg.wrench.force.z = static_cast<float>(data->sensordata[adr + 2] / cutoff);

			config->value_pub.publish(msg);
		}


	}

	last_publish_ = ros::Time::now();
}

void MujocoWrenchPlugin::initSensors(const mjModel *model, mjData *data)
{
	std::string sensor_name, site, frame_id;
	for (int n = 0; n < model->nsensor; n++) {
		int site_id   = model->sensor_objid[n];
		int parent_id = model->site_bodyid[site_id];
		int type      = model->sensor_type[n];

		// Skip user sensors because handling is unknown and should be done in extra plugin
		if (type == mjSENS_USER) {
			ROS_INFO_STREAM_NAMED("sensors", "Skipping USER sensor");
			continue;
		}

		site = mj_id2name(const_cast<mjModel *>(model), model->sensor_objtype[n], site_id);

		if (model->names[model->name_sensoradr[n]]) {
			sensor_name = mj_id2name(const_cast<mjModel *>(model), mjOBJ_SENSOR, n);
		} else {
			ROS_WARN_STREAM_NAMED("sensors",
			                      "Sensor name resolution error. Skipping sensor of type " << type << " on site " << site);
			continue;
		}

		// Global frame sensors
		bool global_frame = false;
		frame_id          = "world";
		WrenchConfigPtr config;
	
		// Check if sensor is in global frame and already setup
		if (global_frame || frame_id != "world") {
			ROS_DEBUG_STREAM_NAMED("wrench", "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
			                                                       << frame_id << ") of type " << SENSOR_STRING[type]);
			continue;
		}

		frame_id = mj_id2name(const_cast<mjModel *>(model), mjOBJ_BODY, parent_id);
		ROS_DEBUG_STREAM_NAMED("wrench", "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
		                                                       << frame_id << ") of type " << SENSOR_STRING[type]);

		switch (type) {
			case mjSENS_FORCE:
				config = std::make_unique<WrenchConfig>(frame_id);
				config->registerPub(sensors_nh_.advertise<geometry_msgs::WrenchStamped>(sensor_name, 1, true));
				sensor_map_[sensor_name] = std::move(config);
				break;

			default:
				ROS_WARN_STREAM_NAMED("sensors", "Sensor of type '" << type << "' (" << sensor_name
				                                                    << ") is unknown! Cannot publish to ROS");
				break;
		}
	}
}

// Nothing to do on reset
void MujocoWrenchPlugin::reset(){};

} // namespace mujoco_ros::sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::MujocoWrenchPlugin, mujoco_ros::MujocoPlugin)
