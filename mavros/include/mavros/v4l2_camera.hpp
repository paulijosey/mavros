// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef V4L2_CAMERA__V4L2_CAMERA_HPP_
#define V4L2_CAMERA__V4L2_CAMERA_HPP_

#include "v4l2_camera_device.hpp"

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <thread>

#include "ros/ros.h"
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opt_flow_msgs/opt_flow.h>

#include "visibility_control.h"
#include "of_cam_config.h"

class V4L2Camera
{
public:
	//                        _                   _
	//     ___ ___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __
	//    / __/ _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__|
	//   | (_| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |
	//    \___\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|
	V4L2Camera();
	V4L2Camera(ros::NodeHandle nh);
	~V4L2Camera();

	void init(ros::NodeHandle nh);
	void capture();

private:
	//    ____   ___  ____
	//   |  _ \ / _ \/ ___|
	//   | |_) | | | \___ \ 
	//   |  _ <| |_| |___) |
	//   |_| \_\\___/|____/
	// Ros nodehandle 
	ros::NodeHandle nodeHandle_;
	// Publisher used for intra process comm
	ros::Publisher image_pub_;
	ros::Publisher info_pub_;
	ros::Publisher flow_pub_;
	// Publisher used for inter process comm
	std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

	//   __     __         _       _     _
	//   \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___
	//    \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
	//     \ V / (_| | |  | | (_| | |_) | |  __/\__ \
	//      \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
	std::shared_ptr<v4l2_camera::V4l2CameraDevice> camera_;
	std::thread capture_thread_;
	std::atomic<bool> canceled_;

	std::string camera_frame_id_;
	std::string output_encoding_;

	std::map<std::string, int32_t> control_name_to_id_;

	// _____                 _   _
	// |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
	// | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
	// |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
  	// |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
	bool requestPixelFormat(std::string const &fourcc);
	bool requestImageSize(std::vector<int64_t> const &size);
	bool requestFrameRate(uint const &fps);
	bool requestOptFlowSettings();

	sensor_msgs::Image::ConstPtr convert(sensor_msgs::Image const &img) const;

	bool checkCameraInfo(
		sensor_msgs::Image const &img,
		sensor_msgs::CameraInfo const &ci);
};

#endif // V4L2_CAMERA__V4L2_CAMERA_HPP_
