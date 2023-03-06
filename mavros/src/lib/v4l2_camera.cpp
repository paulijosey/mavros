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

#include "v4l2_camera.hpp"

using namespace std::chrono_literals;

V4L2Camera::V4L2Camera()
{

}

V4L2Camera::V4L2Camera(ros::NodeHandle nh)
{
  nodeHandle_ = nh;

  readParameters(nh);

  // Prepare publisher
  image_pub_ = nodeHandle_.advertise<sensor_msgs::Image>("image_raw", 10);
  info_pub_ = nodeHandle_.advertise<sensor_msgs::CameraInfo>("camera_info", 10);
  flow_pub_ = nodeHandle_.advertise<opt_flow_msgs::opt_flow>("opt_flow", 10);

  // Prepare camera
  camera_ = std::make_shared<v4l2_camera::V4l2CameraDevice>(DEVICE);

  if (!camera_->open())
  {
    return;
  }

  // set image format
  if (!requestImageSize(IMAGE_SIZE))
  {
    return;
  }

  if (!requestPixelFormat(PIXEL_FORMAT))
  {
    return;
  }

  if (!requestFrameRate(FPS))
  {
    return;
  }

  if(!requestOptFlowSettings())
  {
    return;
  }

  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(nh, camera_->getCameraName());

  // Start the camera
  if (!camera_->start())
  {
    return;
  }
}

V4L2Camera::~V4L2Camera()
{
  canceled_.store(true);
  if (capture_thread_.joinable())
  {
    capture_thread_.join();
  }
}

void V4L2Camera::init(ros::NodeHandle nh)
{
  nodeHandle_ = nh;

  readParameters(nh);

  // Prepare publisher
  image_pub_ = nodeHandle_.advertise<sensor_msgs::Image>("image_raw", 10);
  info_pub_ = nodeHandle_.advertise<sensor_msgs::CameraInfo>("camera_info", 10);
  flow_pub_ = nodeHandle_.advertise<opt_flow_msgs::opt_flow>("opt_flow", 10);

  // Prepare camera
  camera_ = std::make_shared<v4l2_camera::V4l2CameraDevice>(DEVICE);

  if (!camera_->open())
  {
    return;
  }

  // set image format
  if (!requestImageSize(IMAGE_SIZE))
  {
    return;
  }

  if (!requestPixelFormat(PIXEL_FORMAT))
  {
    return;
  }

  if (!requestFrameRate(FPS))
  {
    ROS_WARN("FPS could not be set");
  }

  if(!requestOptFlowSettings())
  {
    return;
  }

  if(!camera_->setControlValue("CAM - set stream to fsync", 1))
  {
    return;
  }

  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(nh, camera_->getCameraName());

  // Start the camera
  if (!camera_->start())
  {
    return;
  }
}

void V4L2Camera::capture()
{
  opt_flow_msgs::opt_flow flow;
  // caputre image and optical flow data
  auto img = camera_->capture(flow);
  ROS_DEBUG_STREAM("CAM Time: " << img->header.stamp);
  if (img == nullptr)
  {
    // Failed capturing image, assume it is temporarily and continue a bit later
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    capture();
  }
  else
  {
    auto stamp = ros::Time::now();
    if (img->encoding != OUTPUT_ENCODING)
    {
      ROS_WARN_STREAM(
          "Image encoding not the same as requested output, performing possibly slow conversion: "
          << img->encoding.c_str() << " " << OUTPUT_ENCODING.c_str());
      img = convert(*img);
    }

    // get current CameraInfo data
    sensor_msgs::CameraInfoPtr
        ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

    if (!checkCameraInfo(*img, *ci))
    {
      ci.reset(new sensor_msgs::CameraInfo());
      ci->height = img->height;
      ci->width = img->width;
    }

    ci->header.stamp = stamp;

    ROS_DEBUG_STREAM("Image message address [PUBLISH]:\t" << img.get());
    image_pub_.publish(std::move(img));
    info_pub_.publish(std::move(ci));
    flow_pub_.publish(std::move(flow));
    // delete all data in flow after publish
    flow.curr_features.data.clear();
    flow.prev_features.data.clear();
    flow.flows.data.clear();
    flow.hamming_distances.first_score.clear();
    flow.hamming_distances.second_score.clear();
  }
}

bool V4L2Camera::requestPixelFormat(std::string const &fourcc)
{
  if (fourcc.size() != 4)
  {
    return false;
  }

  auto code = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given pixel format
  if (dataFormat.pixelFormat == code)
  {
    return true;
  }

  dataFormat.pixelFormat = code;
  return camera_->requestDataFormat(dataFormat);
}

bool V4L2Camera::requestImageSize(std::vector<int64_t> const &size)
{
  if (size.size() != 2)
  {
    ROS_WARN_STREAM(
        "Invalid image size; expected dimensions: 2, actual: " << size.size());
    return false;
  }

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given size
  if (dataFormat.width == size[0] && dataFormat.height == size[1])
  {
    return true;
  }

  dataFormat.width = size[0];
  dataFormat.height = size[1];
  return camera_->requestDataFormat(dataFormat);
}

bool V4L2Camera::requestFrameRate(uint const &fps)
{
  return camera_->requestFrameRateFormat(fps);
}

bool V4L2Camera::requestOptFlowSettings()
{
  return  
      (camera_->setControlValue("OF/Fast - Min threshold", OF_FAST_MIN_THRESHOLD)) &&
      (camera_->setControlValue("OF/Fast - Max threshold", OF_FAST_MAX_THRESHOLD)) &&
      (camera_->setControlValue("OF/Fast - Initial threshold", OF_FAST_INIT_THRESHOLD)) &&
      (camera_->setControlValue("OF/Fast - Corner values", OF_FAST_CORNER_VALS)) &&
      (camera_->setControlValue("OF/Spatial Filter - Pts limit", OF_SPATIAL_FILTER_PTS_LIMIT)) &&
      (camera_->setControlValue("OF/Brief - Max descriptors", OF_BRIEF_MAX_DESC)) &&
      (camera_->setControlValue("OF/Brief - Target descriptors", OF_BRIEF_TARGET_DESC)) &&
      (camera_->setControlValue("OF/Hamming - Window Width", OF_HAMMING_WINDOW_WIDTH)) &&
      (camera_->setControlValue("OF/Hamming - Window Height", OF_HAMMING_WINDOW_HEIGHT)) &&
      (camera_->setControlValue("OF/Hamming - Matching Threshold", OF_HAMMING_MATCHING_THRESHOLD)) &&
      (camera_->setControlValue("OF/Hamming - Disparity (FP8.8)", OF_HAMMING_DISPARITY));
}

sensor_msgs::Image::ConstPtr V4L2Camera::convert(sensor_msgs::Image const &img) const
{
  auto cvImg = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  auto outImg = std::make_unique<sensor_msgs::Image>();
  auto cvConvertedImg = cv_bridge::cvtColor(cvImg, output_encoding_);
  cvConvertedImg->toImageMsg(*outImg);
  return outImg;
}

bool V4L2Camera::checkCameraInfo(
    sensor_msgs::Image const &img,
    sensor_msgs::CameraInfo const &ci)
{
  return ci.width == img.width && ci.height == img.height;
}
