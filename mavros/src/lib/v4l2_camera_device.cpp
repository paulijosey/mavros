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

#include "v4l2_camera_device.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>

#include "fourcc.hpp"

using sensor_msgs::Image;
using v4l2_camera::V4l2CameraDevice;

V4l2CameraDevice::V4l2CameraDevice(std::string device)
    : device_{std::move(device)}
{
}

bool V4l2CameraDevice::open()
{
  fd_ = ::open(device_.c_str(), O_RDWR);

  if (fd_ < 0)
  {
    auto msg = std::ostringstream{};
    msg << "Failed opening device " << device_ << ": " << strerror(errno) << " (" << errno << ")";
    ROS_ERROR(msg.str().c_str());
    return false;
  }

  // List capabilities
  ioctl(fd_, VIDIOC_QUERYCAP, &capabilities_);

  auto canRead = capabilities_.capabilities & V4L2_CAP_READWRITE;
  auto canStream = capabilities_.capabilities & V4L2_CAP_STREAMING;

  ROS_INFO(
      "Driver: %s", capabilities_.driver);
  ROS_INFO(
      "Version: %s", std::to_string(capabilities_.version).c_str());
  ROS_INFO(
      "Device: %s", capabilities_.card);
  ROS_INFO(
      "Location: %s", capabilities_.bus_info);

  ROS_INFO(
      "Capabilities:");
  ROS_INFO(
      "  Read/write: %s", (canRead ? "YES" : "NO"));
  ROS_INFO(
      "  Streaming: %s", (canStream ? "YES" : "NO"));

  // Get current data (pixel) format
  auto formatReq = v4l2_format{};
  formatReq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(fd_, VIDIOC_G_FMT, &formatReq);
  cur_data_format_ = PixelFormat{formatReq.fmt.pix};

  ROS_INFO(
      "Current pixel format: %s @ %sx%s", FourCC::toString(cur_data_format_.pixelFormat).c_str(),
      std::to_string(cur_data_format_.width).c_str(),
      std::to_string(cur_data_format_.height).c_str());

  // List all available image formats and controls
  listImageFormats();
  listImageSizes();
  listControls();

  ROS_INFO("Available pixel formats: ");
  for (auto const &format : image_formats_)
  {
    ROS_INFO(
        "  %s - %s", FourCC::toString(format.pixelFormat).c_str(), format.description.c_str());
  }

  ROS_INFO("Available controls: ");
  for (auto const &control : controls_)
  {
    ROS_INFO(
        "  %s (%s) = %s", control.name.c_str(),
        std::to_string(static_cast<unsigned>(control.type)).c_str(),
        std::to_string(getControlValue(control.id)).c_str());
  }

  return true;
}

bool V4l2CameraDevice::start()
{
  ROS_INFO("Starting camera");
  if (!initMemoryMapping())
  {
    return false;
  }

  // Queue the buffers
  for (auto const &buffer : buffers_)
  {
    auto buf = v4l2_buffer{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = buffer.index;

    if (-1 == ioctl(fd_, VIDIOC_QBUF, &buf))
    {
      ROS_ERROR(
          "Buffer failure on capture start: %s (%s)", strerror(errno),
          std::to_string(errno).c_str());
      return false;
    }
  }
  toEpochOffset_s = getEpochTimeShift();
  // Start stream
  unsigned type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl(fd_, VIDIOC_STREAMON, &type))
  {
    ROS_ERROR(
        "Failed stream start: %s (%s)", strerror(errno),
        std::to_string(errno).c_str());
    return false;
  }
  ROS_INFO("Camera started");
  return true;
}

bool V4l2CameraDevice::stop()
{
  ROS_INFO("Stopping camera");
  // Stop stream
  unsigned type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl(fd_, VIDIOC_STREAMOFF, &type))
  {
    ROS_ERROR(
        "Failed stream stop");
    return false;
  }

  // De-initialize buffers
  for (auto const &buffer : buffers_)
  {
    munmap(buffer.start, buffer.length);
  }

  buffers_.clear();

  auto req = v4l2_requestbuffers{};

  // Free all buffers
  req.count = 0;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  ioctl(fd_, VIDIOC_REQBUFS, &req);

  return true;
}

std::string V4l2CameraDevice::getCameraName()
{
  auto name = std::string{reinterpret_cast<char *>(capabilities_.card)};
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);
  std::replace(name.begin(), name.end(), ' ', '_');
  return name;
}

Image::ConstPtr V4l2CameraDevice::capture()
{
  auto buf = v4l2_buffer{};

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  // ros::Time stamp = ros::Time::now();
  // trigger a capture 
  // Start stream
  setControlValue("CAM - capture", 1);
  // Dequeue buffer with new image
  if (-1 == ioctl(fd_, VIDIOC_DQBUF, &buf))
  {
    ROS_ERROR(
        "Error dequeueing buffer: %s (%s)", strerror(errno),
        std::to_string(errno).c_str());
    return nullptr;
  }
  double temp_s = buf.timestamp.tv_sec + buf.timestamp.tv_usec*1e-6;
  double epochTimeStamp_s = (temp_s + toEpochOffset_s);
  double header_s, header_ns;
  header_ns = modf(epochTimeStamp_s, &header_s);
  header_ns = header_ns*1e9;

  // Create image object
  auto img = std::make_unique<Image>();
  img->width = cur_data_format_.width;
  img->height = cur_data_format_.height;
  img->step = cur_data_format_.bytesPerLine;
  // img->header.stamp = stamp;
  img->header.stamp.sec = header_s;
  img->header.stamp.nsec = header_ns;

  // Requeue buffer to be reused for new captures
  if (-1 == ioctl(fd_, VIDIOC_QBUF, &buf))
  {
    ROS_ERROR(
        "Error re-queueing buffer: %s (%s)", strerror(errno),
        std::to_string(errno).c_str());
    return nullptr;
  }

  if (cur_data_format_.pixelFormat == V4L2_PIX_FMT_YUYV)
  {
    img->encoding = sensor_msgs::image_encodings::YUV422;
  }
  else if (cur_data_format_.pixelFormat == V4L2_PIX_FMT_GREY)
  {
    img->encoding = sensor_msgs::image_encodings::MONO8;
  }
  else if (cur_data_format_.pixelFormat == V4L2_PIX_FMT_SGBRG8)
  {
    img->encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
  }
  else
  {
    ROS_WARN(
        "Current pixel format is not supported yet: %s %d",
        FourCC::toString(cur_data_format_.pixelFormat).c_str(),
        cur_data_format_.pixelFormat);
  }
  img->data.resize(cur_data_format_.imageByteSize);

  auto const &buffer = buffers_[buf.index];
  std::copy(buffer.start, buffer.start + img->data.size(), img->data.begin());

  // set header info
  // img->header.stamp = ros::Time::now();
  img->header.frame_id = FRAME_ID;
  return img;
}

// overload to also get flow data
// this is specific to the vd56g3 sensor from ST.
// It uses the last 64 lines of the image for optical flow data
// and the first 128 rows
Image::ConstPtr V4l2CameraDevice::capture(opt_flow_msgs::opt_flow &flow)
{
  Image::ConstPtr img;
  // get unaltered image data from sensor
  auto img_raw = capture();
  // transform to opencv image for easy modification.
  // Uses mono8 encoding because who needs RGB outside of gaming rigs?
  auto cv_img_raw = cv_bridge::toCvCopy(img_raw, OUTPUT_ENCODING);
  // split the raw img into img and flow matrices
  // TODO: check if ranges are correct
  int flow_cols = 256;
  int flow_rows = 64;
  int num_features = 0;
  computer_vision_msgs::feature prev_point, cur_point;
  opt_flow_msgs::flow flow_data;
  opt_flow_msgs::hamming_distances hamming_distances;
  // TODO: not sure if those ranges are super correct ...
  cv::Mat flow_mat = cv_img_raw->image.colRange(0, flow_cols).rowRange(cur_data_format_.height - flow_rows, cur_data_format_.height);

  // clear everything in case there is data left
  flow.prev_features.data.clear();
  flow.curr_features.data.clear();
  flow.flows.data.clear();
  flow.hamming_distances.first_score.clear();
  flow.hamming_distances.second_score.clear();
  flow.prev_features.numFeatures = 0;
  flow.curr_features.numFeatures = 0;

  for (int i = 0; i < flow_cols; i = i + 8)
  {
    for (int j = 0; j < flow_rows; j++)
    {
      // check if coordinates are zero! (should be 10 or greater if they contain data)
      prev_point.x = OF_MULT * (flow_mat.at<uint8_t>(j, i + 0) + flow_mat.at<uint8_t>(j, i + 1) * 256);
      prev_point.y = OF_MULT * (flow_mat.at<uint8_t>(j, i + 2) + flow_mat.at<uint8_t>(j, i + 3) * 256);

      if (!(prev_point.x == 0 || prev_point.y == 0 || flow_mat.at<uint8_t>(j, i + 7) == 0 || 
          prev_point.x > cur_data_format_.height))
      {
        num_features++;
        // at(row,col)
        flow_data.x = OF_MULT * flow_mat.at<int8_t>(j, i + 4);
        flow_data.y = OF_MULT * flow_mat.at<int8_t>(j, i + 5);
        cur_point.x = prev_point.x + flow_data.x;
        cur_point.y = prev_point.y + flow_data.y;
        // push to flow message
        flow.prev_features.data.push_back(prev_point);
        flow.curr_features.data.push_back(cur_point);
        flow.flows.data.push_back(flow_data);
        flow.hamming_distances.first_score.push_back(flow_mat.at<uint8_t>(j, i + 6));
        flow.hamming_distances.second_score.push_back(flow_mat.at<uint8_t>(j, i + 7));
      }
    }
    flow.prev_features.numFeatures = num_features;
    flow.curr_features.numFeatures = num_features;
  }
  // update header info
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = FRAME_ID;
  cv_img_raw->header = img_raw->header;
  flow.header = img_raw->header;
  flow.height = cur_data_format_.height - flow_rows;
  flow.width = cur_data_format_.width;

  // make image message again with "new" size
  cv_img_raw->image = cv_img_raw->image.colRange(0, cur_data_format_.width)
                          .rowRange(0, cur_data_format_.height - flow_rows);
  img = cv_img_raw->toImageMsg();
  return img;
}

int32_t V4l2CameraDevice::getControlValue(uint32_t id)
{
  auto ctrl = v4l2_control{};
  ctrl.id = id;
  if (-1 == ioctl(fd_, VIDIOC_G_CTRL, &ctrl))
  {
    ROS_ERROR(
        "Failed getting value for control %s: %s (%s); returning 0!", std::to_string(id).c_str(),
        strerror(errno), std::to_string(errno).c_str());
    return 0;
  }
  return ctrl.value;
}

bool V4l2CameraDevice::setControlValue(uint32_t id, int32_t value)
{
  auto ctrl = v4l2_control{};
  ctrl.id = id;
  ctrl.value = value;
  if (-1 == ioctl(fd_, VIDIOC_S_CTRL, &ctrl))
  {
    auto control = std::find_if(
        controls_.begin(), controls_.end(),
        [id](Control const &c)
        { return c.id == id; });
    ROS_ERROR(
        "Failed setting value for control %s to %s: %s (%s)", control->name.c_str(),
        std::to_string(value).c_str(), strerror(errno), std::to_string(errno).c_str());
    return false;
  }
  return true;
}

bool V4l2CameraDevice::setControlValue(std::string ctrl_name, int32_t value)
{
  auto ctrl = v4l2_control{};
  auto control = std::find_if(
      controls_.begin(), controls_.end(),
      [ctrl_name](Control const &c)
      { return c.name == ctrl_name; });
  ctrl.id = control->id;
  ctrl.value = value;
  if (-1 == ioctl(fd_, VIDIOC_S_CTRL, &ctrl))
  {
    ROS_ERROR_STREAM(
        "Failed setting value for control: " << ctrl_name);
    return false;
  }
  return true;
}

bool V4l2CameraDevice::requestDataFormat(const PixelFormat &format)
{
  auto formatReq = v4l2_format{};
  formatReq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  formatReq.fmt.pix.pixelformat = format.pixelFormat;
  formatReq.fmt.pix.width = format.width;
  formatReq.fmt.pix.height = format.height;

  ROS_INFO(
      "Requesting format: %sx%s %s",
      std::to_string(format.width).c_str(),
      std::to_string(format.height).c_str(),
      FourCC::toString(format.pixelFormat).c_str());

  // Perform request
  if (-1 == ioctl(fd_, VIDIOC_S_FMT, &formatReq))
  {
    ROS_ERROR(
        "Failed requesting pixel format: %s (%s)", strerror(errno),
        std::to_string(errno).c_str());
    return false;
  }

  ROS_INFO("Success");
  cur_data_format_ = PixelFormat{formatReq.fmt.pix};
  return true;
}

ros::Time V4l2CameraDevice::getRosTimeShift()
{
  return ros::Time::now();
}

double V4l2CameraDevice::getEpochTimeShift()
{
  struct timeval epochtime;
  struct timespec vsTime;

  gettimeofday(&epochtime, NULL);
  clock_gettime(CLOCK_MONOTONIC, &vsTime);

  double uptime_s = vsTime.tv_sec + vsTime.tv_nsec*1e-9;
  double epoch_s = epochtime.tv_sec + epochtime.tv_usec*1e-6;
  return epoch_s - uptime_s;
}

bool V4l2CameraDevice::requestFrameRateFormat(const uint &fps)
{
  ROS_INFO_STREAM("Requesting FPS: " << fps);

  struct v4l2_streamparm streamparm;
  memset(&streamparm, 0, sizeof(streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_G_PARM, &streamparm) != 0)
  {
    ROS_WARN_STREAM("Failed to set frame rate");
    return false;
  }

  streamparm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
  streamparm.parm.capture.timeperframe.numerator = 1;
  streamparm.parm.capture.timeperframe.denominator = fps;
  if (ioctl(fd_, VIDIOC_S_PARM, &streamparm) != 0)
  {
    ROS_WARN_STREAM("Failed to set frame rate");
    return false;
  }

  ROS_INFO("Success");
  return true;
}

void V4l2CameraDevice::listImageFormats()
{
  image_formats_.clear();

  struct v4l2_fmtdesc fmtDesc;
  fmtDesc.index = 0;
  fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while (ioctl(fd_, VIDIOC_ENUM_FMT, &fmtDesc) == 0)
  {
    image_formats_.emplace_back(fmtDesc);
    fmtDesc.index++;
  }
}

void V4l2CameraDevice::listImageSizes()
{
  image_sizes_.clear();
  struct v4l2_frmsizeenum frmSizeEnum;
  // Supported sizes can be different per format
  for (auto const &f : image_formats_)
  {
    frmSizeEnum.index = 0;
    frmSizeEnum.pixel_format = f.pixelFormat;

    if (-1 == ioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frmSizeEnum))
    {
      ROS_ERROR_STREAM(
          "Failed listing frame size " << strerror(errno) << " (" << errno << ")");
      continue;
    }

    switch (frmSizeEnum.type)
    {
    case V4L2_FRMSIZE_TYPE_DISCRETE:
      image_sizes_[f.pixelFormat] = listDiscreteImageSizes(frmSizeEnum);
      break;
    case V4L2_FRMSIZE_TYPE_STEPWISE:
      image_sizes_[f.pixelFormat] = listStepwiseImageSizes(frmSizeEnum);
      break;
    case V4L2_FRMSIZE_TYPE_CONTINUOUS:
      image_sizes_[f.pixelFormat] = listContinuousImageSizes(frmSizeEnum);
      break;
    default:
      ROS_WARN_STREAM(
          "Frame size type not supported: " << frmSizeEnum.type);
      continue;
    }
  }
}

V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listDiscreteImageSizes(
    v4l2_frmsizeenum frm_size_enum)
{
  auto sizes = ImageSizesVector{};

  do
  {
    sizes.emplace_back(std::make_pair(frm_size_enum.discrete.width, frm_size_enum.discrete.height));
    frm_size_enum.index++;
  } while (ioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frm_size_enum) == 0);

  return std::make_pair(ImageSizeType::DISCRETE, std::move(sizes));
}

V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listStepwiseImageSizes(
    v4l2_frmsizeenum frm_size_enum)
{
  // Three entries: min size, max size and stepsize
  auto sizes = ImageSizesVector(3);
  sizes[0] = std::make_pair(frm_size_enum.stepwise.min_width, frm_size_enum.stepwise.min_height);
  sizes[1] = std::make_pair(frm_size_enum.stepwise.max_width, frm_size_enum.stepwise.max_height);
  sizes[2] = std::make_pair(frm_size_enum.stepwise.step_width, frm_size_enum.stepwise.step_height);

  return std::make_pair(ImageSizeType::STEPWISE, std::move(sizes));
}

V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listContinuousImageSizes(
    v4l2_frmsizeenum frm_size_enum)
{
  // Two entries: min size and max size, stepsize is implicitly 1
  auto sizes = ImageSizesVector(2);
  sizes[0] = std::make_pair(frm_size_enum.stepwise.min_width, frm_size_enum.stepwise.min_height);
  sizes[1] = std::make_pair(frm_size_enum.stepwise.max_width, frm_size_enum.stepwise.max_height);

  return std::make_pair(ImageSizeType::CONTINUOUS, std::move(sizes));
}

void V4l2CameraDevice::listControls()
{
  controls_.clear();

  auto queryctrl = v4l2_queryctrl{};
  queryctrl.id = V4L2_CID_USER_CLASS | V4L2_CTRL_FLAG_NEXT_CTRL;

  while (ioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl) == 0)
  {
    // Ignore disabled controls
    if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    {
      continue;
    }

    auto menuItems = std::map<int, std::string>{};
    if (queryctrl.type == (unsigned)ControlType::MENU)
    {
      auto querymenu = v4l2_querymenu{};
      querymenu.id = queryctrl.id;

      // Query all enum values
      for (auto i = queryctrl.minimum; i <= queryctrl.maximum; i++)
      {
        querymenu.index = i;
        if (ioctl(fd_, VIDIOC_QUERYMENU, &querymenu) == 0)
        {
          menuItems[i] = (const char *)querymenu.name;
        }
      }
    }

    auto control = Control{};
    control.id = queryctrl.id;
    control.name = std::string{reinterpret_cast<char *>(queryctrl.name)};
    control.type = static_cast<ControlType>(queryctrl.type);
    control.minimum = queryctrl.minimum;
    control.maximum = queryctrl.maximum;
    control.defaultValue = queryctrl.default_value;
    control.menuItems = std::move(menuItems);

    controls_.push_back(control);

    // Get ready to query next item
    queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }
}

bool V4l2CameraDevice::initMemoryMapping()
{
  auto req = v4l2_requestbuffers{};

  // Request 4 buffers
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  ioctl(fd_, VIDIOC_REQBUFS, &req);

  // Didn't get more than 1 buffer
  if (req.count < 2)
  {
    ROS_ERROR("Insufficient buffer memory");
    return false;
  }

  buffers_ = std::vector<Buffer>(req.count);

  for (auto i = 0u; i < req.count; ++i)
  {
    auto buf = v4l2_buffer{};

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    ioctl(fd_, VIDIOC_QUERYBUF, &buf);

    buffers_[i].index = buf.index;
    buffers_[i].length = buf.length;
    buffers_[i].start =
        static_cast<unsigned char *>(
            mmap(
                NULL /* start anywhere */,
                buf.length,
                PROT_READ | PROT_WRITE /* required */,
                MAP_SHARED /* recommended */,
                fd_, buf.m.offset));

    if (MAP_FAILED == buffers_[i].start)
    {
      ROS_ERROR("Failed mapping device memory");
      return false;
    }
  }

  return true;
}
