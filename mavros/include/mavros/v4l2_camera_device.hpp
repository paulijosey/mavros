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

#ifndef V4L2_CAMERA__V4L2_CAMERA_DEVICE_HPP_
#define V4L2_CAMERA__V4L2_CAMERA_DEVICE_HPP_

#include <map>
#include <math.h>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/core/mat.hpp>

#include <sensor_msgs/Image.h>
#include <opt_flow_msgs/opt_flow.h>
#include "opt_flow_msgs/flows.h"             // custom opt flow message
#include "opt_flow_msgs/flow.h"              // custom opt flow message
#include "opt_flow_msgs/hamming_distances.h" // custom opt flow message
#include <cv_bridge/cv_bridge.h>

#include "control.hpp"
#include "image_format.hpp"
#include "pixel_format.hpp"
#include "of_cam_config.h"

namespace v4l2_camera
{

/** Camera device using Video4Linux2
 */
class V4l2CameraDevice
{
public:
  explicit V4l2CameraDevice(std::string device);

  bool open();
  bool start();
  bool stop();

  auto const & getControls() const {return controls_;}
  int32_t getControlValue(uint32_t id);
  bool setControlValue(uint32_t id, int32_t value);
  bool setControlValue(std::string ctrl_name, int32_t value);
  ros::Time getRosTimeShift();
  double getEpochTimeShift();

  // Types used to describe available image sizes
  enum class ImageSizeType
  {
    DISCRETE,
    STEPWISE,
    CONTINUOUS
  };
  using ImageSizesVector = std::vector<std::pair<uint16_t, uint16_t>>;
  using ImageSizesDescription = std::pair<ImageSizeType, ImageSizesVector>;

  auto const & getImageFormats() const {return image_formats_;}
  auto const & getImageSizes() const {return image_sizes_;}
  auto const & getCurrentDataFormat() const {return cur_data_format_;}
  bool requestDataFormat(PixelFormat const & format);
  bool requestFrameRateFormat(const uint &fps);

  std::string getCameraName();

  sensor_msgs::Image::ConstPtr capture();
  sensor_msgs::Image::ConstPtr capture(opt_flow_msgs::opt_flow &flow);
  double toEpochOffset_s;

private:
  /// Image buffer
  struct Buffer
  {
    unsigned index;
    unsigned char * start;
    size_t length;
  };

  std::string device_;
  int fd_;

  v4l2_capability capabilities_;
  std::vector<ImageFormat> image_formats_;
  std::map<unsigned, ImageSizesDescription> image_sizes_;
  std::vector<Control> controls_;

  PixelFormat cur_data_format_;

  std::vector<Buffer> buffers_;

  // Requests and stores all formats available for this camera
  void listImageFormats();

  // Requests and stores all frame sizes available for this camera
  void listImageSizes();

  ImageSizesDescription listDiscreteImageSizes(v4l2_frmsizeenum frm_size_enum);
  ImageSizesDescription listStepwiseImageSizes(v4l2_frmsizeenum frm_size_enum);
  ImageSizesDescription listContinuousImageSizes(v4l2_frmsizeenum frm_size_enum);

  // Requests and stores all controls available for this camera
  void listControls();

  // Set up memory mapping to buffers
  bool initMemoryMapping();
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__V4L2_CAMERA_DEVICE_HPP_
