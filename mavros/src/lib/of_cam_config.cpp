#include "of_cam_config.h"

using ImageSize = std::vector<int64_t>;

std::string DEVICE;
std::string OUTPUT_ENCODING;
std::string FRAME_ID;
std::string PIXEL_FORMAT;
int FPS;
int HEIGHT;
int WIDTH;

int OF_FAST_MIN_THRESHOLD;
int OF_FAST_MAX_THRESHOLD;
int OF_FAST_INIT_THRESHOLD;
int OF_FAST_CORNER_VALS;

int OF_MULT;

int OF_SPATIAL_FILTER_PTS_LIMIT;

int OF_BRIEF_TARGET_DESC;
int OF_BRIEF_MAX_DESC;

int OF_HAMMING_WINDOW_WIDTH;
int OF_HAMMING_WINDOW_HEIGHT;
int OF_HAMMING_MATCHING_THRESHOLD;
int OF_HAMMING_DISPARITY;

ImageSize IMAGE_SIZE;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    // read file name from param server
    config_file = readParam<std::string>(n, "of_cam_config_yaml");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

    // check if there actually is a file
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // Read Parameters from conf file
    fsSettings["cam_device"] >> DEVICE;
    fsSettings["cam_output_encoding"] >> OUTPUT_ENCODING;
    fsSettings["cam_frame_id"] >> FRAME_ID;
    fsSettings["cam_pixel_format"] >> PIXEL_FORMAT;
    fsSettings["cam_fps"] >> FPS;
    fsSettings["cam_height"] >> HEIGHT;
    fsSettings["cam_width"] >> WIDTH;
    IMAGE_SIZE = {WIDTH, HEIGHT};

    fsSettings["of_multiplicator"] >> OF_MULT;

    fsSettings["of_fast_min_threshold"] >> OF_FAST_MIN_THRESHOLD;
    fsSettings["of_fast_max_threshold"] >> OF_FAST_MAX_THRESHOLD;
    fsSettings["of_fast_initial_threshold"] >> OF_FAST_INIT_THRESHOLD;
    fsSettings["of_fast_corner_values"] >> OF_FAST_CORNER_VALS;

    fsSettings["of_spatial_filter_pts_limit"] >> OF_SPATIAL_FILTER_PTS_LIMIT;

    fsSettings["of_brief_target_descriptors"] >> OF_BRIEF_TARGET_DESC;
    fsSettings["of_brief_max_descriptors"] >> OF_BRIEF_MAX_DESC;

    fsSettings["of_hamming_window_width"] >> OF_HAMMING_WINDOW_WIDTH;
    fsSettings["of_hamming_window_height"] >> OF_HAMMING_WINDOW_HEIGHT;
    fsSettings["of_hamming_matching_threshold"] >> OF_HAMMING_MATCHING_THRESHOLD;
    fsSettings["of_hamming_disparity"] >> OF_HAMMING_DISPARITY;

    fsSettings.release();
}
