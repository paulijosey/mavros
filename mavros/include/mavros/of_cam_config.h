#pragma once
#include<string>
#include<vector>

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

using ImageSize = std::vector<int64_t>;

extern std::string DEVICE;
extern std::string OUTPUT_ENCODING;
extern std::string FRAME_ID;
extern std::string PIXEL_FORMAT;
extern ImageSize IMAGE_SIZE;
extern int FPS;

extern int OF_MULT;

extern int OF_FAST_MIN_THRESHOLD;
extern int OF_FAST_MAX_THRESHOLD;
extern int OF_FAST_INIT_THRESHOLD;
extern int OF_FAST_CORNER_VALS;

extern int OF_SPATIAL_FILTER_PTS_LIMIT;

extern int OF_BRIEF_TARGET_DESC;
extern int OF_BRIEF_MAX_DESC;

extern int OF_HAMMING_WINDOW_WIDTH;
extern int OF_HAMMING_WINDOW_HEIGHT;
extern int OF_HAMMING_MATCHING_THRESHOLD;
extern int OF_HAMMING_DISPARITY;

void readParameters(ros::NodeHandle &n);
