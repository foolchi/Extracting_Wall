#ifndef INCLUDES_H
#define INCLUDES_H

#include <vector>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <time.h>
#include <list>
#include <math.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <pcl/console/parse.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#define TOLERANCE 0.00001
#define DISTANCE_TOLERANCE 0.3
#define PERPENDICULAR_TOLERANCE 0.2 // error tolerance for perpendicular
#define MAX_CLUSTER  10 // maximum plane number for plane segmentation
#define MIN_LINE_SIZE 20 // minimun line size for line segmentation
#define PointT pcl::PointXYZ
const std::string msg_vector_file("map_vector");
#endif // INCLUDES_H
