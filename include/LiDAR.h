// LiDAR.h
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LiDAR {
public:
    LiDAR();
    pcl::PointCloud<pcl::PointXYZ>::Ptr capturePointCloud();
private:
    // Add specific LiDAR SDK initialization and data capture methods.
};
