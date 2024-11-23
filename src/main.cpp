#include "StereoCamera.h"
#include "LiDAR.h"
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <boost/thread.hpp>
#include <iostream>
#include <chrono>

int main() {
    StereoCamera stereoCam(0, 1); // Adjust IDs based on your setup
    LiDAR lidar;

    cv::Mat leftFrame, rightFrame;
    auto startTime = std::chrono::steady_clock::now();

    int frameCounter = 0;

    while (true) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();

        if (elapsed >= 50) { // 1000 ms / 20 FPS = 50 ms
            // Capture stereo frames
            if (stereoCam.captureFrames(leftFrame, rightFrame)) {
                cv::imwrite("data/left_" + std::to_string(frameCounter) + ".png", leftFrame);
                cv::imwrite("data/right_" + std::to_string(frameCounter) + ".png", rightFrame);
            } else {
                std::cerr << "Failed to capture stereo frames" << std::endl;
            }

            // Capture LiDAR point cloud
            auto pointCloud = lidar.capturePointCloud();
            pcl::io::savePCDFileBinary("data/pointcloud_" + std::to_string(frameCounter) + ".pcd", *pointCloud);

            frameCounter++;
            startTime = std::chrono::steady_clock::now(); // Reset timer
        }

        if (frameCounter >= 1000) // Stop after 1000 frames (adjust as needed)
            break;
    }

    return 0;
}
