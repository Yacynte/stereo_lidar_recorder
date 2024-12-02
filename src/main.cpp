#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <thread>
#include <iostream>
#include <chrono>
#include "StereoCamera.h"
#include "LiDAR.h"
// #include <filesystem> // For creating folders (C++17)

void camera_record(StereoCamera stereoCam){
    // StereoCamera stereoCam(0, 2); // Adjust IDs based on your setup

    cv::Mat leftFrame, rightFrame;
    auto startTime = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();
    int frameCounter = 0;

    while (true) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();

        if (elapsed >= 50) { // 1000 ms / 20 FPS = 50 ms
            // Capture stereo frames
            if (stereoCam.captureFrames(leftFrame, rightFrame)) {
                cv::imwrite("data/left/left_" + std::to_string(frameCounter) + ".png", leftFrame);
                cv::imwrite("data/right/right_" + std::to_string(frameCounter) + ".png", rightFrame);
            } else {
                std::cerr << "Failed to capture stereo frames" << std::endl;
            }

            frameCounter++;
            startTime = std::chrono::steady_clock::now(); // Reset timer
        }
        auto curTime = std::chrono::steady_clock::now();
        auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - start).count();
        if (totalTime >= 6000) // Stop after 1000 frames (adjust as needed)
            break;
    }

}

void lidar_record(LiDAR lidar){

    cv::Mat leftFrame, rightFrame;
    auto startTime = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();
    int frameCounter = 0;

    while (true) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();

        if (elapsed >= 50) { // 1000 ms / 20 FPS = 50 ms
            // Capture LiDAR point cloud
            auto pointCloud = lidar.capturePointCloud();
            pcl::io::savePCDFileBinary("data/lidar/pointcloud_" + std::to_string(frameCounter) + ".pcd", *pointCloud);

            frameCounter++;
            startTime = std::chrono::steady_clock::now(); // Reset timer
        }
        auto curTime = std::chrono::steady_clock::now();
        auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - start).count();
        if (totalTime >= 6000) // Stop after 1000 frames (adjust as needed)
            break;
    }

}

int main() {

    // cv::VideoCapture cap(0); // Try index 0 first
    // if (!cap.isOpened()) {
    //     std::cerr << "Error: Cannot open camera!" << std::endl;
    //     return -1;
    // }

    // cv::Mat frame;
    // while (true) {
    //     cap >> frame;
    //     if (frame.empty()) break;
    //     cv::imshow("Camera Test", frame);
    //     if (cv::waitKey(10) == 27) break; // Exit on 'ESC'
    // }
    // return 0;

    StereoCamera stereoCam(1, 2); // Adjust IDs based on your setup
    LiDAR lidar;

    std::thread t1(camera_record, stereoCam);
    // std::thread t2(lidar_record, lidar);

    t1.join();
    // t2.join();
    std::cout<< "Recording complete"<<"\n";
    return 0;
}
