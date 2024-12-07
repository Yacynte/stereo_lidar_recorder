#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp> // For ORB
#include <pcl/io/pcd_io.h>
#include <thread>
#include <iostream>
#include <algorithm> 
#include <chrono>
#include "StereoCamera.h"
#include "LiDAR.h"
// #include <filesystem> // For creating folders (C++17)

void camera_record(StereoCamera stereoCam, VisualOdometry vo, 
                    std::vector<std::vector<double>> &totalTranslation, std::vector<cv::Mat> &totalRotation){
    // StereoCamera stereoCam(0, 2); // Adjust IDs based on your setup

    cv::Mat leftFrame, rightFrame, leftFrame_pre, rightFrame_pre;
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
                if (frameCounter > 0){
                    cv::Mat rotation_matrix;                // Sotre Rotation matrix 
                    std::vector<double> translation_vector;
                    std::pair<std::vector<double>, cv::Mat> motionPair;

                    motionPair = vo.StereoOdometry(leftFrame_pre, leftFrame, rightFrame_pre, rightFrame);
                    totalTranslation.push_back(motionPair.first);
                    totalRotation.push_back(motionPair.second);
                    // std::cout<<motionPair.first[2]<<std::endl;
                }
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
        if (totalTime >= 50) // Stop after 1000 frames (adjust as needed)
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
    

    // Create an ORB detector
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    // Input images
    cv::Mat left_prev = cv::imread("left_prev.jpg", cv::IMREAD_GRAYSCALE);
    if (left_prev.empty()) {
        std::cerr << "Could not open left_prev.jpg" << std::endl;
        return -1;
    }

    // Keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints_prev;
    cv::Mat descriptors_prev;

    // Detect and compute ORB features
    orb->detectAndCompute(left_prev, cv::noArray(), keypoints_prev, descriptors_prev);

    std::cout << "Number of keypoints detected: " << keypoints_prev.size() << std::endl;

    return 0;



    StereoCamera stereoCam(1, 2); // Adjust IDs based on your setup
    LiDAR lidar;
    VisualOdometry vo;

    std::vector<cv::Mat> totalRotation;
    std::vector<std::vector<double>> totalTranslation;
    // std::cout<<totalTranslation<<std::endl;
    std::thread t1(camera_record, stereoCam, vo, &totalTranslation, &totalRotation);
    // std::thread t2(lidar_record, lidar);

    t1.join();
    // t2.join();
    std::cout<< "Recording complete"<<"\n";
    return 0;
}
