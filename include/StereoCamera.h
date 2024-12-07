// StereoCamera.h
#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/features2d.hpp> // ORB and feature detection
#include <opencv2/calib3d.hpp>
#include <cassert> // Include this header

class StereoCamera {
    public:
        StereoCamera(int leftCamID, int rightCamID);
        bool captureFrames(cv::Mat& leftFrame, cv::Mat& rightFrame);
    private:
        cv::VideoCapture leftCam, rightCam;
};

class VisualOdometry {
public:
    // Method to rectify images
    std::pair<cv::Mat, cv::Mat> RectifyImage(const cv::Mat& leftImage, const cv::Mat& rightImage);

    // Method to compute stereo odometry
    std::pair<std::vector<double>, cv::Mat> StereoOdometry(const cv::Mat leftImage_pre, const cv::Mat leftImage_cur,
                       const cv::Mat rightImage_pre, const cv::Mat rightImage_cur);

    // Motion estimation from two sets of 2D points and depth map.
    bool motionEstimation(const std::vector<cv::Point2f>& image1_points, const std::vector<cv::Point2f>& image2_points,
                          const cv::Mat& depth, float max_depth = 500.0f);

    // const std::vector<cv::Point2f> &contours, cv::Mat &contour_3d,

    // void feature_matching(const cv::Mat &left_prev, const cv::Mat &left_cur, 
    //                     std::vector<cv::Point2f> &pts_prev_L, std::vector<cv::Point2f> &pts_cur_L); 
    // void feature_matching(const cv::Mat &left_prev, const cv::Mat &left_cur, 
    //                     std::vector<cv::Point2f> &pts_prev_L, std::vector<cv::Point2f> &pts_cur_L) {
private:
    // Step 2: Define camera parameters
    cv::Mat K1 = (cv::Mat_<double>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1); // Left camera intrinsic
    cv::Mat K2 = (cv::Mat_<double>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1); // Right camera intrinsic
    cv::Mat D1 = (cv::Mat_<double>(1, 5) << -0.351, 0.173, 0, 0, 0); // Left camera distortion
    cv::Mat D2 = (cv::Mat_<double>(1, 5) << -0.351, 0.173, 0, 0, 0); // Right camera distortion
    cv::Mat R = (cv::Mat_<double>(3, 3) << 0.9998, 0.0175, -0.0048, -0.0175, 0.9998, -0.0053, 0.0048, 0.0053, 1.0000); // Rotation between cameras
    cv::Mat T = (cv::Mat_<double>(3, 1) << -0.54, 0, 0); // Translation between cameras
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    cv::Mat disparity;
    cv::Mat depth_map;
    cv::Mat rvec;                           // Sotre Rotation vector
    cv::Mat rotation_matrix;                // Sotre Rotation matrix 
    std::vector<double> translation_vector;
    std::vector<cv::Mat> totalRotation;
    std::vector<std::vector<double>> totalTranslation;
    // cv::Mat depth; 
    // Rectified images
    cv::Mat leftImageRec_pre, leftImageRec_cur;
    cv::Mat rightImageRec_pre, rightImageRec_cur;
    // double baseline_;      // Baseline distance between the stereo cameras
    // double focal_length_;  // Focal length of the left camera
    // Reconstruct 3D points from 2D points using depth information.
    void reconstruct3D(const std::vector<cv::Point2f>& image_points, const cv::Mat& depth,
                       std::vector<cv::Point3f>& points_3D, std::vector<size_t>& outliers, float max_depth);

    //Compute mean of 3D points.
    cv::Point3f computeMean3D(const std::vector<cv::Point3f>& points);

    // Compute Disparity
    cv::Mat computeDisparity(const cv::Mat& left, const cv::Mat& right);

    // Compute depth map
    cv::Mat computeDepth(const cv::Mat& left, const cv::Mat& right, cv::Mat& depth); 

    void feature_matching(const cv::Mat& left_prev, const cv::Mat& left_cur, 
                        std::vector<cv::Point2f>& pts_prev_L, std::vector<cv::Point2f>& pts_cur_L);
};


