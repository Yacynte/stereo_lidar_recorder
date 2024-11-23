// StereoCamera.h
#pragma once
#include <opencv2/opencv.hpp>

class StereoCamera {
public:
    StereoCamera(int leftCamID, int rightCamID);
    bool captureFrames(cv::Mat &leftFrame, cv::Mat &rightFrame);
private:
    cv::VideoCapture leftCam, rightCam;
};
