// StereoCamera.cpp
#include "StereoCamera.h"

StereoCamera::StereoCamera(int leftCamID, int rightCamID)
    : leftCam(leftCamID), rightCam(rightCamID) {
    leftCam.set(cv::CAP_PROP_FPS, 20);
    rightCam.set(cv::CAP_PROP_FPS, 20);
}

bool StereoCamera::captureFrames(cv::Mat &leftFrame, cv::Mat &rightFrame) {
    return leftCam.read(leftFrame) && rightCam.read(rightFrame);
}