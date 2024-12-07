// StereoCamera.cpp
#include "StereoCamera.h"

StereoCamera::StereoCamera(int leftCamID, int rightCamID)
    : leftCam(leftCamID), rightCam(rightCamID) {
    leftCam.set(cv::CAP_PROP_FPS, 20);
    rightCam.set(cv::CAP_PROP_FPS, 20);
}

bool StereoCamera::captureFrames(cv::Mat& leftFrame, cv::Mat& rightFrame) {
    return leftCam.read(leftFrame) && rightCam.read(rightFrame);
}

std::pair<cv::Mat, cv::Mat> VisualOdometry::RectifyImage(const cv::Mat& leftImage, const cv::Mat& rightImage) {
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, leftImage.size(), R, T, R1, R2, P1, P2, Q);

    // Compute rectification maps
    cv::Mat map1x, map1y, map2x, map2y;
    cv::initUndistortRectifyMap(K1, D1, R1, P1, leftImage.size(), CV_32FC1, map1x, map1y);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, rightImage.size(), CV_32FC1, map2x, map2y);

    // Apply the rectification maps
    cv::Mat rectifiedLeft, rectifiedRight;
    cv::remap(leftImage, rectifiedLeft, map1x, map1y, cv::INTER_LINEAR);
    cv::remap(rightImage, rectifiedRight, map2x, map2y, cv::INTER_LINEAR);

    return std::make_pair(rectifiedLeft, rectifiedRight);
}

void VisualOdometry::reconstruct3D(const std::vector<cv::Point2f>& image_points, const cv::Mat& depth,
                                   std::vector<cv::Point3f>& points_3D, std::vector<size_t>& outliers, float max_depth) {
    points_3D.clear();
    outliers.clear();

    float fx = K1.at<double>(0, 0);
    float fy = K1.at<double>(1, 1);
    float cx = K1.at<double>(0, 2);
    float cy = K1.at<double>(1, 2);

    for (size_t i = 0; i < image_points.size(); ++i) {
        float u = image_points[i].x;
        float v = image_points[i].y;
        float z = depth.at<float>(static_cast<int>(v), static_cast<int>(u));

        // Ignore points with invalid depth
        if (z > max_depth || z <= 0) {
            outliers.push_back(i);
            continue;
        }

        float x = z * (u - cx) / fx;
        float y = z * (v - cy) / fy;
        points_3D.emplace_back(x, y, z);
    }
}

cv::Point3f VisualOdometry::computeMean3D(const std::vector<cv::Point3f>& points) {
    if (points.empty()) return cv::Point3f(0, 0, 0);

    cv::Point3f mean(0, 0, 0);
    for (const auto &pt : points) {
        mean += pt;
    }
    mean *= (1.0f / points.size());
    return mean;
}

bool VisualOdometry::motionEstimation(const std::vector<cv::Point2f>& image1_points, const std::vector<cv::Point2f>& image2_points,
                                      const cv::Mat& depth, float max_depth) {
    // const std::vector<cv::Point2f> &contours, cv::Mat &contour_3d,
    if (image1_points.size() != image2_points.size()) {
        std::cerr << "Error: Point sets must have the same size." << std::endl;
        return false;
    }

    // Step 1: Reconstruct 3D points
    std::vector<cv::Point3f> points_3D;
    std::vector<size_t> outliers;
    reconstruct3D(image1_points, depth, points_3D, outliers, max_depth);

    // Remove outliers
    std::vector<cv::Point2f> filtered_image1_points, filtered_image2_points;
    for (size_t i = 0; i < image1_points.size(); ++i) {
        if (std::find(std::begin(outliers), std::end(outliers), i) == outliers.end()) {
            filtered_image1_points.push_back(image1_points[i]);
            filtered_image2_points.push_back(image2_points[i]);
            }
    }

    // Step 2: Solve PnP with RANSAC
    bool success = cv::solvePnPRansac(points_3D, filtered_image2_points, K1, cv::Mat(),
                                      rvec, translation_vector, false, 100, 8.0, 0.99, cv::noArray());

    if (!success) {
        std::cerr << "Error: solvePnPRansac failed." << std::endl;
        return false;
    }

    // Convert rotation vector to matrix
    cv::Rodrigues(rvec, rotation_matrix);

    // Step 3: Compute mean of contours in 3D if available
    // if (!contours.empty()) {
    //     std::vector<cv::Point3f> contours_3D;
    //     reconstruct3D(contours, depth, contours_3D, outliers, max_depth);
    //     cv::Point3f mean_3D = computeMean3D(contours_3D);

    //     // Convert to cv::Mat format
    //     contour_3d = (cv::Mat_<float>(1, 3) << mean_3D.x, mean_3D.y, mean_3D.z);
    // } else {
    //     contour_3d = cv::Mat::zeros(1, 3, CV_32F);
    // }

    return true;
}

cv::Mat VisualOdometry::computeDisparity(const cv::Mat& left, const cv::Mat& right) {
    // Create StereoSGBM object
    
    // cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create();
    // stereo->setBlockSize(9);                    // Block size (odd number, typically 5 to 15)
    // stereo->setNumDisparities(4*16);            // Number of disparities (must be divisible by 16)
    // stereo->setPreFilterCap(31);
    // stereo->setBlockSize(9);
    // stereo->setP1(8 * 9 * 9);
    // stereo->setP2(32 * 9 * 9);
    // stereo->setMode(cv::StereoSGBM::MODE_SGBM);

    // Create StereoSGBM object and set parameters directly in the create function
    // auto stereo = cv::StereoSGBM::create();
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
        0,            // Min disparity
        4 * 16,       // Number of disparities (must be divisible by 16)
        9,            // Block size (odd number, typically 5 to 15)
        8 * 9 * 9,    // P1: The first parameter controlling the disparity smoothness
        32 * 9 * 9,   // P2: The second parameter controlling the disparity smoothness
        31,           // Pre-filter cap
        10,           // Uniqueness ratio
        0,            // Speckle window size
        0,            // Speckle range
        cv::StereoSGBM::MODE_SGBM // Mode
    );
    // Validate object creation
    assert(stereo != nullptr && "StereoSGBM creation failed!");
    stereo->compute(left, right, disparity);

    

    // Normalize disparity for visualization (optional)
    cv::Mat disparity_normalized;
    cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

}

cv::Mat VisualOdometry::computeDepth(const cv::Mat& left, const cv::Mat& right, cv::Mat& depth){
    // const cv::Mat disparity = 
    computeDisparity(left, right);
    double focal_length_ = K1.at<double>(0, 0);
    double baseline_ = T.at<double>(0);
    // cv::Mat depth(disparity.size(), CV_32F);
    for (int y = 0; y < disparity.rows; ++y) {
        for (int x = 0; x < disparity.cols; ++x) {
            float d = static_cast<float>(disparity.at<short>(y, x)) / 16.0; // SGBM divides disparity by 16
            if (d > 0) { // Avoid division by zero
                depth.at<float>(y, x) = (focal_length_ * baseline_) / d;
            } else {
                depth.at<float>(y, x) = 0.0f; // Invalid depth
            }
        }
    }
    // return depth;
}

// Feature matching function
void VisualOdometry::feature_matching(const cv::Mat& left_prev, const cv::Mat& left_cur, 
                        std::vector<cv::Point2f>& pts_prev_L, std::vector<cv::Point2f>& pts_cur_L) {
    // Step 1: Detect ORB keypoints and compute descriptors for both images
    // cv::ORB orb; // Direct instantiation (deprecated in newer versions)
    // cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints_prev, keypoints_cur;
    cv::Mat descriptors_prev, descriptors_cur;

    // Detect keypoints and compute descriptors for the previous and current left images
    
    orb->detectAndCompute(left_prev, cv::noArray(), keypoints_prev, descriptors_prev);
    orb->detectAndCompute(left_cur, cv::noArray(), keypoints_cur, descriptors_cur);

    // Step 2: Match descriptors using Brute-Force Matcher (BFMatcher)
    cv::BFMatcher bf(cv::NORM_HAMMING, true);  // Use Hamming distance for ORB
    std::vector<cv::DMatch> matches;
    bf.match(descriptors_prev, descriptors_cur, matches);

    // Step 3: Extract the points from the matched keypoints
    pts_prev_L.clear();
    pts_cur_L.clear();
    
    for (size_t i = 0; i < matches.size(); i++) {
        // Get the keypoints from the matches
        pts_prev_L.push_back(keypoints_prev[matches[i].queryIdx].pt);
        pts_cur_L.push_back(keypoints_cur[matches[i].trainIdx].pt);
    }
}


std::pair<std::vector<double>, cv::Mat> VisualOdometry::StereoOdometry(const cv::Mat leftImage_pre, const cv::Mat leftImage_cur, 
                                    const cv::Mat rightImage_pre, const cv::Mat rightImage_cur){

    if (leftImage_pre.empty() || leftImage_cur.empty() || rightImage_pre.empty() || rightImage_cur.empty()) {
        std::cerr << "Could not open the images!" << std::endl;
        cv::Mat rot = cv::Mat(3, 3, CV_64F, cv::Scalar(-1));
        std::vector<double> trans = {-1, -1, -1};
        return std::make_pair(trans, rot);
    }
     // Step 1: Rectify images
    auto rectifiedPre = RectifyImage(leftImage_pre, rightImage_pre);
    auto rectifiedCur = RectifyImage(leftImage_cur, rightImage_cur);

    leftImageRec_pre = rectifiedPre.first;
    rightImageRec_pre = rectifiedPre.second;
    leftImageRec_cur = rectifiedCur.first;
    rightImageRec_cur = rectifiedCur.second;

    //Compute depth map of previous Image pair
    computeDepth(leftImageRec_pre, rightImageRec_pre, depth_map);

    // Vectors to hold the matched points
    std::vector<cv::Point2f> pts_prev_L, pts_cur_L;

    // Call the feature matching function
    feature_matching(leftImageRec_pre, leftImageRec_cur, pts_prev_L, pts_cur_L);

    motionEstimation(pts_prev_L, pts_cur_L, depth_map);

    // totalTranslation.push_back(translation_vector);
    // totalRotation.push_back(rotation_matrix);
    return std::make_pair(translation_vector, rotation_matrix);

}