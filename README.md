# stereo_lidar_recorder
## Realtime Stereo Visual Odometry
This repository implements a stereo visual-LiDAR odometry algorithm designed to estimate the motion of a robot or drone using data from stereo cameras and LiDAR sensors. The project aims to integrate depth information from both visual and LiDAR sources for robust and accurate localization.

Currently, the visual odometry component is fully functional, using stereo camera data to compute relative pose (rotation and translation). The integration of LiDAR data is a planned enhancement to improve accuracy and robustness in environments with limited visual features.

Key Features:
Stereo Camera-Based Odometry:

Processes left and right camera images to estimate depth via triangulation.
Computes motion using feature-based techniques for 3D point reconstruction.
Future LiDAR Integration:

Plans to incorporate LiDAR data for enhanced depth estimation and odometry performance, especially in visually challenging environments.
Real-Time Focus: Optimized for lightweight computation, with potential for integration into drones or autonomous robots.

Applications:
Autonomous navigation for drones and mobile robots.
3D mapping and reconstruction for terrain and environment studies.
Multi-sensor fusion for robust odometry and localization.
Tools and Technologies:
Languages: Python.
Libraries: OpenCV, NumPy, Open3D (planned for LiDAR integration).
Data: Supports stereo images and calibration parameters; LiDAR data integration in progress.
How to Use:
Clone this repository.
Provide stereo camera images and calibration parameters.
Run the visual odometry algorithm to estimate the camera's trajectory.
[Future] Integrate LiDAR data for improved accuracy.
Contributions are welcome! Feel free to explore and help enhance the LiDAR integration or optimize the visual odometry component.
