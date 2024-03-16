/* camera.hpp
 * author: nikobk
 * created: mar 15 2024
*/

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <opencv4/opencv2/opencv.hpp>

/**
 * Initialize OpenCV and instantiate a capture
 * device for the drone to use as well as any other
 * setting relevant to the image/video capture.
 */
void initCamera(const int& id, const std::string& output, const std::string& ext);

void takePic();

#endif
