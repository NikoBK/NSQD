/* camera.cpp
 * author: nikobk
 * created: mar 15 2024
*/

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include "camera.hpp"

int main() {
		initCam();
		return 0;
}

void initCam() {
		log("Initalizing camera...");
		takePic();
}

void takePic() {
		log("Taking picture...");
		cv::VideoCapture cap(2);

		// Check if camera opened succesfully.
		if (!cap.isOpened()) {
				std::cerr << "Error: Unable to open camera" << std::endl;
		}

		// Capture a single frame
		cv::Mat frame;
		cap >> frame;

		if (frame.empty()) {
				std::cerr << "Error: Unable to capture frame" << std::endl;
		}

		cv::imwrite("images/capured_img.jpg", frame);
		cap.release();

		log("Image captured succesfully");
		log("Image saved in images/");
}

void log(const std::string& text) {
		std::cout << text << std::endl;
}
