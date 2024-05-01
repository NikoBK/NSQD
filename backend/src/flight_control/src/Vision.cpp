#include <opencv2/opencv.hpp>
#include <iostream>

void initCamera() {
	std::cout << "Starting drone camera initialization..." << std::endl;
	int id = 2;
	cv::VideoCapture cap(id);

	if (!cap.isOpened()) {
		std::cerr << "Error: Unable to open camera (" << std::to_string(id) << ")" << std::endl;
		return;
	}

	system("mkdir -p images");

	while(true) {
		// Capture a single frame (image)
		cv::Mat frame;
		cap >> frame;
		if (frame.empty()) {
			std::cerr << "Error: Unable to capture frame" << std::endl;
		}

		// cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

		std::string fileName = "images/frame.jpg";
		cv::imwrite("images/frameCap.jpg", frame);

		cv::imshow("Frame", frame);

		if (cv::waitKey(1) == 'q') {
			break;
		}
	}
	cap.release();
	cv::destroyAllWindows();
	std::cout << "Stopped camera process" << std::endl;
}

int main() {
	initCamera();
	return 1;
}
