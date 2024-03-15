/* camera.cpp
 * author: nikobk
 * created: mar 15 2024
*/

#include <iostream>
#include "camera.hpp"

int main() {
		initCam();
		return 0;
}

void initCam() {
		log("Initalizing camera...");	
}

void log(const std::string& text) {
		std::cout << text << std::endl;
}
