#include "ImageDetectionAccessPoint.h"

int main() {
	std::cout << "StartMain" << std::endl;

	IDAP::ImageDetectionAccessPoint *access = new IDAP::ImageDetectionAccessPoint();
	

	uint16_t errorCode = 0;
	uint16_t cameraId = 0;
	std::string path = "ARBang/Settings0.xml";
	access->InitImageDetectionAccessPoint(errorCode, cameraId, path.data());

	std::cout << "ALL DONE" << std::endl;

}