#include "ImageDetectionAccessPoint.h"

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>

#define TABLE_ID 0



int main() {
	std::cout << "StartMain" << std::endl;

	IDAP::ImageDetectionAccessPoint *access = new IDAP::ImageDetectionAccessPoint();
	

	uint16_t errorCode = 0;
	uint16_t cameraId = 0;
	std::string path = "ARBang/Settings0.xml";
	access->InitImageDetectionAccessPoint(errorCode, cameraId, path.data(), TABLE_ID);

	std::cout << "Loading Game Card Data" << std::endl;
	access->LoadCardData("ARBang/gameCardData");

	cv::namedWindow("Current");

	/*
	for (auto &item : *(access->GetGameCardData()))
	{
		cv::imshow("Current", item.second);
		cv::waitKey();
	}
	*/
	uint16_t cardID = 0;

	while (true)
	{
		char pressed = cv::waitKey(1);
		// get next frame from camera
		access->PrepareNextFrame(errorCode);
		cv::imshow("Current", access->getSubSampledFrame());
		// for all players, check if area is active
		uint16_t isActive;
		for (uint16_t i = 1; i < access->GetNumberOfPlayers() + 1; i++)
		{
			access->IsPlayerActiveByID(errorCode, i, isActive);
			if (isActive)
			{
				std::cout << "Player: " << i << " is ACTIVE!" << std::endl;
			}
		}
		// TO-DO: CHECK IF CARD HAS CHANGED
		
		// check card if c pressed
		if (pressed == 'c')
		{
			for (uint16_t i = 0; i < access->GetNumberOfCardAreas(); i++)
			{
				access->IsCardChangedByID(errorCode, i, cardID);
			}
		}
		// exit on q pressed
		if (pressed == 'q')
			break;
	}

	// free memory
	delete(access);
	std::cout << "ALL DONE" << std::endl;

}