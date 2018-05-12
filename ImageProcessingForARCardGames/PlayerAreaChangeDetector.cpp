#include "PlayerAreaChangeDetector.h"

namespace IDAP
{
    //int val = 0;

	PlayerAreaActiveDetector::PlayerAreaActiveDetector(int _playerID, int _areaX, int _areaY, int _width, int _height)
	{
		// set instance
		roi = cv::Rect(_areaX, _areaY, _width, _height);
		playerID = _playerID;

		// prepare background subtraction
		backSub = cv::createBackgroundSubtractorMOG2(5, 20.0, false);
		initState = false;
	}


	PlayerAreaActiveDetector::~PlayerAreaActiveDetector()
	{
		backSub.release();
	}

	double PlayerAreaActiveDetector::isAreaActive(cv::Mat currentFrame)
	{
		// cut area from currenFrame
		cv::Mat area = currentFrame(roi);
        const cv::Size ss(30, 30);
        cv::Mat areaSub;
        cv::resize(area, areaSub, ss);

		// remove background using openCV BackgroundSubstractorMOG2
		backSub->apply(areaSub, fgmask, 0.35);
        
        cv::Mat ero, dil;
        const cv::Mat kernel = cv::Mat::eye(5, 5, CV_8UC1);
        cv::morphologyEx(fgmask, ero, cv::MORPH_ERODE, kernel);
        cv::morphologyEx(ero, dil, cv::MORPH_DILATE, kernel);


        double value = cv::mean(dil)[0];

		// check if the value is high enough and if it already drop down from last time
		if (value > UPPER_THRESHOLD && initState) {            
            /*std::cout << "Player " << playerID << " has exited init state." << std::endl;
            std::stringstream name;
            name << val++ << ".png";
            std::string nameS = name.str();
            cv::imwrite(nameS.c_str(), dil);*/
			initState = false;
			return value;
		}
		else if (value < LOWER_THRESHOLD && !initState) {
            //std::cout << "Player " << playerID << " is back at init state." << std::endl;
			initState = true;
		}

		return -1.0;
	}
}