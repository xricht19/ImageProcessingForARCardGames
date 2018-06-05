#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2\imgproc\imgproc.hpp>

/***************************************************************
* Author: Jiri Richter
* This file define the PlayerAreaActiveDetector class. The instance
* of this class is created for every player and provides the function
* to check if player is in current frame active.
****************************************************************/


// number of frames before the hand of player is again detected
#define LOWER_THRESHOLD 7
#define UPPER_THRESHOLD 15

namespace IDAP
{
	class PlayerAreaActiveDetector
	{
	public:
		PlayerAreaActiveDetector(int _playerID, int _areaX, int _areaY, int _width, int _height);
		~PlayerAreaActiveDetector();

        // return the intensity, if evaluation function was bigger then UPPER_THRESHOLD
        // and the init state was set to true, otherwise zero is returned
		double isAreaActive(cv::Mat currentFrame);

	private:
		int playerID;
		cv::Ptr<cv::BackgroundSubtractorMOG2> backSub;
		int lastFrameDetected;
		bool initState;
		// info about players area
		cv::Rect roi;
		cv::Mat fgmask;
	};

}