#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2\imgproc\imgproc.hpp>


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