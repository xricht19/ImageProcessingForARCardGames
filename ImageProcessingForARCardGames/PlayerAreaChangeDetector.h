#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// number of frames before the hand of player is again detected
#define NUMBER_OF_FRAMES_DISTANCE_FROM_LAST_DETECTION 10
#define LOWER_THRESHOLD 40
#define UPPER_THRESHOLD 90

namespace IDAP
{
	class PlayerAreaChangeDetector
	{
	public:
		PlayerAreaChangeDetector();
		~PlayerAreaChangeDetector();

		bool isHandDetected(cv::Mat area);

	private:
		int areaID;
		cv::Ptr<cv::BackgroundSubtractorMOG2> backSub;
		int lastFrameDetected;
		bool initState;
	};

}