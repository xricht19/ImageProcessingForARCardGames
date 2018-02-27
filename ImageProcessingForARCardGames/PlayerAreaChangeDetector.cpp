#include "PlayerAreaChangeDetector.h"

namespace IDAP
{

	PlayerAreaChangeDetector::PlayerAreaChangeDetector()
	{
		backSub = cv::createBackgroundSubtractorMOG2();
		initState = true;
	}


	PlayerAreaChangeDetector::~PlayerAreaChangeDetector()
	{
	}

	bool PlayerAreaChangeDetector::isHandDetected(cv::Mat area)
	{
		// remove background using openCV BackgroundSubstractorMOG2
		cv::Mat fgmask;
		backSub->apply(area, fgmask);

		double value = cv::sum(fgmask)[0] / (fgmask.rows*fgmask.cols);

		// check if the value is high enough and if it already drop down from last time
		if (value > UPPER_THRESHOLD && initState) {
			initState = false;
			return true;
		}
		else if (value < LOWER_THRESHOLD) {
			initState = true;
		}

		return false;
	}
}