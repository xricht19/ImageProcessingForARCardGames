#include "PlayerAreaChangeDetector.h"

namespace IDAP
{

	PlayerAreaActiveDetector::PlayerAreaActiveDetector(int _playerID, int _areaX, int _areaY, int _width, int _height)
	{
		// set instance
		roi = cv::Rect(_areaX, _areaY, _width, _height);
		playerID = _playerID;

		// prepare background subtraction
		backSub = cv::createBackgroundSubtractorMOG2();
		initState = false;
	}


	PlayerAreaActiveDetector::~PlayerAreaActiveDetector()
	{
		backSub.release();
	}

	bool PlayerAreaActiveDetector::isAreaActive(cv::Mat currentFrame)
	{
		// cut area from currenFrame
		cv::Mat area = currentFrame(roi);

		// remove background using openCV BackgroundSubstractorMOG2
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