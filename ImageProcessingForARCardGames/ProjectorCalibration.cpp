#include "ProjectorCalibration.h"
#include <opencv2/calib3d.hpp>


bool ProjectorCalibration::DetectChessboardCorners(cv::Mat image)
{
    if (_chessboardDimension.width <= 0 || _chessboardDimension.height <= 0)
    {
        _isError = true;
        _errorCode = NO_PARAM_FOR_CALIB_SET;
        _errorMsg = "GetChessboardCorners -> Chessboard dimension not set.";
        return;
    }
    std::vector<cv::Point2f> pointBuf;
    const bool found = cv::findChessboardCorners(image, _chessboardDimension, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

	if (found) // all corners found?
	{
		_foundedCorners = pointBuf;
		return true;
	}
    else
    {
        _isError = true;
        _errorCode = NO_ALL_CORNERS_FOUND;
        _errorMsg = "GetChessboardCorners -> Not all corners found";
    }

	return false;
}

void ProjectorCalibration::Get4CornersOfChessboard()
{
	// looking for extremes
	// 1. min(x), min(y)
	cv::Point2f bottomLeft = _foundedCorners[0];
	// 2. min(x), max(y)
	cv::Point2f topLeft = _foundedCorners[0];
	// 3. max(x), min(y)
	cv::Point2f bottomRight = _foundedCorners[0];
	// 4. max(x), max(y)
	cv::Point2f topRight = _foundedCorners[0];
	// go through all points and find the correct ones
	for (auto &point : _foundedCorners)
	{
		if (point.x <= bottomLeft.x && point.y <= bottomLeft.y)
			bottomLeft = point;
		else if (point.x <= topLeft.x && point.y >= topLeft.y)
			topLeft = point;
		else if (point.x >= bottomRight.x && point.y <= bottomRight.y)
			bottomRight = point;
		else if (point.x >= topRight.x && point.y >= topRight.y)
			topRight = point;
	}
	// if any point are same, cancel
	if (bottomLeft == topLeft || bottomLeft == bottomRight || bottomLeft == topRight ||
		topLeft == bottomRight || topLeft == topRight || bottomRight == topRight)
	{
		_isError = true;
		_errorCode = NO_DIFFERENT_CORNERS_FOUND;
		_errorMsg = "Get4CornersOfChessboard -> No different corners found.";
		return;
	}

	// calculate the desired values of points


}

ProjectorCalibration::ProjectorCalibration()
{
}


ProjectorCalibration::~ProjectorCalibration()
{
}

void ProjectorCalibration::GetProjectionMatrix(float * output, cv::Mat inputImage, TableCalibration::tableCalibrationResults* tableCalibResult)
{
	// try find corners
	bool founded = DetectChessboardCorners(inputImage);
	if (!founded)
	{
		// set error
		_errorCode = 3;
		_isError = true;
		_errorMsg = "GetProjectionMatrix -> Cannot find all corners.";
		// to return
		return;
	}

	std::cout << _foundedCorners << std::endl;
	// corners found, try to get four corresponding points
	Get4CornersOfChessboard();
	if (_isError)
	{
		return;
	}
		

	// according to table setting, calculate the size of square in chessboard

	// using 4 corresponding points and real square size, calculate perspective matrix

}
