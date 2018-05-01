#include "ProjectorCalibration.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/videostab/ring_buffer.hpp>
#include <opencv2/highgui/highgui.hpp>


bool ProjectorCalibration::DetectChessboardCorners(cv::Mat image)
{
    if (_chessboardDimension.width <= 0 || _chessboardDimension.height <= 0)
    {
        _isError = true;
        _errorCode = NO_PARAM_FOR_CALIB_SET;
        _errorMsg = "GetChessboardCorners -> Chessboard dimension not set.";
        return false;
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
	// looking for extremes - corners sorted row by row, left to right
	// 1. min(x), min(y)
	cv::Point2f bottomLeft = _foundedCorners[_chessboardDimension.width*(_chessboardDimension.height-1)];
	// 2. min(x), max(y)
	cv::Point2f topLeft = _foundedCorners[0];
	// 3. max(x), min(y)
	cv::Point2f bottomRight = _foundedCorners[_chessboardDimension.width*_chessboardDimension.height-1];
	// 4. max(x), max(y)
	cv::Point2f topRight = _foundedCorners[_chessboardDimension.width-1];
	// if any point are same, cancel
	if (bottomLeft == topLeft || bottomLeft == bottomRight || bottomLeft == topRight ||
		topLeft == bottomRight || topLeft == topRight || bottomRight == topRight)
	{
		_isError = true;
		_errorCode = NO_DIFFERENT_CORNERS_FOUND;
		_errorMsg = "Get4CornersOfChessboard -> No different corners found.";
		return;
	}

    // offset settings
    const int offsetX = topLeft.x;
    const int offsetY = topLeft.y;

	// calculate the desired values of points
    const float maxWidth = static_cast<float>(MAX(cv::norm(bottomRight - bottomLeft),
        cv::norm(topRight - topLeft)));
    const float maxHeight = static_cast<float>(MAX(cv::norm(topRight - bottomRight),
        cv::norm(topLeft - bottomLeft)));
    _projectionPointsOrigin.emplace_back(topLeft);
    _projectionPointsTarget.emplace_back(cv::Point2f(offsetX, offsetY));
    _projectionPointsOrigin.emplace_back(bottomLeft);
    _projectionPointsTarget.emplace_back(cv::Point2f(offsetX, offsetY+maxHeight));
    _projectionPointsOrigin.emplace_back(bottomRight); 
    _projectionPointsTarget.emplace_back(cv::Point2f(offsetX+maxWidth, offsetY+maxHeight));
    _projectionPointsOrigin.emplace_back(topRight);
    _projectionPointsTarget.emplace_back(cv::Point2f(offsetX+maxWidth, offsetY));

}

ProjectorCalibration::ProjectorCalibration(): _squareDimension(0.f), _isError(false), _errorCode(0)
{
}

bool ProjectorCalibration::GetProjectionMatrix(double* output, double& sizeOut, cv::Mat inputImage, TableCalibration::tableCalibrationResults* tableCalibResult)
{
	// try find corners
	const bool founded = DetectChessboardCorners(inputImage);
	if (!founded)
	{
		// set error
		_errorCode = 403;
		_isError = true;
		_errorMsg = "GetProjectionMatrix -> Cannot find all corners.";
		// to return
		return false;
	}

	//std::cout << _foundedCorners << std::endl;
	// corners found, try to get four corresponding points
	Get4CornersOfChessboard();
	if (_isError)
	{
        return false;
	}

    //find transform matrix 
    const cv::Mat proj = cv::getPerspectiveTransform(_projectionPointsOrigin, _projectionPointsTarget);
    /*std::vector<cv::Point2f> temp;
    temp.emplace_back(_foundedCorners[0]);
    temp.emplace_back(_foundedCorners[CHESSBOARD_WIDTH]);
    for(auto &point : temp)
    {
        cv::rectangle(inputImage, point, cv::Point2f(point.x + 2.f, point.y + 2.f), cv::Scalar(0, 0, 255));
    }
    cv::Mat out;
    cv::warpPerspective(inputImage, out, proj, cv::Size(inputImage.cols, inputImage.rows));
    cv::imshow("corners", out);*/
    _inversePerspectiveMatrix = proj.inv();
    // copy to output
    for(auto i=0; i < 3; ++i)
        for(auto j=0; j < 3; ++j)        
            output[3*i + j] = _inversePerspectiveMatrix.at<double>(i,j);
	
	// according to table setting, calculate the real size of square in chessboard
    _squareSizeMm = tableCalibResult->mmInPixels * cv::norm(_foundedCorners[0] - _foundedCorners[1]);
    sizeOut = _squareSizeMm;

    return true;
}
