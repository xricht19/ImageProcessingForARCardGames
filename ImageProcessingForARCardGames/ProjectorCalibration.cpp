#include "ProjectorCalibration.h"
#include <opencv2/calib3d.hpp>


void ProjectorCalibration::DetectChessboardCorners(cv::Mat image)
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
        _foundedCorners = pointBuf;
    else
    {
        _isError = true;
        _errorCode = NO_ALL_CORNERS_FOUND;
        _errorMsg = "GetChessboardCorners -> Not all corners found";
    }
}

ProjectorCalibration::ProjectorCalibration()
{
}


ProjectorCalibration::~ProjectorCalibration()
{
}
