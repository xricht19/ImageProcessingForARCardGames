#pragma once
#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <map>

#include "TableCalibration.h"

#define CHESSBOARD_WIDTH_PROJ 9
#define CHESSBOARD_HEIGHT_PROJ 6

/***************************************************************
* Author: Jiri Richter
* This file define class Projector calibration.
****************************************************************/


class ProjectorCalibration
{
private:
    // constant for calibration
    float _squareDimension;
    cv::Size _chessboardDimension;

    // chessboard corners
    std::vector<cv::Point2f> _foundedCorners;
    std::vector<cv::Point2f> _projectionPointsOrigin;
    std::vector<cv::Point2f> _projectionPointsTarget;

    // results
	cv::Point2f _topLeftDetectableChessboardCorner;
    cv::Mat _inversePerspectiveMatrix;
    double _squareSizeMm;

    // error
    bool _isError;
    uint16_t _errorCode;
    std::string _errorMsg;

    bool DetectChessboardCorners(cv::Mat image);
    void Get4CornersOfChessboard(cv::Size imageSize);

    enum ErrorStates
    {
        OK = 0,
        NO_PARAM_FOR_CALIB_SET = 405,
        NO_ALL_CORNERS_FOUND = 404,
		NO_DIFFERENT_CORNERS_FOUND = 407
    };

public:
    ProjectorCalibration();
    ~ProjectorCalibration() = default;

    bool GetProjectionMatrix(double* output, double& sizeOut, double* tableCorners, cv::Mat inputImage, TableCalibration::tableCalibrationResults* tableCalibResult);

    int GetErrorCode() const { return _errorCode; }

    void SetSquareDimension(float value) { _squareDimension = 1000 * value; /* to meters, must be calculate usign data from table calibration */ }
    void SetChessboardDimension(int width = CHESSBOARD_WIDTH_PROJ, int height = CHESSBOARD_HEIGHT_PROJ) { _chessboardDimension = cv::Size(width, height); } // fixed size according to chessboard sprite in unity

};

