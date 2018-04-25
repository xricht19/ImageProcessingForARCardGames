#pragma once
#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <map>

#include "TableCalibration.h"

#define CHESSBOARD_WIDTH_PROJ 9
#define CHESSBOARD_HEIGHT_PROJ 6


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
    cv::Mat _inversePerspectiveMatrix;
    float _squareSizeMm;

    // error
    bool _isError;
    uint16_t _errorCode;
    std::string _errorMsg;

    bool DetectChessboardCorners(cv::Mat image);
    void Get4CornersOfChessboard();

    enum ErrorStates
    {
        OK = 0,
        NO_PARAM_FOR_CALIB_SET,
        NO_ALL_CORNERS_FOUND,
		NO_DIFFERENT_CORNERS_FOUND
    };

public:
    ProjectorCalibration();
    ~ProjectorCalibration() = default;

    bool GetProjectionMatrix(double* output, double& sizeOut, cv::Mat inputImage, TableCalibration::tableCalibrationResults* tableCalibResult);

    void SetSquareDimension(float value) { _squareDimension = 1000 * value; /* to meters, must be calculate usign data from table calibration */ }
    void SetChessboardDimension(int width = CHESSBOARD_WIDTH_PROJ, int height = CHESSBOARD_HEIGHT_PROJ) { _chessboardDimension = cv::Size(width, height); } // fixed size according to chessboard sprite in unity

};

