#pragma once
#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <map>

#define CHESSBOARD_WIDTH 6
#define CHESSBOARD_HEIGHT 9


class ProjectorCalibration
{
private:
    // constant for calibration
    float _squareDimension;
    cv::Size _chessboardDimension;
    // chessboard corners
    std::vector<cv::Point2f> _foundedCorners;
    std::pair<cv::Point2f, cv::Point2f> _projectionPoints;
    // error
    bool _isError;
    uint16_t _errorCode;
    std::string _errorMsg;

    void DetectChessboardCorners(cv::Mat image);
    void Get4CornersOfChessboard();

    enum ErrorStates
    {
        OK = 0,
        NO_PARAM_FOR_CALIB_SET,
        NO_ALL_CORNERS_FOUND
    };

public:
    ProjectorCalibration();
    ~ProjectorCalibration();

    void GetProjectionMatrix(uint16_t chessBoardFounded, float* output, cv::Mat inputImage);

    void SetSquareDimension(float value) { _squareDimension = 1000 * value; /* to meters, must be calculate usign data from table calibration */ }
    void SetChessboardDimension(int width, int height) { _chessboardDimension = cv::Size(CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT); } // fixed size according to chessboard sprite in unity

};

