#pragma once
#include <opencv2/core/mat.hpp>
#include <fstream>
#include <iostream>

#define ENOUGH_IMAGES_FOR_CALIB 15
#define CHESSBOARD_WIDTH 6
#define CHESSBOARD_HEIGHT 9
#define CHESSBOARD_SQUARE_SIZE 0.024f

namespace IDAP
{
    class CameraCalibration
    {
    private:
        std::vector<cv::Mat> _calibrationImages;

        // results of calibration
        cv::Vec<int, 2> _originInFrame;

        // error
        bool _isError;

        // constant for calibration
        const float _squareDimension = CHESSBOARD_SQUARE_SIZE; // in meters
        const cv::Size _chessboardDimension = cv::Size(CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT);
        // calibration result
        cv::Mat _cameraMatrix;
        cv::Mat _distanceCoeff;

        void CreateKnownBoardPositions(std::vector<cv::Point3f>& corners);
        void GetChessboardCorners(std::vector<std::vector<cv::Point2f>>& allFoundCorners, bool showResult = false);

    public:
        CameraCalibration();
        ~CameraCalibration();

        void AddImageWithChessboard(const cv::Mat& frame);
        void Calibrate();

        /// return value > 0 if there is more than 4 image for calibration 
        void IsEnoughData(uint16_t& numberOfData) const;

        bool SaveCameraCalib(std::string name);
        bool LoadCameraCalib(std::string name);

        cv::Mat GetCameraMatrix() const { return _cameraMatrix; }
        cv::Mat GetDistanceCoeff() const { return _distanceCoeff; }

        bool IsErrorOccure() const { return _isError; }
    };
}

