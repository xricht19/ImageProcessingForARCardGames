#pragma once
#include <opencv2/core/mat.hpp>
#include <fstream>
#include <iostream>

#define ENOUGH_IMAGES_FOR_CALIB 10

namespace IDAP
{
    class CameraCalibration
    {
    private:
        std::vector<cv::Mat> _calibrationImages;

        // results of calibration
        cv::Vec<int, 2> _originInFrame;


        // constant for calibration
        const float _squareDimension = 0.024f; // in meters
        const cv::Size _chessboardDimension = cv::Size(6,9);
        // calibration result
        cv::Mat _cameramatrix;
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

        bool saveCameraCalib(std::string name);

        cv::Mat GetCameraMatrix() const { return _cameramatrix; }
        cv::Mat GetDistanceCoeff() const { return _distanceCoeff; }
    };
}

