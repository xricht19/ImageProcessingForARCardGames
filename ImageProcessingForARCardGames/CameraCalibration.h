#pragma once
#include <opencv2/core/mat.hpp>
#include <fstream>
#include <iostream>

#define ENOUGH_IMAGES_FOR_CALIB 25
#define MAX_IMAGES_FOR_CALIB 50

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
        uint16_t _errorCode;
        std::string _errorMsg;
        // constant for calibration
        float _squareDimension;
        cv::Size _chessboardDimension;
        // calibration result
        cv::Mat _cameraMatrix;
        cv::Mat _distanceCoeff;
        double _calibrationScore;

        void CreateKnownBoardPositions(std::vector<cv::Point3f>& corners);
        void GetChessboardCorners(std::vector<std::vector<cv::Point2f>>& allFoundCorners, bool showResult = false);

        enum ErrorStates
        {
            OK = 0,
            NO_PARAM_FOR_CALIB_SET,
            NOT_ENOUGH_IMAGES,
            NO_CORNERS_FOUND,
            UNKNOWN_IMAGE_NUMBER,
            IMAGE_TOO_BIG,
        };

    public:
        CameraCalibration();
        ~CameraCalibration() = default;

        void AddImageWithChessboard(const cv::Mat& frame);
        void Calibrate();

        /// return value > 0 if there is more than enough image for calibration 
        void IsEnoughData(uint16_t& numberOfData) const;

        bool SaveCameraCalib(std::string name);
        bool LoadCameraCalib(std::string name);

        cv::Mat GetCameraMatrix() const { return _cameraMatrix; }
        cv::Mat GetDistanceCoeff() const { return _distanceCoeff; }

        void SetSquareDimension(float value) { _squareDimension = 1000 * value; /* to meters */ }
        void SetChessboardDimension(int width, int height) { _chessboardDimension = cv::Size(width, height); }

        bool IsErrorOccure() const { return _isError; }
        std::string GetErrorMsg() const { return _errorMsg; }
        uint16_t GetErrorCode() const { return _errorCode; }

        void GetCameraCalibImage(uint16_t& errorCode, uint16_t& imgNumber, uint16_t& width, uint16_t& height, uint16_t& channels, uchar* data);

        bool IsCalibrationDone() const;
    
    };
}

