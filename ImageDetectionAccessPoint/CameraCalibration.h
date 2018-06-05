#pragma once
#include <opencv2/core/mat.hpp>
#include <fstream>
#include <iostream>

/***************************************************************
* Author: Jiri Richter
* The CameraCalibration.h file contain the class for camera 
* calibration. It helps to find camera intrinsic parameters.
****************************************************************/

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

        /// getters
        bool IsErrorOccure() const { return _isError; }
        std::string GetErrorMsg() const { return _errorMsg; }
        uint16_t GetErrorCode() const { return _errorCode; }
        cv::Mat GetCameraMatrix() const { return _cameraMatrix; }
        cv::Mat GetDistanceCoeff() const { return _distanceCoeff; }


        /**
         * \brief Function add the provided frema to vector of images for calibration.
         * \param frame Frame, which is added to images for calibration.
         */
        void AddImageWithChessboard(const cv::Mat& frame);
        /**
         * \brief Function perform the camera calibration using OpenCV functions.
         */
        void Calibrate();

        /// return value > 0 if there is more than enough image for calibration 
        void IsEnoughData(uint16_t& numberOfData) const;

        /**
         * \brief Save  camera matrix and distance coeeficints found by camera calibration.
         * \param name The name of file in which the camera calibration is saved.
         * \return True, if the saving was successfull. False, otherwise.
         */
        bool SaveCameraCalib(std::string name);
        /**
         * \brief Complementary function to SaveCameaCalib(). Loads camera matrix and distance coefficients from file specified by name.
         * \param name Name of file from which the camera calibration is loaded.
         * \return True, if the loading was successfull. False, otherwise.
         */
        bool LoadCameraCalib(std::string name);

        /**
         * \brief Set the size of chessboard square needed for camera calibration.
         * \param value Size of square in mm.
         */
        void SetSquareDimension(float value) { _squareDimension = 1000 * value; /* to meters */ }
        /**
         * \brief Set the number of square in chessboard in both dimensions.
         * \param width Number of squares in chessboard on X-axis.
         * \param height Number of squares in chessboard on Y-axis. 
         */
        void SetChessboardDimension(int width, int height) { _chessboardDimension = cv::Size(width, height); }        

        /**
         * \brief Return image from _calibrationImages vector at position specified by imgNumber.
         * \param errorCode Reference to return the error code to calledr.
         * \param imgNumber Position of image in _calibrationImaages vector.
         * \param width The width of frame returned.
         * \param height The height of frame returned.
         * \param channels The number of channels of frame returned.
         * \param data Raw image data of frame.
         */
        void GetCameraCalibImage(uint16_t& errorCode, uint16_t& imgNumber, uint16_t& width, uint16_t& height, uint16_t& channels, uchar* data);

        /**
         * \brief Check, if calibration is done.
         * \return True, if calibration was performed or loaded from file.
         */
        bool IsCalibrationDone() const;
    
    };
}

