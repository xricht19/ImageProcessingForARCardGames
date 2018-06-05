#include "CameraCalibration.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/shape/hist_cost.hpp>
// defines includes
#include <opencv2\imgcodecs\imgcodecs_c.h>
#include <opencv2\imgproc\types_c.h>
#include <opencv2\calib3d\calib3d_c.h>



namespace IDAP
{
    void CameraCalibration::CreateKnownBoardPositions(std::vector<cv::Point3f>& corners)
    {
        if (_chessboardDimension.width <= 0 || _chessboardDimension.height <= 0 || _squareDimension <= 0.f)
        {
            _isError = true;
            _errorCode = NO_PARAM_FOR_CALIB_SET;
            _errorMsg = "CreateKnownBoardPositions -> Chessboard parameters not set.";
            return;
        }
        for (int i = 0; i < _chessboardDimension.height; i++)
        {
            for (int j = 0; j < _chessboardDimension.width; j++)
            {
                corners.emplace_back(j * _squareDimension, i* _squareDimension, 0.0f);
            }
        }
    }

    void CameraCalibration::GetChessboardCorners(std::vector<std::vector<cv::Point2f>>& allFoundCorners, bool showResult)
    {
        if (_chessboardDimension.width <= 0 || _chessboardDimension.height <= 0)
        {
            _isError = true;
            _errorCode = NO_PARAM_FOR_CALIB_SET;
            _errorMsg = "GetChessboardCorners -> Chessboard dimension not set.";
            return;
        }
        int currentCount = 0;
        for (auto &item : _calibrationImages)
        {
            std::vector<cv::Point2f> pointBuf;
            bool found = cv::findChessboardCorners(item, _chessboardDimension, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

            if (found)
            {
                allFoundCorners.push_back(pointBuf);
            }
            if (showResult)
            {
                cv::drawChessboardCorners(item, _chessboardDimension, pointBuf, found);
            }
            ++currentCount;
            if(currentCount >= MAX_IMAGES_FOR_CALIB) 
            {
                // have enough, the calculation would took too long
                break;
            }
        }
    }

    CameraCalibration::CameraCalibration()
    {
        _originInFrame = cv::Vec<int, 2>(0, 0);
        _cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        _distanceCoeff = cv::Mat::zeros(8, 1, CV_64F);
        _isError = false;
        _errorMsg = "";
        _errorCode = OK;
        _chessboardDimension = cv::Size(0, 0);
        _squareDimension = 0.0f;
        _calibrationScore = 0.0;
    }


    void CameraCalibration::AddImageWithChessboard(const cv::Mat& frame)
    {
        // copy frame and save it
        const cv::Mat frameCopy = frame.clone();
        _calibrationImages.push_back(frameCopy);
        std::cout << "Adding image" << std::endl;
    }

    void CameraCalibration::Calibrate()
    {
        // find real corners
        std::vector<std::vector<cv::Point2f>> foundedCorners;
        GetChessboardCorners(foundedCorners, true);
        if (IsErrorOccure())
        {
            return;
        }
        else if (foundedCorners.empty())
        {
            _isError = true;
            _errorMsg = "Calibrate -> No corners found in input images.";
            _errorCode = NO_CORNERS_FOUND;
            return;
        }

        // all corners we should found
        std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);
        CreateKnownBoardPositions(worldSpaceCornerPoints[0]);
        if (IsErrorOccure())
            return;

        // remap 2d founded corners to 3d expected corners
        worldSpaceCornerPoints.resize(foundedCorners.size(), worldSpaceCornerPoints[0]);
        std::vector<cv::Mat> rVectors, tVectors;
        if (_calibrationImages.size() < ENOUGH_IMAGES_FOR_CALIB)
        {
            _isError = true;
            _errorCode = NOT_ENOUGH_IMAGES;
            _errorMsg = "Calibrate -> Not enough calib images.";
            return;
        }
        _calibrationScore = cv::calibrateCamera(worldSpaceCornerPoints, foundedCorners, _chessboardDimension, _cameraMatrix, _distanceCoeff, rVectors, tVectors);

        /*cv::namedWindow("CalibrationImages", CV_WINDOW_NORMAL);
        for (auto &image : _calibrationImages)
        {
            cv::imshow("CalibrationImages", image);
            cv::waitKey(1000);
        }
        cv::destroyWindow("CalibrationImages");*/
    }

    void CameraCalibration::IsEnoughData(uint16_t& numberOfData) const
    {
        if (_calibrationImages.size() >= ENOUGH_IMAGES_FOR_CALIB)
            numberOfData = 1;
        else
            numberOfData = 0;
    }

    bool CameraCalibration::SaveCameraCalib(std::string name)
    {
        std::ofstream outStream(name);
        if (outStream)
        {
            uint16_t rows = _cameraMatrix.rows;
            uint16_t cols = _cameraMatrix.cols;

            outStream << rows << std::endl;
            outStream << cols << std::endl;

            for (int r = 0; r < rows; r++)
            {
                for (int c = 0; c < cols; c++)
                {
                    double value = _cameraMatrix.at<double>(r, c);
                    outStream << value << std::endl;
                }
            }

            rows = _distanceCoeff.rows;
            cols = _distanceCoeff.cols;

            outStream << rows << std::endl;
            outStream << cols << std::endl;

            for (int r = 0; r < rows; r++)
            {
                for (int c = 0; c < cols; c++)
                {
                    double value = _distanceCoeff.at<double>(r, c);
                    outStream << value << std::endl;
                }
            }

            outStream.close();
            return true;
        }

        return false;
    }

    bool CameraCalibration::LoadCameraCalib(std::string name)
    {
        std::ifstream inStream(name);
        if (inStream)
        {
            uint16_t rows;
            uint16_t cols;

            // camera matrix
            inStream >> rows;
            inStream >> cols;
            _cameraMatrix = cv::Mat(cv::Size(cols, rows), CV_64F);
            for (int r = 0; r < rows; r++)
            {
                for (int c = 0; c < cols; c++)
                {
                    double value = 0.0;
                    inStream >> value;
                    _cameraMatrix.at<double>(r, c) = value;
                }
            }

            // distance coefficient
            inStream >> rows;
            inStream >> cols;
            _distanceCoeff = cv::Mat::zeros(rows, cols, CV_64F);
            for (int r = 0; r < rows; r++)
            {
                for (int c = 0; c < cols; c++)
                {
                    double value = 0.0;
                    inStream >> value;
                    _distanceCoeff.at<double>(r, c) = value;
                }
            }
            inStream.close();
            return true;
        }

        return false;
    }

    void CameraCalibration::GetCameraCalibImage(uint16_t& errorCode, uint16_t& imgNumber, uint16_t& width, uint16_t& height,
        uint16_t& channels, uchar* data)
    {
        if(imgNumber > _calibrationImages.size())
        {
            errorCode = UNKNOWN_IMAGE_NUMBER;
            return;
        }
        cv::Mat image = _calibrationImages[imgNumber];
        // is enough space allocated?
        if((image.rows*image.cols*image.channels()) < width*height*channels)
        {
            errorCode = IMAGE_TOO_BIG;
            return;
        }
        // copy data
        cv::Mat rgbImage;
        cv::cvtColor(image, rgbImage, CV_BGR2RGBA);
        std::memcpy(data, rgbImage.data, rgbImage.total() * rgbImage.elemSize());

    }

    bool CameraCalibration::IsCalibrationDone() const
    {
        return !(_cameraMatrix.empty() || _distanceCoeff.empty());
    }
}
