#include "CameraCalibration.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>



namespace IDAP
{
    void CameraCalibration::CreateKnownBoardPositions(std::vector<cv::Point3f>& corners)
    {
        for (int i = 0; i < _chessboardDimension.height; i++)
        {
            for (int j = 0; j < _chessboardDimension.width; j++)
            {
                corners.push_back(cv::Point3f(j * _squareDimension, i* _squareDimension, 0.0f));
            }
        }
    }

    void CameraCalibration::GetChessboardCorners(std::vector<std::vector<cv::Point2f>>& allFoundCorners, bool showResult)
    {
        for (auto &item : _calibrationImages)
        {
            std::vector<cv::Point2f> pointBuf;
            bool found = cv::findChessboardCorners(item, _chessboardDimension, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

            if (found)
            {
                allFoundCorners.push_back(pointBuf);
            }
            if(showResult)
            {
                cv::drawChessboardCorners(item, _chessboardDimension, pointBuf, found);
            }
        }
    }

    CameraCalibration::CameraCalibration()
    {
        _originInFrame = cv::Vec<int, 2>(0, 0);
        _cameramatrix = cv::Mat::eye(3, 3, CV_64F);
        _distanceCoeff = cv::Mat::zeros(8, 1, CV_64F);
    }


    CameraCalibration::~CameraCalibration()
    {
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
        cv::namedWindow("CalibrationProcess", CV_WINDOW_NORMAL);

        // find real corners
        std::vector<std::vector<cv::Point2f>> foundedCorners;
        GetChessboardCorners(foundedCorners, false);

        // all corners we should found
        std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);
        CreateKnownBoardPositions(worldSpaceCornerPoints[0]);
        
        // remap 2d founded corners to 3d expected corners
        worldSpaceCornerPoints.resize(_calibrationImages.size(), worldSpaceCornerPoints[0]);

        std::vector<cv::Mat> rVectors, tVectors;

        cv::calibrateCamera(worldSpaceCornerPoints, foundedCorners, _chessboardDimension, _cameramatrix, _distanceCoeff, rVectors, tVectors);

        for (auto &image : _calibrationImages)
        {
            cv::imshow("CalibrationProcess", image);
            cv::waitKey(1000);
        }
        cv::destroyWindow("CalibrationProcess");

    }

    void CameraCalibration::IsEnoughData(uint16_t& numberOfData) const
    {
        if (_calibrationImages.size() >= ENOUGH_IMAGES_FOR_CALIB)
            numberOfData = 1;
        else
            numberOfData = 0;
    }

    bool CameraCalibration::saveCameraCalib(std::string name)
    {
        std::ofstream outStream(name);
        if (outStream)
        {
            uint16_t rows = _cameramatrix.rows;
            uint16_t cols = _cameramatrix.cols;

            for (int r = 0; r < rows; r++)
            {
                for (int c = 0; c < cols; c++)
                {
                    double value = _cameramatrix.at<double>(r, c);
                    outStream << value << std::endl;
                }
            }

            rows = _distanceCoeff.rows;
            cols = _distanceCoeff.cols;

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
}
