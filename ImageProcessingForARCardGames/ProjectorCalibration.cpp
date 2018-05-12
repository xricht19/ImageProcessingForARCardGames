#include "ProjectorCalibration.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/videostab/ring_buffer.hpp>
#include <opencv2/highgui/highgui.hpp>


bool ProjectorCalibration::DetectChessboardCorners(cv::Mat image)
{
    if (_chessboardDimension.width <= 0 || _chessboardDimension.height <= 0)
    {
        _isError = true;
        _errorCode = NO_PARAM_FOR_CALIB_SET;
        _errorMsg = "GetChessboardCorners -> Chessboard dimension not set.";
        return false;
    }
    std::vector<cv::Point2f> pointBuf;
    const bool found = cv::findChessboardCorners(image, _chessboardDimension, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

	if (found) // all corners found?
	{
		_foundedCorners = pointBuf;
		return true;
	}

    _isError = true;
    _errorCode = NO_ALL_CORNERS_FOUND;
    _errorMsg = "GetChessboardCorners -> Not all corners found";
	return false;
}

void ProjectorCalibration::Get4CornersOfChessboard(cv::Size imageSize)
{
	// looking for extremes - corners sorted row by row, left to right
    std::vector<cv::Point2f> corners;
	corners.push_back(_foundedCorners[_chessboardDimension.width*(_chessboardDimension.height-1)]);
    corners.push_back(_foundedCorners[0]);
    corners.push_back(_foundedCorners[_chessboardDimension.width*_chessboardDimension.height-1]);
    corners.push_back(_foundedCorners[_chessboardDimension.width-1]);

    // set the worst possible
    cv::Point2f topLeft = cv::Point2f(imageSize.width, imageSize.height); // distance to (0, 0)
    cv::Point2f bottomLeft = cv::Point2f(imageSize.width, 0.f); // distance to (0, height)
    cv::Point2f bottomRight = cv::Point2f(0.f, 0.f); // distance to (width, height)
    cv::Point2f topRight = cv::Point2f(0.f, imageSize.height);   // distance to (width, 0)
    // calculate distances
    float TLDistance = static_cast<float>(cv::norm(topLeft - cv::Point2f(0.f, 0.f)));
    float BLDistance = static_cast<float>(cv::norm(bottomLeft - cv::Point2f(0.f, imageSize.height)));
    float BRDistance = static_cast<float>(cv::norm(bottomRight - cv::Point2f(imageSize.width, imageSize.height)));
    float TRDistance = static_cast<float>(cv::norm(topRight - cv::Point2f(imageSize.width, 0.f)));

    for(std::vector<cv::Point2f>::iterator it = corners.begin(); it != corners.end(); ++it)
    {
        float TLDistanceCurr = static_cast<float>(cv::norm((*it) - cv::Point2f(0.f,0.f)));
        float BLDistanceCurr = static_cast<float>(cv::norm((*it) - cv::Point2f(0.f, imageSize.height)));
        float BRDistanceCurr = static_cast<float>(cv::norm((*it) - cv::Point2f(imageSize.width, imageSize.height)));
        float TRDistanceCurr = static_cast<float>(cv::norm((*it) - cv::Point2f(imageSize.width, 0.f)));
        // topLeft
        if(TLDistanceCurr < TLDistance)
        {
            topLeft = (*it);
            TLDistance = TLDistanceCurr;
        }
        // bottom left
        if(BLDistanceCurr < BLDistance)
        {
            bottomLeft = (*it);
            BLDistance = BLDistanceCurr;
        }
        // bottom right
        if(BRDistanceCurr < BRDistance)
        {
            bottomRight = (*it);
            BRDistance = BRDistanceCurr;
        }
        // top right
        if(TRDistanceCurr < TRDistance)
        {
            topRight = (*it);
            TRDistance = TRDistanceCurr;
        }
    }

	// if any point are same, cancel
	if (bottomLeft == topLeft || bottomLeft == bottomRight || bottomLeft == topRight ||
		topLeft == bottomRight || topLeft == topRight || bottomRight == topRight)
	{
		_isError = true;
		_errorCode = NO_DIFFERENT_CORNERS_FOUND;
		_errorMsg = "Get4CornersOfChessboard -> No different corners found.";
		return;
	}

    // offset settings
    const float offsetX = topLeft.x;
    const float offsetY = topLeft.y;

	// calculate the desired values of points
    const float maxWidth = static_cast<float>(MAX(cv::norm(bottomRight - bottomLeft),
        cv::norm(topRight - topLeft)));
    const float maxHeight = static_cast<float>(MAX(cv::norm(topRight - bottomRight),
        cv::norm(topLeft - bottomLeft)));
    _projectionPointsOrigin.emplace_back(topLeft);
    _projectionPointsTarget.emplace_back(cv::Point2f(offsetX, offsetY));
    _projectionPointsOrigin.emplace_back(bottomLeft);
    _projectionPointsTarget.emplace_back(cv::Point2f(offsetX, offsetY+maxHeight));
    _projectionPointsOrigin.emplace_back(bottomRight); 
    _projectionPointsTarget.emplace_back(cv::Point2f(offsetX+maxWidth, offsetY+maxHeight));
    _projectionPointsOrigin.emplace_back(topRight);
    _projectionPointsTarget.emplace_back(cv::Point2f(offsetX+maxWidth, offsetY));

}

ProjectorCalibration::ProjectorCalibration() : _squareDimension(0.f), _isError(false), _errorCode(0), _topLeftDetectableChessboardCorner(0.0, 0.0)
{
}

bool ProjectorCalibration::GetProjectionMatrix(double* output, double& sizeOut, double* tableCorners, cv::Mat inputImage, TableCalibration::tableCalibrationResults* tableCalibResult)
{
    // try thresholding to better view matrix
    /*cv::Mat greyed;
    cv::cvtColor(inputImage, greyed, CV_BGR2GRAY);
    //cv::adaptiveThreshold(greyed, greyed, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 2);
    cv::threshold(greyed, greyed, 0, 255, CV_THRESH_OTSU);
    cv::imshow("thresh", greyed);
    */
    cv::imwrite("ChessboardDetection.png", inputImage);
	// try find corners
	const bool founded = DetectChessboardCorners(inputImage);
	if (!founded)
	{
		// set error
		_errorCode = NO_ALL_CORNERS_FOUND;
		_isError = true;
		_errorMsg = "GetProjectionMatrix -> Cannot find all corners.";
		// to return
		return false;
	}

	// corners found, try to get four corresponding points
	Get4CornersOfChessboard(inputImage.size());
	if (_isError)
	{
        return false;
	}
	// find left bottom corner of matrix
	_topLeftDetectableChessboardCorner = _foundedCorners[0];
	/*cv::Point2f bottomRightCorner = _foundedCorners[_foundedCorners.size() - 1];
	std::cout << "NumOfFoundedCorners: " << _foundedCorners.size() << std::endl;
	cv::Point2f chessboardCenter = (leftTopCorner + bottomRightCorner) * 0.5;
	std::cout << "chessCenter: " << chessboardCenter.x << ", " << chessboardCenter.y;
	cv::rectangle(inputImage, cv::Rect(leftTopCorner.x, leftTopCorner.y, 2, 2), cv::Scalar(255,0,0));
	cv::rectangle(inputImage, cv::Rect(bottomRightCorner.x, bottomRightCorner.y, 2, 2), cv::Scalar(255, 0, 0));
	cv::rectangle(inputImage, cv::Rect(chessboardCenter.x, chessboardCenter.y, 2, 2), cv::Scalar(0, 0, 255));*/

    /*std::cout << _projectionPointsOrigin << std::endl;
    std::cout << _projectionPointsTarget << std::endl;
    for(auto &item : _projectionPointsOrigin)
    {
        cv::rectangle(inputImage, cv::Rect(item.x,item.y, 5,5), cv::Scalar(0, 0, 255, 255));
    }
    cv::rectangle(inputImage, cv::Rect(_projectionPointsOrigin[0].x, _projectionPointsOrigin[0].y, 5, 5), cv::Scalar(255, 0, 0, 255));
    for (auto &item : _projectionPointsTarget)
    {
        cv::rectangle(inputImage, cv::Rect(item.x, item.y, 5, 5), cv::Scalar(0, 255, 0, 255));
    }

    cv::imshow("TEMP", inputImage);*/
    //find transform matrix 
    const cv::Mat proj = cv::getPerspectiveTransform(_projectionPointsOrigin, _projectionPointsTarget);
    /*std::vector<cv::Point2f> temp;
    temp.emplace_back(_foundedCorners[0]);
    temp.emplace_back(_foundedCorners[CHESSBOARD_WIDTH]);
    for(auto &point : temp)
    {
        cv::rectangle(inputImage, point, cv::Point2f(point.x + 2.f, point.y + 2.f), cv::Scalar(0, 0, 255));
    }
    cv::Mat out;
    cv::warpPerspective(inputImage, out, proj, cv::Size(inputImage.cols, inputImage.rows));
    cv::imshow("corners", out);*/


    _inversePerspectiveMatrix = proj.inv();

    // copy to output
    for(auto i=0; i < 3; ++i)
        for(auto j=0; j < 3; ++j)        
            output[3*i + j] = _inversePerspectiveMatrix.at<double>(i,j);

	/*cv::Mat temp;
	cv::warpPerspective(inputImage, temp, _inversePerspectiveMatrix, cv::Size(2 * inputImage.cols, 2 * inputImage.rows));

	cv::imshow("inv", temp);*/
	
	// according to table setting, calculate the real size of square in chessboard
    _squareSizeMm = static_cast<double>(tableCalibResult->mmInPixels * cv::norm(_foundedCorners[0] - _foundedCorners[1]));
    // outside the size in pixels is needed
    sizeOut = _squareSizeMm;

	// calculate the size of table for projector, the common known point is one corner of chessboard
	const double width = static_cast<double>(inputImage.cols);
	const double height = static_cast<double>(inputImage.rows);
	tableCorners[0] = -_foundedCorners[0].x;			// x1 -> top left
	tableCorners[1] = -_foundedCorners[0].y;			// y1
	tableCorners[2] = width* static_cast<double>(tableCalibResult->mmInPixels);
	tableCorners[3] = height* static_cast<double>(tableCalibResult->mmInPixels);
	/*tableCorners[2] = -_foundedCorners[0].x;			// x2 -> bottom left
	tableCorners[3] = height - _foundedCorners[0].y;	// y2
	tableCorners[4] = width - _foundedCorners[0].x;		// x3 -> bottom right
	tableCorners[5] = height - _foundedCorners[0].y;	// y3
	tableCorners[6] = width - _foundedCorners[0].x;		// x4 -> top right
	tableCorners[7] = -_foundedCorners[0].y;			// y4*/
    return true;
}
