#include "TableCalibration.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videostab/ring_buffer.hpp>

TableCalibration::TableCalibration()
{
    InitTableCalibration();
}


TableCalibration::~TableCalibration()
{
    ClearMarkersInfos();
}

void TableCalibration::ClearMarkersInfos()
{
    for (auto &item : _markersInfos)
    {
        if (item.second != nullptr) // should not be needed, just a fuse
            delete(item.second);
    }
    _markersInfos.clear();
}



float TableCalibration::GetCmInPixels(std::vector<cv::Point2f> points, float realSize, Direction d)
{
    float distance = 0;
    // take two point next to each other and calculate distance
    if (d == HORIZONTAL)
    {
        distance += cv::norm(points[TOP_LEFT] - points[TOP_RIGHT]);
        distance += cv::norm(points[BOTTOM_LEFT] - points[BOTTOM_RIGHT]);
    }
    else 
    {
        distance += cv::norm(points[TOP_LEFT] - points[BOTTOM_LEFT]);
        distance += cv::norm(points[TOP_RIGHT] - points[BOTTOM_RIGHT]);
    }
    return (distance / 2.0f)/realSize;
}

float TableCalibration::GetCmInPixels(Direction d)
{
    float value = 0.f;
    for (auto &item : _markersInfos)
    {
        if (d == VERTICAL)
            value += item.second->cmInPixelHeight;
        else
            value += item.second->cmInPixelsWidth;
    }
    return value / _markersInfos.size();
}


void TableCalibration::InitTableCalibration()
{
    _markersIDs.clear();
    _markersCorners.clear();
    if (_tableCalibResults != nullptr)
        delete(_tableCalibResults);
    ClearMarkersInfos();
    _tableCalibResults = new tableCalibrationResults;
}

bool TableCalibration::HasFourPoints() const
{
    if(_markersIDs.size() == 4)
    {
        return true;
    }
    return false;
}

void TableCalibration::DetectMarkers(cv::Mat inputImage)
{
    const cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(ARUCO_PREDEFINED_DICTIONARY);
    const cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    /*detectorParams->adaptiveThreshWinSizeMin = 3;
    detectorParams->adaptiveThreshWinSizeMax = 23;
    detectorParams->adaptiveThreshWinSizeStep = 10;
    detectorParams->adaptiveThreshConstant = 7;
    detectorParams->minMarkerPerimeterRate = 0.03;
    detectorParams->maxMarkerPerimeterRate = 4.0;
    detectorParams->polygonalApproxAccuracyRate = 0.05;
    detectorParams->minCornerDistanceRate = 0.05;
    detectorParams->minDistanceToBorder = 3;
    detectorParams->minMarkerDistanceRate = 0.05;
    detectorParams->cornerRefinementWinSize = 5;
    detectorParams->cornerRefinementMaxIterations = 30;
    detectorParams->cornerRefinementMinAccuracy = 0.1;
    detectorParams->markerBorderBits = 1;
    detectorParams->perspectiveRemovePixelPerCell = 8;
    detectorParams->perspectiveRemoveIgnoredMarginPerCell = 0.13;
    detectorParams->maxErroneousBitsInBorderRate = 0.35;
    detectorParams->minOtsuStdDev = 5.0;
    detectorParams->errorCorrectionRate = 0.6;*/

    cv::aruco::detectMarkers(inputImage, markerDictionary, _markersCorners, _markersIDs, detectorParams, _rejectedCandidates);
}

void TableCalibration::DrawDetectedMarkersInImage(cv::Mat inputImage)
{
    cv::namedWindow("arUco");
    std::cout << _markersIDs.size() << std::endl;
    cv::aruco::drawDetectedMarkers(inputImage, _markersCorners, _markersIDs, cv::Scalar(0,0,255));
	/*for (int i = 0; i < _markersCorners.size(); i++)
		std::cout << _markersCorners[i] << std::endl;*/
	/*for (int i = 0; i < _rejectedCandidates.size(); i++)
	{
		cv::rectangle(inputImage, _rejectedCandidates[i][0], _rejectedCandidates[i][2], cv::Scalar(255, 0, 0), 2);
        cv::rectangle(inputImage, _rejectedCandidates[i][1], _rejectedCandidates[i][3], cv::Scalar(255, 0, 0), 2);
	}*/
    cv::imshow("arUco", inputImage);
}

void TableCalibration::CalculateTableCalibrationResults(cv::Mat inputImage)
{
    // generate marker info for each marker
    for (auto i = 0; i < _markersIDs.size(); i++)
    {
        struct markerInfo* mrkrInf = new markerInfo;
        mrkrInf->ID = _markersIDs[i];
        mrkrInf->Position = _markersCorners[i][0]; // top left corner
        mrkrInf->realSizeInCm = MARKERS_REAL_SIZE_CENTIMETERS;
        mrkrInf->cmInPixelsWidth = GetCmInPixels(_markersCorners[i], mrkrInf->realSizeInCm, HORIZONTAL);
        mrkrInf->cmInPixelHeight = GetCmInPixels(_markersCorners[i], mrkrInf->realSizeInCm, VERTICAL);
        // finaly push to vector
        _markersInfos.insert(std::make_pair(_markersIDs[i], mrkrInf));
    }

    // calculate projection matrix to undistort the image
    std::vector<cv::Point2f> projectionPointsSource(_markersIDs.size()); // contain four corners of table from image, starting with top left to
    std::vector<cv::Point2f> projectionPointsTarget(_markersIDs.size()); // contain four corners of table, starting with top left to
    for (int i = 0; i < _markersIDs.size(); i++)
    {
        projectionPointsSource.at(_markersIDs[i]) = _markersCorners[i][0];
    }

    // calculate the point it should reflect
    // calculate new dimension of image defined by markers
    const float maxWidth = MAX(cv::norm(projectionPointsSource[BOTTOM_RIGHT] - projectionPointsSource[BOTTOM_LEFT]),
        cv::norm(projectionPointsSource[TOP_RIGHT] - projectionPointsSource[TOP_LEFT]));
    const float maxHeight = MAX(cv::norm(projectionPointsSource[TOP_RIGHT] - projectionPointsSource[BOTTOM_RIGHT]),
        cv::norm(projectionPointsSource[TOP_LEFT] - projectionPointsSource[BOTTOM_LEFT]));
    // create new points and push them to vector
    projectionPointsTarget[TOP_LEFT] = cv::Point2f(0, 0);
    projectionPointsTarget[BOTTOM_LEFT] = cv::Point2f(0, maxHeight);
    projectionPointsTarget[BOTTOM_RIGHT] = cv::Point2f(maxWidth, maxHeight);
    projectionPointsTarget[TOP_RIGHT] = cv::Point2f(maxWidth, 0);

    // calculate projection matrix
    _tableCalibResults->perspectiveProjectionMatrix = cv::getPerspectiveTransform(projectionPointsSource, projectionPointsTarget);
    _tableCalibResults->cmInPixelsWidth = GetCmInPixels(HORIZONTAL);
    _tableCalibResults->cmInPixelHeight = GetCmInPixels(VERTICAL);
    cv::Mat warped;
    cv::warpPerspective(inputImage, warped, _tableCalibResults->perspectiveProjectionMatrix, cv::Size(maxWidth, maxHeight));
    cv::rectangle(warped, cv::Point2f(warped.size().width/2, warped.size().height/2), cv::Point2f(12.4 * _tableCalibResults->cmInPixelsWidth + warped.size().width / 2, 12.4 * _tableCalibResults->cmInPixelHeight + warped.size().height / 2), cv::Scalar(0, 0, 255));

    cv::imshow("markerCorners", warped);
}

void TableCalibration::CreateArucoMarkers(std::string path)
{
    cv::Mat outputMarker;
    const cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(ARUCO_PREDEFINED_DICTIONARY);
    
    // generate markers, limit is 50
    for(int i = 0; i < (NUMBER_OF_MARKERS_GENERATED < 50 ? NUMBER_OF_MARKERS_GENERATED : 50); i++)
    {
        cv::aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
        std::ostringstream convert;
        const std::string imageName = "4x4Marker_";
        convert << path << "/" << imageName << i << ".jpg";
        cv::imwrite(convert.str(), outputMarker);
    }
}
