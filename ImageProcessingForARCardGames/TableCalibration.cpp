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



float TableCalibration::GetCmInPixels(std::vector<cv::Point2f> points, float realSize)
{
    float distance = 0;
    const int numOfElem = points.size();
    // take two point next to each other and calculate distance
    for (int i = 0; i < numOfElem; i++)
    {
        const int i_1 = (i + 1) % numOfElem;
        distance += cv::norm(points[i] - points[i_1]);
    }
    return (distance / static_cast<float>(numOfElem))/realSize;
}

float TableCalibration::GetCmInPixels()
{
    float value = 0.f;
    for (auto &item : _markersInfos)
    {
        value += item.second->cmInPixels;
    }
    return value / _markersInfos.size();
}

int TableCalibration::GetPosistionMarkerIDZero()
{
    for (int i = 0; i < _markersIDs.size(); i++)
    {
        if (_markersIDs[i] == 0)
            return i;
    }
    return 0;
}


void TableCalibration::InitTableCalibration()
{
    _markersIDs.clear();
    _markersCorners.clear();
    if (_tableCalibResults != nullptr)
        delete(_tableCalibResults);
    ClearMarkersInfos();
    _tableCalibResults = new tableCalibrationResults;
    _calibrationDone = false;
}

bool TableCalibration::HasFourPoints() const
{
    if(_markersIDs.size() == 4)
    {
        return true;
    }
    return false;
}

bool TableCalibration::IsCalibrationDone() const
{
    return _calibrationDone;
}

void TableCalibration::DetectMarkers(cv::Mat inputImage)
{
    InitTableCalibration();
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
    cv::aruco::drawDetectedMarkers(inputImage, _markersCorners, _markersIDs, cv::Scalar(0, 0, 255));
}


void TableCalibration::CalculateTableCalibrationResults(cv::Mat inputImage)
{
    // generate marker info for each marker
    for (auto i = 0; i < _markersIDs.size(); i++)
    {
        struct markerInfo* mrkrInf = new markerInfo;
        mrkrInf->ID = _markersIDs[i];
        mrkrInf->Position = _markersCorners[i][0]; // top left corner
        mrkrInf->cmInPixels = GetCmInPixels(_markersCorners[i], MARKERS_REAL_SIZE_CENTIMETERS);
        // finaly push to vector
        _markersInfos.insert(std::make_pair(_markersIDs[i], mrkrInf));
    }

    // calculate projection matrix to undistort the image
    const int infoSize = GetInfoSize();
    std::vector<cv::Point2f> projectionPointsSource(infoSize); // contain four corners of table from image, starting with top left to
    std::vector<cv::Point2f> projectionPointsTarget(infoSize); // contain four corners of table, starting with top left to
    for (int i = 0; i < infoSize; i++)
        projectionPointsSource.at(i) = _markersInfos[i]->Position;

    // create corresponding points and push them to vector
    // calculate new dimension of image defined by markers
    const float maxWidth = MAX(cv::norm(projectionPointsSource[BOTTOM_RIGHT] - projectionPointsSource[BOTTOM_LEFT]),
        cv::norm(projectionPointsSource[TOP_RIGHT] - projectionPointsSource[TOP_LEFT]));
    const float maxHeight = MAX(cv::norm(projectionPointsSource[TOP_RIGHT] - projectionPointsSource[BOTTOM_RIGHT]),
        cv::norm(projectionPointsSource[TOP_LEFT] - projectionPointsSource[BOTTOM_LEFT]));    
    projectionPointsTarget[TOP_LEFT] = cv::Point2f(0, 0);
    projectionPointsTarget[BOTTOM_LEFT] = cv::Point2f(0, maxHeight);
    projectionPointsTarget[BOTTOM_RIGHT] = cv::Point2f(maxWidth, maxHeight);
    projectionPointsTarget[TOP_RIGHT] = cv::Point2f(maxWidth, 0);

    // calculate projection matrix
    _tableCalibResults->perspectiveProjectionMatrix = cv::getPerspectiveTransform(projectionPointsSource, projectionPointsTarget);
    _tableCalibResults->cmInPixels = GetCmInPixels();
    _tableCalibResults->height = maxHeight;
    _tableCalibResults->width = maxWidth;
    _calibrationDone = true;
}

void TableCalibration::ApplyTableCalibrationMatrixOnInput(cv::Mat& inputImage) const
{
    cv::Mat warped;
    cv::warpPerspective(inputImage, warped, _tableCalibResults->perspectiveProjectionMatrix, cv::Size(_tableCalibResults->width, _tableCalibResults->height));
    inputImage = warped;
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
