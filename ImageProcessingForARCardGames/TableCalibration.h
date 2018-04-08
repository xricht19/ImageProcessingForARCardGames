#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/aruco.hpp>
#include <map>

#include <iostream>

#define NUMBER_OF_MARKERS_GENERATED 4 //max 50
#define ARUCO_PREDEFINED_DICTIONARY cv::aruco::DICT_4X4_50
#define MARKERS_REAL_SIZE_CENTIMETERS 12

class TableCalibration
{
private:
    std::vector<int> _markersIDs;   // markers to look for, determine the order in which are the _markersCorners stored
    std::vector<std::vector<cv::Point2f>> _markersCorners /* used for store founded markers in image*/, _rejectedCandidates;
    cv::aruco::DetectorParameters _detectorParameters;  // parameters to set up the detector

    // info obtained from markers
    struct markerInfo
    {
        int ID;
        cv::Point Position;
        float cmInPixels;
        int realSizeInCm;
    };
    std::map<int, markerInfo> markersInfos;
    // table calibration final values
    struct tableCalibrationResults
    {
        float cmInPixels;
        int xPos;
        int yPos;
        int width;
        int height;
        cv::Mat affineTransformMatrix;
    };

public:
    TableCalibration();
    ~TableCalibration();

    void InitTableCalibration();
    bool HasFourPoints();
    void DetectMarkers(cv::Mat inputImage);
    void DrawDetectedMarkersInImage(cv::Mat inputImage);
    
    void CalculateTableCalibrationResults();

    static void CreateArucoMarkers(std::string path);
};

