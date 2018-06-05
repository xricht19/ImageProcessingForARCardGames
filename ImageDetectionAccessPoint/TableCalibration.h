#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/aruco.hpp>
#include <map>

#include <iostream>

/***************************************************************
* Author: Jiri Richter
* This file define class Table Calibration.
****************************************************************/

#define NUMBER_OF_MARKERS_GENERATED 4 //max 50
#define ARUCO_PREDEFINED_DICTIONARY cv::aruco::DICT_4X4_50
#define MARKERS_REAL_SIZE_MINIMETERS 124 //real 124, needed to find out real size of objects in pixels

class TableCalibration
{
public:
	// table calibration final values
	struct tableCalibrationResults
	{
		float mmInPixels;
		int width;
		int height;
		cv::Mat perspectiveProjectionMatrix;
	};
private:
    std::vector<int> _markersIDs;   // markers to look for, determine the order in which are the _markersCorners stored
    std::vector<std::vector<cv::Point2f>> _markersCorners /* used for store founded markers in image*/, _rejectedCandidates;
    cv::aruco::DetectorParameters _detectorParameters;  // parameters to set up the detector

    bool _calibrationDone;

    // info obtained from markers
    struct markerInfo
    {
        int ID;
        cv::Point Position; // first corner in corner list (top left corner)
        float mmInPixels;
    };
    std::map<int, markerInfo*> _markersInfos;
    tableCalibrationResults* _tableCalibResults;

    enum Corner
    {
        TOP_LEFT = 0,
        BOTTOM_LEFT,
        BOTTOM_RIGHT,
        TOP_RIGHT
    };
    enum Direction
    {
        HORIZONTAL = 0,
        VERTICAL
    };

    // calculate real size from four points in image and real size
    float GetMmInPixels(std::vector<cv::Point2f> points, float realSize);
    // mean of already known sizes from separate markers
    float GetMmInPixels();
    int GetPosistionMarkerIDZero();
    size_t GetInfoSize() { return _markersInfos.size(); }
    // clear functions
    // delete all struct in _markersInfos map
    void ClearMarkersInfos();

public:
    TableCalibration();
    ~TableCalibration();

    void InitTableCalibration();
    // return true if all four points needed for calibration were returned, otherwise false is returned
    bool HasFourPoints() const;
    // return true if table calibration was successfully performed, otherwise false is returned
    bool IsCalibrationDone() const;
    // perform markers detection
    void DetectMarkers(cv::Mat inputImage);
    
    // 
    void CalculateTableCalibrationResults(cv::Mat inputImage);
    // the matrix computed during table calibration is applied to the inputImage provide which mostly be current frame
    void ApplyTableCalibrationMatrixOnInput(cv::Mat& inputImage) const;
    // return the table calibration result struct defined above
	tableCalibrationResults* GetTableCalibrationResult() const { return _tableCalibResults; }
    
    static void CreateArucoMarkers(std::string path);
};

