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
}

void TableCalibration::InitTableCalibration()
{
    _markersIDs.clear();
    _markersCorners.clear();
}

bool TableCalibration::HasFourPoints()
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
    cv::aruco::detectMarkers(inputImage, markerDictionary, _markersCorners, _markersIDs, cv::aruco::DetectorParameters::create(), _rejectedCandidates);
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
	}*/
    cv::imshow("arUco", inputImage);
}

void TableCalibration::CalculateTableCalibrationResults()
{
    // generate marker info for each 

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
