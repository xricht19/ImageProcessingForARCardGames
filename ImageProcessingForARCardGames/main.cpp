#include "ImageDetectionAccessPoint.h"

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include "CameraCalibration.h"

#define TABLE_ID 0
#define CAMERA_CALIBRATION_FILE "IDAP_CameraCalibCoeff"


int main() {
	std::cout << "StartMain" << std::endl;

	IDAP::ImageDetectionAccessPoint *access = new IDAP::ImageDetectionAccessPoint();
	
	uint16_t errorCode = 0;
    uint16_t cameraId = 0;
    // get list of all cameras ----------- CAMERA SELECTION ------------------------------------------
    /*{
        uint16_t numOfAvailCam;
        IDAP::ImageDetectionAccessPoint::GetNumberOfAllAvailableDevices(errorCode, numOfAvailCam);
        if (errorCode == IDAP::ImageDetectionAccessPoint::ErrorCodes::OK)
        {
            // list all devices in loop
            int temp = 0;
            cv::Mat frame;
            cv::namedWindow("CameraSelection");
            while (true)
            {
                // open stream for camera
                cv::VideoCapture selectCameraCapture = cv::VideoCapture(temp);
                if (!selectCameraCapture.isOpened()) {
                    errorCode = IDAP::ImageDetectionAccessPoint::ErrorCodes::CANNOT_OPEN_VIDEO_STREAM;
                    break;
                }
                selectCameraCapture >> frame;
                const cv::Size newFrSize = cv::Size(500, frame.rows*(500.0 / static_cast<float>(frame.cols)));
                cv::resize(frame, frame, newFrSize);
                cv::imshow("CameraSelection", frame);
                const char pressed = cv::waitKey(1000);
                if (pressed == 's')
                {
                    // destroy windows
                    cv::destroyWindow("CameraSelection");
                    selectCameraCapture.release();
                    cameraId = temp;
                    break;
                }
                temp = (temp + 1) % numOfAvailCam;
            }
        }

        if (errorCode != IDAP::ImageDetectionAccessPoint::ErrorCodes::OK)
        {
            IDAP::ImageDetectionAccessPoint::IDAPPrintError(errorCode, std::to_string(cameraId));
            delete(access);
            exit(1);
        }
        std::cout << "Camera Seleted" << std::endl;
    }*/

	// ------------------------------------- INIT CAMERA ---------------------------------------------
	std::string path = "ARBang/Settings0.xml";
	access->InitImageDetectionAccessPointCamera(errorCode, cameraId);

    // ----------------------------------- CAMERA  CALIBRATION ---------------------------------------
    // check if camera calibration file exists, skip calibration in that case
    if (!access->GetCameraCalibration()->LoadCameraCalib(CAMERA_CALIBRATION_FILE))
    {
        cv::namedWindow("CameraCalib");
        uint16_t enoughData = 0;
        access->GetCameraCalibration()->IsEnoughData(enoughData);
        while (!enoughData)
        {
            // prepare next frame
            access->PrepareNextFrame(errorCode);
            if (errorCode != IDAP::ImageDetectionAccessPoint::OK)
            {
                IDAP::ImageDetectionAccessPoint::IDAPPrintError(errorCode, std::to_string(cameraId));
                delete(access);
                exit(1);
            }

            cv::imshow("CameraCalib", access->getSubSampledFrame());
            char pressed = cv::waitKey(2000);       // wait two seconds between each insert, to let user move the chessboard
            access->GetCameraCalibration()->AddImageWithChessboard(access->getFrame());
            access->GetCameraCalibration()->IsEnoughData(enoughData);
        }
        cv::destroyWindow("CameraCalib");
        // have enough data, calibrating
        std::cout << "Calibration Started" << std::endl;
        access->GetCameraCalibration()->Calibrate();
        if (access->GetCameraCalibration()->IsErrorOccure())
        {
            delete(access);
            exit(1);
        }
        access->GetCameraCalibration()->SaveCameraCalib(CAMERA_CALIBRATION_FILE);
        std::cout << "Camera Calibrated" << std::endl;
    }

    // ----------------------------------- TABLE CALIBRATION - ARUCO ---------------------------------
    //TableCalibration::CreateArucoMarkers("arUcoMarkers");
    int enoughNumber = 0;
    while (enoughNumber < 5) // 5 frames to let camera start
    {
        access->PrepareNextFrame(errorCode);
        cv::waitKey(1000 / 20); // 20 fps 
        enoughNumber++;
    }
    cv::Mat frame = access->getFrame();
    cv::Mat undistFrame;
    cv::undistort(frame, undistFrame, access->GetCameraCalibration()->GetCameraMatrix(), access->GetCameraCalibration()->GetDistanceCoeff());
    // detect aruco markers and show them
    TableCalibration* tblCalib = access->GetTableCalibration();
    
    cv::namedWindow("CalibTest_undistorted", CV_WINDOW_NORMAL);
    cv::namedWindow("CalibTest_distorted", CV_WINDOW_NORMAL);

    int nameId = 0;
    // check calibration result
    while (true)
    {
        // prepare next frame
        access->PrepareNextFrame(errorCode);
        if (errorCode != IDAP::ImageDetectionAccessPoint::OK)
        {
            IDAP::ImageDetectionAccessPoint::IDAPPrintError(errorCode, std::to_string(cameraId));
            delete(access);
            exit(1);
        }

        cv::Mat frame = access->getFrame();
        cv::Mat undistFrame;
        cv::undistort(frame, undistFrame, access->GetCameraCalibration()->GetCameraMatrix(), access->GetCameraCalibration()->GetDistanceCoeff());
        // already have four markers, don't look for them anymore
        if (!tblCalib->HasFourPoints())
        {
            tblCalib->InitTableCalibration();
            tblCalib->DetectMarkers(undistFrame);
            tblCalib->DrawDetectedMarkersInImage(undistFrame);
        }

        // show
        cv::imshow("CalibTest_undistorted", undistFrame);
        cv::imshow("CalibTest_distorted", frame);
        char pressed = cv::waitKey(1000 / 35);
        if (pressed == 'q') // stop
        {
            break;
        }
        else if (pressed == 'r') // take point for table calibration again
        {
            tblCalib->InitTableCalibration();
        }

        /*cv::imwrite("undist" + std::to_string(nameId) + ".png", undistFrame);
        cv::imwrite("dist" + std::to_string(nameId) + ".png", frame);*/

        nameId++;

    }
    cv::destroyWindow("CalibTest");
    
    // ----------------------------------- LOAD DATA FOR DETECTION ---------------------
    access->InitImageDetectionAccessPointData(errorCode, path.data(), TABLE_ID);

	std::cout << "Loading Game Card Data" << std::endl;
    access->LoadCardData("ARBang/gameCardData");

    std::cout << "Data Loaded" << std::endl;

	/*
	for (auto &item : *(access->GetGameCardData()))
	{
		cv::imshow("Current", item.second);
		cv::waitKey();
	}
	*/

    // ----------------------------- GAME INPUT MAIN LOOP ----------------------------------------------
    uint16_t cardID = 0;
    cv::namedWindow("Current");

	while (true)
	{
		char pressed = cv::waitKey(1);
		// get next frame from camera
		access->PrepareNextFrame(errorCode);
		cv::imshow("Current", access->getSubSampledFrame());
		// for all players, check if area is active
		uint16_t isActive;
		for (uint16_t i = 1; i < access->GetNumberOfPlayers() + 1; i++)
		{
			access->IsPlayerActiveByID(errorCode, i, isActive);
			if (isActive)
			{
				std::cout << "Player: " << i << " is ACTIVE!" << std::endl;
			}
		}
		// TO-DO: CHECK IF CARD HAS CHANGED
		
		// check card if c pressed
		if (pressed == 'c')
		{
			for (uint16_t i = 0; i < access->GetNumberOfCardAreas(); i++)
			{
				access->IsCardChangedByID(errorCode, i, cardID);
			}
		}
		// exit on q pressed
		if (pressed == 'q')
			break;
	}

	// free memory
	delete(access);
	std::cout << "ALL DONE" << std::endl;

}