#include "ImageDetectionAccessPoint.h"

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include "CameraCalibration.h"

#define TABLE_ID 0



int main() {
	std::cout << "StartMain" << std::endl;

	IDAP::ImageDetectionAccessPoint *access = new IDAP::ImageDetectionAccessPoint();
	
	uint16_t errorCode = 0;
    uint16_t cameraId = 0;
    // get list of all cameras ----------- CAMERA SELECTION ------------------------------------------
    {
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
    }

	// ------------------------------------- INIT CAMERA ---------------------------------------------
	std::string path = "ARBang/Settings0.xml";
	access->InitImageDetectionAccessPointCamera(errorCode, cameraId);

    // ----------------------------------- CAMERA  CALIBRATION ---------------------------------------
    {
        cv::namedWindow("CameraCalibration");
        uint16_t enoughData = 0;
        access->GetCalibration()->IsEnoughData(enoughData);
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

            cv::Mat frame = access->getFrame();
            cv::Mat frameForShow;
            // resize for showing
            const cv::Size newFrSize = cv::Size(500, frame.rows*(500.0 / static_cast<float>(frame.cols)));
            cv::resize(frame, frameForShow, newFrSize);
            cv::imshow("CameraCalibration", frameForShow);
            char pressed = cv::waitKey(40);         
            if (pressed == 'i') // inserts
            {
                access->GetCalibration()->AddImageWithChessboard(frame); // not resized one for better results
                access->GetCalibration()->IsEnoughData(enoughData);
            }
        }
        cv::destroyWindow("CameraCalibration");
        // have enough data, calibrating
        std::cout << "Calibration Started" << std::endl;
        access->GetCalibration()->Calibrate();
        access->GetCalibration()->saveCameraCalib("TestCalibOutput");
        std::cout << "Camera Calibrated" << std::endl;
    }

    cv::namedWindow("CalibTest");

    cv::VideoWriter undistorted;
    cv::VideoWriter distorted;

    

    undistorted.open("Undistorted.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 15, access->getFrame().size());
    distorted.open("distorted.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 15, access->getFrame().size());

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
        cv::undistort(frame, undistFrame, access->GetCalibration()->GetCameraMatrix(), access->GetCalibration()->GetDistanceCoeff());

        // resize for showing
        cv::imshow("CalibTest", undistFrame);
        char pressed = cv::waitKey(1000 / 20);
        if (pressed == 'q') // stop
        {
            undistorted.release();
            distorted.release();
            break;
        }

        // save video with distorted and undistorted image
        undistorted.write(undistFrame);
        distorted.write(frame);

        cv::imwrite("undist" + std::to_string(nameId) + ".png", undistFrame);
        cv::imwrite("dist" + std::to_string(nameId) + ".png", frame);

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