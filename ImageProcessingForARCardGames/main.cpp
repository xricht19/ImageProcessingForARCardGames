#include "ImageDetectionAccessPoint.h"

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include "CameraCalibration.h"
#include "ImageDetectionAccessPointCaller.h"

#define TABLE_ID 0
#define CAMERA_CALIBRATION_FILE "IDAP_CameraCalibCoeff_kinect2"
#define CHESSBOARD_WIDTH 9
#define CHESSBOARD_HEIGHT 6
#define CHESSBOARD_SQUARE_SIZE 23.2

int main() {
	std::cout << "StartMain" << std::endl;

	IDAP::ImageDetectionAccessPoint *access = new IDAP::ImageDetectionAccessPoint();

	uint16_t errorCode = 0;
	uint16_t cameraId = 0;
	std::cout << "Size of 16 doubles: " << sizeof(double) * 16 << std::endl;
	// get list of all cameras ----------- CAMERA SELECTION ------------------------------------------
	{
		/*uint16_t numOfAvailCam;
		IDAP::ImageDetectionAccessPoint::GetNumberOfAllAvailableDevices(errorCode, numOfAvailCam);
		if (errorCode == IDAP::ImageDetectionAccessPoint::ErrorCodes::OK)
		{
			// list all devices in loop
			int temp = 0;
			int lastOpened = -1;
			cv::Mat frame;
			cv::namedWindow("CameraSelection");
			cv::VideoCapture selectCameraCapture;
			while (true)
			{
				// open stream for camera
				if (temp != lastOpened)	{
					selectCameraCapture = cv::VideoCapture(temp);
					if (!selectCameraCapture.isOpened()) {
						errorCode = IDAP::ImageDetectionAccessPoint::ErrorCodes::CANNOT_OPEN_VIDEO_STREAM;
						break;
					}
					lastOpened = temp;
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
		std::cout << "Camera Seleted" << std::endl;*/
	}

	// ------------------------------------- INIT CAMERA ---------------------------------------------
	/*access->InitImageDetectionAccessPointCamera(errorCode, cameraId);
	if (errorCode != IDAP::ImageDetectionAccessPoint::OK)
	{
		fprintf(stderr, "Cannot init IDAP!\n");
		delete(access);
		exit(1);
	}
	// IF USING KINECT FLIP THE IMAGE ON Y AXIS
	//access->SetFlipVertically();

	// ----------------------------------- CAMERA  CALIBRATION ---------------------------------------
	// check if camera calibration file exists, skip calibration in that case
	if (!access->GetCameraCalibration()->LoadCameraCalib(CAMERA_CALIBRATION_FILE))
	{
		cv::namedWindow("CameraCalib");
		uint16_t enoughData = 0;
		access->GetCameraCalibration()->SetChessboardDimension(CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT);
		access->GetCameraCalibration()->SetSquareDimension(CHESSBOARD_SQUARE_SIZE);
		access->GetCameraCalibration()->IsEnoughData(enoughData);
		int timeCounter = 0;
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

			cv::imshow("CameraCalib", access->getFrame());
			const int fps = 1000 / 25;
			char pressed = cv::waitKey(fps);       // cca 25 fps
			if (timeCounter >= 2000 / fps)                      // 2 second between catching the image
			{
				access->GetCameraCalibration()->AddImageWithChessboard(access->getFrame());
				access->GetCameraCalibration()->IsEnoughData(enoughData);
				timeCounter = 0;
			}
			else
				timeCounter++;
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
	/*int enoughNumber = 0;
	while (enoughNumber < 5) // 5 frames to let camera start
	{
		access->PrepareNextFrame(errorCode);
		cv::waitKey(1000 / 20); // 20 fps
		enoughNumber++;
	}*/
	/*const cv::Mat frame = access->getFrame();
	// detect aruco markers and show them
	TableCalibration* tblCalib = access->GetTableCalibration();

	cv::namedWindow("CalibResult", CV_WINDOW_NORMAL);
	// check calibration result
	while (true)
	{
		// prepare next frame
		access->PrepareNextFrame(errorCode);
		const cv::Mat currentFrame = access->getFrame();
		if (errorCode != IDAP::ImageDetectionAccessPoint::OK)
		{
			IDAP::ImageDetectionAccessPoint::IDAPPrintError(errorCode, std::to_string(cameraId));
			delete(access);
			exit(1);
		}

		// already have four markers, don't look for them anymore
		if (!tblCalib->HasFourPoints())
		{
			tblCalib->DetectMarkers(currentFrame);
		}
		else // we have four points, perform calibration
		{
			tblCalib->CalculateTableCalibrationResults(currentFrame);
		}

		// show
		cv::imshow("CalibResult", currentFrame);
		const char pressed = cv::waitKey(1000 / 35);
		if (pressed == 'q') // stop
		{
			break;
		}
		else if (pressed == 'r') // take point for table calibration again
		{
			tblCalib->InitTableCalibration();
		}
		cv::waitKey();
	}
	cv::destroyWindow("CalibResult");

	int i = 0;
	cv::namedWindow("ToSave");
	while (true)
	{
		access->PrepareNextFrame(errorCode);
		std::string name = "TemplateMatchignInput/Image" + std::to_string(i++) + ".png";

		cv::imshow("ToSave", access->getFrame());
		const char pressed = cv::waitKey(0);
		if (pressed == 's')
			cv::imwrite(name, access->getFrame());
		else if (pressed == 'q')
			break;
	}
	cv::destroyWindow("ToSave");

	std::cout << "mmInPixels: " << access->GetTableCalibration()->GetTableCalibrationResult()->mmInPixels << std::endl;

	// ------------------------------ PROJECTION CALIBRATION ---------------------
	/*double* matrix = new double[9]{ 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
    double size = 0.0;
	double* tableValues = new double[4]{ 0.0,0.0,0.0,0.0 };
    access->PrepareNextFrame(errorCode);
    //access->GetProjectorCalibration()->SetChessboardDimension();
    uint16_t errorCode3 = 0;
    uint16_t width = 9, height = 6;
    IDAP::SetChessboardDimensionProjectionCaller(access, errorCode3, width, height);
    //bool success = access->GetProjectorCalibration()->GetProjectionMatrix(matrix, size, tableValues, access->getFrame(), access->GetTableCalibration()->GetTableCalibrationResult());
    if (errorCode3 != 0)
        std::cout << "SET error with IDAP use!\n";
    uint16_t datAvail = 9;
    IDAP::GetProjectionTranformMatrixCaller(access, errorCode3, datAvail, size, matrix, tableValues);
	if (errorCode3 == 0)
	{
		std::cout << "Matrix:" << std::endl;
		for (int i = 0; i < 9; ++i)
			std::cout << matrix[i] << ",";

		std::cout << std::endl;
		std::cout << "square Size: " << size << std::endl;
		
		std::cout << "Table dimensions: " << std::endl;
		for (int i = 0; i < 4; ++i)
			std::cout << tableValues[i] << ", ";
	}
	else
	{
		std::cout << "Cannot find chessboard. " << errorCode3 << std::endl;
	}
	cv::namedWindow("center");
	cv::imshow("center", access->getFrame());*/
    
    // ----------------------------------- LOAD DATA FOR DETECTION ---------------------
    access->InitImageDetectionAccessPointDataAndDetection(errorCode, TABLE_ID);

	std::cout << "Loading Game Card Data" << std::endl;
    access->LoadCardData(errorCode, GAME_CARD_PATH);

	if (errorCode != 0)
	{

	}

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
		char pressed = cv::waitKey(40);
		// get next frame from camera
		/*access->PrepareNextFrame(errorCode);
		cv::imshow("Current", access->getSubSampledFrame());
		// for all players, check if area is active
		double isActive;
        bool active = false;
		for (uint16_t i = 1; i < access->GetNumberOfPlayers() + 1; ++i)
		{
			access->IsPlayerActiveByID(errorCode, i, isActive);
			if (isActive > 0)
			{
				std::cout << "Player: " << i << " is ACTIVE with intensity: " << isActive << std::endl;
                active = true;
			}
		}
        if(active)
            std::cout << "----------------\n";*/
		// TO-DO: CHECK IF CARD HAS CHANGED
		uint16_t cardID = 0;
		uint16_t cardType = 0;
		access->IsCardChangedByID(errorCode, cardID, cardType);

		
		// check card if c pressed
		if (pressed == 'c')
		{
            uint16_t cardID = 0;
            uint16_t cardType = 0;
            access->IsCardChangedByID(errorCode, cardID, cardType);
            if (errorCode != 0)
            {
                std::cout << "ERROR while card check" << std::endl;
            }
			/*for (uint16_t i = 0; i < access->GetNumberOfCardAreas(); i++)
			{
				access->IsCardChangedByID(errorCode, i, cardID);
			}*/
		}
		// exit on q pressed
		if (pressed == 'q')
			break;
	}

	// free memory
	delete(access);
	std::cout << "ALL DONE" << std::endl;

}