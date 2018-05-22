#include "ImageDetectionAccessPoint.h"
#include "ImageDetectionAccessPointCaller.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "CameraCalibration.h"
#include "ImageDetectionAccessPointCaller.h"

/***************************************************************
* Author: Jiri Richter
* The main.cpp contain testing sequecies, which were needed
* during the developing of IDAP library. It is not supposed
* to be run separately as self standing program.
****************************************************************/

#define TABLE_ID 0
#define CAMERA_CALIBRATION_FILE "IDAP_CameraCalibCoeff_kinect2"
#define CHESSBOARD_WIDTH 9
#define CHESSBOARD_HEIGHT 6
#define CHESSBOARD_SQUARE_SIZE 23.2

int main() {
	std::cout << "StartMain" << std::endl;

    /*int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    // template matching example
    cv::Mat rawImg = cv::imread("Bang_06.png", CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat rawTempl = cv::imread("1.png", CV_LOAD_IMAGE_GRAYSCALE);

    const float ratio = static_cast<float>(rawImg.size().width) / static_cast<float>(rawImg.size().height);   
    cv::Mat temp;
    const int newHeight = 60.f / ratio;
    const cv::Size ss(60, newHeight);
    cv::resize(rawTempl, temp, ss);

    rawTempl = temp;

    cv::Mat grad_x, grad_y, grad_x_templ, grad_y_templ;
    cv::Mat abs_grad_x, abs_grad_y, abs_grad_x_templ, abs_grad_y_templ;
    cv::Mat grad, gradTemple;

    /// Gradient X
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    Sobel(rawImg, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    convertScaleAbs(grad_x, abs_grad_x);

    Sobel(rawTempl, grad_x_templ, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    convertScaleAbs(grad_x_templ, abs_grad_x_templ);

    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel(rawImg, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    convertScaleAbs(grad_y, abs_grad_y);

    Sobel(rawTempl, grad_y_templ, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    convertScaleAbs(grad_y_templ, abs_grad_y_templ);
 
    /// Total Gradient (approximate)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
    cv::addWeighted(abs_grad_x_templ, 0.5, abs_grad_y_templ, 0.5, 0, gradTemple);

    cv::Mat img_display, result;
    rawImg.copyTo(img_display);
    const int result_cols = rawImg.cols - rawTempl.cols + 1;
    const int result_rows = rawImg.rows - rawTempl.rows + 1;


    cv::imshow("img", grad);
    cv::imshow("templ", gradTemple);

    result.create(result_rows, result_cols, CV_32FC1);
    cv::matchTemplate(rawImg, rawTempl, result, CV_TM_CCORR_NORMED);

    double minVal; double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
    const cv::Point matchLoc = maxLoc;

    /// Show me what you got
    rectangle(img_display, matchLoc, cv::Point(matchLoc.x + rawTempl.cols, matchLoc.y + rawTempl.rows), cv::Scalar::all(0), 2, 8, 0);

    cv::imshow("image_window", img_display);
    cv::imshow("result_window", result);

    cv::Mat result8;
    result.convertTo(result8, CV_8UC1, 255.0);
    cv::imshow("result", result8);

    cv::imwrite("tmResultRaw.png", result8);
    cv::imwrite("tmInputRaw.png", grad);
    cv::imwrite("tmTemplRaw.png", gradTemple);

    cv::waitKey(0);
    exit(1);*/


	IDAP::ImageDetectionAccessPoint *access = new IDAP::ImageDetectionAccessPoint();

	uint16_t errorCode = 0;
	uint16_t cameraId = 0;
	std::cout << "Size of 16 doubles: " << sizeof(double) * 16 << std::endl;
	// get list of all cameras ----------- CAMERA SELECTION ------------------------------------------
	{
		uint16_t numOfAvailCam = 0;
		//IDAP::ImageDetectionAccessPoint::GetNumberOfAllAvailableDevices(errorCode, numOfAvailCam);
        std::cout << "Avail devicss: " << numOfAvailCam << std::endl;
		/*if (errorCode == IDAP::ImageDetectionAccessPoint::ErrorCodes::OK)
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
	access->InitImageDetectionAccessPointCamera(errorCode, cameraId);
	if (errorCode != IDAP::ImageDetectionAccessPoint::OK)
	{
		fprintf(stderr, "Cannot init IDAP!\n");
		delete(access);
		exit(1);
	}
	// IF USING KINECT FLIP THE IMAGE ON Y AXIS
	access->SetFlipVertically();

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
	const cv::Mat frame = access->getFrame();
	// detect aruco markers and show them
	TableCalibration* tblCalib = access->GetTableCalibration();

	cv::namedWindow("CalibResult");
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
		cv::waitKey(0);
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

	//std::cout << "Loading Game Card Data" << std::endl;
    //access->LoadCardData(errorCode, GAME_CARD_PATH);

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

    //IDAP::CardAreaDetection* testerCardArea = new IDAP::CardAreaDetection(0, 0, 0, 0, 0, 0, 0, 0, 0, 0.f, false);
    //testerCardArea->CardDetectionTester(access->GetCardData());

	while (true)
	{
		char pressed = cv::waitKey(20);
		// get next frame from camera
		access->PrepareNextFrame(errorCode);
		cv::imshow("Current", access->getSubSampledFrame());
		// for all players, check if area is active
		double isActive = 0;
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
            std::cout << "----------------\n";
		// TO-DO: CHECK IF CARD HAS CHANGED
		uint16_t cardID = 1;
        uint16_t cardType = 0;
		
	    access->IsCardChangedByID(errorCode, cardID, cardType);

        if(cardType != 0)
        {
            std::cout << "Some card was detected. Card type: " << cardType << std::endl;
        }

		// exit on q pressed
		if (pressed == 'q')
			break;
	}

	// free memory
    IDAP::DestroyImageDetectionAccessPoint(access);
	std::cout << "ALL DONE" << std::endl;

    int tj = 5;

}