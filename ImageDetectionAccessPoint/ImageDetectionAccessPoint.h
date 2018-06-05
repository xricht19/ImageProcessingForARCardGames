#pragma once

#include <cstdint>
#include <iostream>
#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>
#include <vector>
#include <filesystem>

#include "PlayerAreaChangeDetector.h"
#include "CardAreaDetection.h"
#include "CameraCalibration.h"
#include "TableCalibration.h"
#include "ProjectorCalibration.h"

#include "rapidxml-1.13\rapidxml.hpp"
#include "rapidxml-1.13\rapidxml_utils.hpp"

/***************************************************************
* Author: Jiri Richter
* The main class of IDAP library is defined in 
* ImageDetectionAccessPoint.h. The class is control the process
* of active player detection and card classification. It constrols
* the camera in input and holds all classes needed for detection.
****************************************************************/


#define GAME_DATA_PATH "Assets/ARBang/Settings0.xml"

#define GAME_CARD_PATH "Assets/ARBang/cardData"
#define CARD_MATCHING_WIDTH 60

namespace IDAP
{
	class CardSize
	{
	private:
		int id;
		int width;
		int height;

	public:
		CardSize(char* _id, char* _width, char* _height);
		~CardSize() {}

		int getID() { return id; }
		int getWidth() { return width; }
		int getHeight() { return height; }
	};

	class CardPosition
	{
	private:
		int id;
		int playerID;
		int cardSizeID;
		int leftTop_x;
		int leftTop_y;
        bool turn_ninety;

	public:
		CardPosition(char *_id, char* _playerID, char* _cardSizeID, char* _leftTop_x, char* _leftTop_y, char* _turn_ninety);
        ~CardPosition() = default;

		int getID() { return id; }
		int getPlayerID() { return playerID; }
		int getCardSizeID() { return cardSizeID; }
		int getLeftTopX() { return leftTop_x; }
		int getLeftTopY() { return leftTop_y; }
        bool isTurnenNinety() { return turn_ninety; }
	};


	class PlayerInfo 
	{
	private:
		int id;
		// player active area 
		int areaX;
		int areaY;
		int areaWidth;
		int areaHeight;

	public:
		PlayerInfo(char* _id, char* _areaX, char* _areaY, char* areaWidth, char* areaHeight);
		~PlayerInfo() {}

		int getID() { return id; }
		int getAreaX() { return areaX; }
		int getAreaY() { return areaY; }
		int getAreaWidth() { return areaWidth; }
		int getAreaHeight() { return areaHeight; }
	};

	class CardAreaDetection;
	class ImageDetectionAccessPoint
	{
	private:
        inline static std::string cardDataNames[28] = 
        {
            "1.png", "2.png", "3.png", "4.png", "5.png", "6.png", "7.png", "8.png", "9.png", "10.png",
            "11.png", "12.png", "13.png", "14.png", "15.png", "16.png", "17.png", "18.png", "19.png", "20.png",
            "21.png", "22.png", "23.png", "24.png", "25.png", "26.png", "27.png", "28.png",
        };

		cv::VideoCapture openedStream;
		cv::Mat frame;
		cv::Mat subSampledFrame; // frame in size 100 x 100
        bool _flipVertically;
        bool _flipHorizontally;

		std::map<int, PlayerAreaActiveDetector*> isPlayerActiveDetectors;
		std::map<int, CardAreaDetection*> cardAreaDetectors;

		// loaded settings
		int numberOfPlayers;
		std::vector<CardSize*> cardTypes;
		std::vector<CardPosition*> cardPositions;
		std::vector<PlayerInfo*> playersInfo;

		bool usingROS = false;

		// card data for template matching
		std::vector<std::pair<int, cv::Mat>> cardData;
        std::vector<std::pair<int, cv::Mat>> cardDataGradient;
        cv::Mat meanCardGrad;

		// additional function to load settings from xml
		void loadSettingsFromXml(const char*, int);
		rapidxml::xml_node<>* getTableSettingNodeByID(int ID, rapidxml::xml_node<>* rootNode);
		void loadNumberOfPlayers(char*);
		
		// set active area detectors
		void initPlayerActiveAreaDetectors();		
		// set card area detectors
		void initCardAreaDetectors();

		CardSize* getCardSizeByID(int id);

        // camera calibration variables
        CameraCalibration* _cameraCalib;

        // table calibration variables
        TableCalibration* _tableCalib;

		// projector calibration
		ProjectorCalibration* _projectorCalib;

	public:
		ImageDetectionAccessPoint();
		~ImageDetectionAccessPoint();

        enum ErrorCodes {
            OK = 0,
            CANNOT_OPEN_VIDEO_STREAM,
            VIDEO_STREAM_IS_NOT_OPENED,
            CANNOT_GET_IMAGE_FROM_CAMERA,
            FRAME_WAS_NOT_READ,

            CANNOT_LOAD_SETTINGS_FROM_XML = 501,
        };

        // return number of available cameras connected to this computeer
		static void GetNumberOfAllAvailableDevices(uint16_t&, uint16_t&);
        // print an error to stderr, used for testing
        static void IDAPPrintError(uint16_t errorCode, std::string data = "");

        // initialization functions
        // The camera is inicialized and video stream opened.
		void InitImageDetectionAccessPointCamera(uint16_t&, uint16_t&);
        // Load the configuration file and create CardAreaDetectors for every card position and 
	    // PlayerAreaChange detector for every player specified in configuration file.
        void InitImageDetectionAccessPointDataAndDetection(uint16_t&, int);
        // return the width, height of frame captured by camera.
		void GetVideoResolution(uint16_t& errorCode, uint16_t& width, uint16_t& height);
		// load cards data
		void LoadCardData(uint16_t& errorCode, std::string);

        // capture next frame from camera
		void PrepareNextFrame(uint16_t&);
        // return the width, height and number of channels of frames captured by camera.
		void GetCurrentFrameSize(uint16_t& errorCode, uint16_t& width, uint16_t& height, uint16_t& channels);
        // The current frame raw data are copied to rawData variable provided as reference. The size of rawData is defined by width, height and number of channels,
        // if these do not correspond to actual size, errorCode different from zero is returned.
		void GetCurrentFrameData(uint16_t& errorCode, uint16_t& width, uint16_t& height, uint16_t& channels, uchar* rawData);
        // Function calls function IsPlayerActive of PlayerAreaChangeDetector by provided user ID.
		void IsPlayerActiveByID(uint16_t& errorCode, uint16_t& playerID, double& intensity);
        // Function calls function IsCardChanged of CardAreaDetection by provided card ID. Card type is used for results.
		void IsCardChangedByID(uint16_t& errorCode, uint16_t& cardID, uint16_t& cardType);
        // return the number of card areas defined in configuration file
		uint16_t GetNumberOfCardAreas();
        // return the number of players defined in configuration file
		uint16_t GetNumberOfPlayers();
        // return the templates of every card class in vector
		std::vector<std::pair<int, cv::Mat>>& GetCardData() { return cardData; }
        // return the gradients of templates of every card class in vector
        std::vector<std::pair<int, cv::Mat>>& GetCardDataGradient() { return cardDataGradient; }
        // return mean gradient of every template card
        cv::Mat& GetMeanCardGradient() { return meanCardGrad; }

		int errorCode;

        // if set, every captured frame is preprocessed by flipping
        void SetFlipHorizontally() { _flipHorizontally ? _flipHorizontally = false : _flipHorizontally = true; }
        void SetFlipVertically() { _flipVertically ? _flipVertically = false : _flipVertically = true; }
		
        // return current frame
		cv::Mat getFrame();
        // return subsampled frame of size 100x100
		cv::Mat getSubSampledFrame();

		// free everything created on heap
		void freeSettings();

        // camera calibration access
        CameraCalibration* GetCameraCalibration();

        // table calibration access
        TableCalibration* GetTableCalibration();

		// projection calibration access
		ProjectorCalibration* GetProjectorCalibration();
	};
}
