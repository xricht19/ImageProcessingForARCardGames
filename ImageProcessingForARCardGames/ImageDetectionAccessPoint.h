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

	public:
		CardPosition(char *_id, char* _playerID, char* _cardSizeID, char* _leftTop_x, char* _leftTop_y);
		~CardPosition() {}

		int getID() { return id; }
		int getPlayerID() { return playerID; }
		int getCardSizeID() { return cardSizeID; }
		int getLeftTopX() { return leftTop_x; }
		int getLeftTopY() { return leftTop_y; }
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

		static void GetNumberOfAllAvailableDevices(uint16_t&, uint16_t&);
        static void IDAPPrintError(uint16_t errorCode, std::string data = "");

		void InitImageDetectionAccessPointCamera(uint16_t&, uint16_t&);
        void InitImageDetectionAccessPointDataAndDetection(uint16_t&, int);
		void InitImageDetectionAccessPointROS(uint16_t&, uchar*, uint16_t&, const char*&);
		void GetVideoResolution(uint16_t&, uint16_t&, uint16_t&);
		// load cards data
		void LoadCardData(uint16_t& errorCode, std::string);

		void PrepareNextFrame(uint16_t&);
		void GetCurrentFrameSize(uint16_t&, uint16_t&, uint16_t&, uint16_t&);
		void GetCurrentFrameData(uint16_t&, uint16_t&, uint16_t&, uint16_t&, uchar*);
		void IsPlayerActiveByID(uint16_t&, uint16_t&, double&);
		void IsCardChangedByID(uint16_t& errorCode, uint16_t& cardID, uint16_t& cardType);

		uint16_t GetNumberOfCardAreas();
		uint16_t GetNumberOfPlayers();
		std::vector<std::pair<int, cv::Mat>>& GetCardData() { return cardData; }

		int errorCode;

        void SetFlipHorizontally() { _flipHorizontally ? _flipHorizontally = false : _flipHorizontally = true; }
        void SetFlipVertically() { _flipVertically ? _flipVertically = false : _flipVertically = true; }
		
		cv::Mat getFrame();
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
