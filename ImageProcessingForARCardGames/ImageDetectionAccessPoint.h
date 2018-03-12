#pragma once

#include <cstdint>
#include <iostream>
#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>
#include <vector>

#include "PlayerAreaChangeDetector.h"

#include "rapidxml-1.13\rapidxml.hpp"
#include "rapidxml-1.13\rapidxml_utils.hpp"


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


	class ImageDetectionAccessPoint
	{
	private:
		cv::VideoCapture openedStream;
		cv::Mat frame;
		cv::Mat subSampledFrame; // frame in size 100 x 100
		std::map<int, PlayerAreaActiveDetector*> isPlayerActiveDetectors;

		// loaded settings
		int numberOfPlayers;
		std::vector<CardSize*> cardTypes;
		std::vector<CardPosition*> cardPositions;
		std::vector<PlayerInfo*> playersInfo;

		int centralAreaX;
		int centralAreaY;
		int centralAreaWidth;
		int centralAreaHeight;

		bool usingROS = false;

		enum ErrorCodes {
			OK = 0,
			CANNOT_OPEN_VIDEO_STREAM,
			VIDEO_STREAM_IS_NOT_OPENED,
			CANNOT_GET_IMAGE_FROM_CAMERA,
			CONNECTION_TO_ROS_IS_AVAILABLE_ONLY_ON_WINDOWS,
			FRAME_WAS_NOT_READ,
			CANNOT_LOAD_SETTINGS_FROM_XML,
		};
		// additional function to load settings from xml
		void loadSettingsFromXml(const char*, int);
		rapidxml::xml_node<>* getTableSettingNodeByID(int ID, rapidxml::xml_node<>* rootNode);
		void loadNumberOfPlayers(char*);
		
		// set active area detectors
		void initPlayerActiveAreaDetectors();		

	public:
		ImageDetectionAccessPoint();
		~ImageDetectionAccessPoint();

		void GetNumberOfAllAvailableDevices(uint16_t&, uint16_t&);

		void InitImageDetectionAccessPoint(uint16_t&, uint16_t&, const char*, int);
		void InitImageDetectionAccessPointROS(uint16_t&, uchar*, uint16_t&, const char*&);
		void GetVideoResolution(uint16_t&, uint16_t&, uint16_t&);

		void PrepareNextFrame(uint16_t&);
		void GetCurrentFrameData(uint16_t&, uint16_t&, uint16_t&, uint16_t&, uchar*&);
		void IsPlayerActiveByID(uint16_t&, uint16_t&, uint16_t&);
		void HasGameObjectChanged(uint16_t&, uint16_t&, uint16_t&);

		int errorCode;

		int getNumberOfPlayers();
		cv::Mat getFrame();
		cv::Mat getSubSampledFrame();

		// free everything created on heap
		void freeSettings();
	};
}