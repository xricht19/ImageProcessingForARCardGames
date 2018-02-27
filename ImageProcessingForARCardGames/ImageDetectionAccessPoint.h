#pragma once

#include <cstdint>
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
		~CardSize() {};

		int getID() { return id; };
		int getWidth() { return width; };
		int getHeight() { return height; };
	};

	class CardPositions
	{
	private:
		int id;
		int playerID;
		int cardSizeID;
		int leftTop_x;
		int leftTop_y;
	};


	class ImageDetectionAccessPoint
	{
	private:
		cv::VideoCapture openedStream;
		cv::Mat frame;
		std::vector<PlayerAreaChangeDetector> isPlayerActiveDetectors;

		// loaded settings
		int numberOfPlayers;
		std::vector<CardSize> cardTypes;
		std::vector<CardPositions> cardPositions;

		bool usingROS = false;

		enum ErrorCodes {
			OK = 0,
			CANNOT_OPEN_VIDEO_STREAM,
			VIDEO_STREAM_IS_NOT_OPENED,
			CANNOT_GET_IMAGE_FROM_CAMERA,
			CONNECTION_TO_ROS_IS_AVAILABLE_ONLY_ON_WINDOWS,
			FRAME_WAS_NOT_READ,
		};

		void loadSettingsFromXmlAndInit(const char*);

	public:
		ImageDetectionAccessPoint();
		~ImageDetectionAccessPoint();


		void GetNumberOfAllAvailableDevices(uint16_t&, uint16_t&);

		void InitImageDetectionAccessPoint(uint16_t&, uint16_t&, const char*);
		void InitImageDetectionAccessPointROS(uint16_t&, uchar*, uint16_t&, const char*&);
		void GetVideoResolution(uint16_t&, uint16_t&, uint16_t&);

		void PrepareNextFrame(uint16_t&);
		void GetCurrentFrameData(uint16_t&, uint16_t&, uint16_t&, uint16_t&, uchar*&);
		void IsPlayerActiveByID(uint16_t&, uint16_t&);
		void HasGameObjectChanged(uint16_t&, uint16_t&, uint16_t&);
	};
}
