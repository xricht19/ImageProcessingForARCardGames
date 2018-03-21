#pragma once

#include <cstdint>
#include <vector>
#include <iostream>
#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>

#define CARD_DATA_SUBSAMPLING_RATE 0.2


namespace IDAP
{
	class CardAreaDetection
	{
	public:
		CardAreaDetection(int _id, int _playerID, int _sizeID, int _xPos, int _yPos, int _width, int _height);
		~CardAreaDetection();

		uint16_t isCardChanged(cv::Mat currentFrame); // core function with template matching function

	private:
		
		// data from previous frame
		cv::Mat previousFrameData;

		int id;
		int playerID;
		int sizeID;
		int posX;
		int posY;

		int cardID;
		bool initState;

		// the position where the card in image is
		cv::Rect roi;
	};
}

