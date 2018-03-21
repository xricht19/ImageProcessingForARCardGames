#include "CardAreaDetection.h"



IDAP::CardAreaDetection::CardAreaDetection(int _id, int _playerID, int _sizeID, int _xPos, int _yPos, int _width, int _height)
{
	id = _id;
	playerID = _playerID;
	sizeID = _sizeID;
	posX = _xPos;
	posY = _yPos;
	
	// create roi for card
	roi = cv::Rect(posX, posY, _width, _height);
	
	initState = true;
}


IDAP::CardAreaDetection::~CardAreaDetection()
{
}

uint16_t IDAP::CardAreaDetection::isCardChanged(cv::Mat currentFrame)
{
	// cut roi from frame, settle up card in it, set the right direction and perform template matching
	std::cout << "Matching card." << std::endl;
	return 1;
}

