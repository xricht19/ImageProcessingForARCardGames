#include "CardAreaDetection.h"
#include <opencv2/videostab/ring_buffer.hpp>

int i = 0;
IDAP::CardAreaDetection::CardAreaDetection(int _id, int _playerID, int _sizeID, int _xPos, int _yPos, int _width, int _height, int imageWidth, int imageHeight, float mmInPixel)
{
	id = _id;
	playerID = _playerID;
	sizeID = _sizeID;
	posX = _xPos;
	posY = _yPos;
	
    // to over come the inaccuracy of calibration, the roi is 50 % bigger
    const float deltaWidth = (_width * mmInPixel) * 0.50;
    const float deltaHeight = (_height * mmInPixel) * 0.50;

    const float newPosX = (posX * (imageWidth / 100.f)) - deltaWidth / 2.f;
    const float newPosY = (posY * (imageHeight / 100.f)) - deltaHeight / 2.f;


	roi = cv::Rect(newPosX, newPosY, (_width * mmInPixel) + deltaWidth, (_height*mmInPixel) + deltaHeight);
	
	initState = true;
}


IDAP::CardAreaDetection::~CardAreaDetection()
{
}

uint16_t IDAP::CardAreaDetection::isCardChanged(cv::Mat currentFrame)
{
	// cut roi from frame, settle up card in it, set the right direction and perform template matching
    const cv::Mat area = currentFrame(roi);

    std::stringstream nameSS;
    if (i < 10)
        nameSS << "Other_0" << ++i;
    else
        nameSS << "Other_" << ++i;

    nameSS << ".png";

    std::string name = nameSS.str();

    cv::imwrite(name.c_str(), area);

    cv::Mat subArea;
    const float ratio = static_cast<float>(area.rows) / static_cast<float>(area.cols);
    const cv::Size ss(60,60*ratio);
    cv::resize(area, subArea, ss);

    cv::Mat greyArea;
    cv::cvtColor(area, greyArea, CV_BGR2GRAY);


    cv::imshow("cardArea", area);
    cv::imshow("subArea", subArea);
    cv::imshow("greyArea", greyArea);

    cv::Mat edgeArea;
    cv::Canny(greyArea, edgeArea, 100, 200);

    cv::imshow("edges", edgeArea);


	return 1;
}

