#pragma once

#include <cstdint>
#include <vector>
#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>

class CardAreaDetection
{
public:
	CardAreaDetection();
	~CardAreaDetection();

	uint16_t isCardChanged(); // core function with template matching function

private:
	// firstly, the card in roi has to be settle up
	cv::Mat settleUpRoi();
	// load cards data
	void loadCardData(std::string); // path to folder with data required
	
	int id;
	int playerID;
	int cardID;
	bool initState;

	// the position where the card in image is
	cv::Rect roi;
	// card data for template matching
	std::vector<std::pair<int, cv::Mat>> cardData;
};

