#pragma once

#include <cstdint>
#include <vector>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <utility>

#include "ImageDetectionAccessPoint.h"

#define MIN_MEAN_VALUE 20.0 // to override to high score of template matching mean card with empty space
#define TEMPLATE_MATCH_SCORE_MIN 0.7
#define MAX_LINE_ANGLE_FOR_HOUGH 90

namespace IDAP
{
    static enum BangCardTypes
    {
        NONE = 0,
    };


	struct TopThree
	{
	public:
		TopThree()
		{
			min = false;
			Init();
		};
		void Init();
		void SetMin(bool val)
		{
			min = val;
			Init();
		}
		void TryAddNew(uint16_t cardType, float eval);

        bool isBetter(float val);

		uint16_t GetFirst() { return first; }
		uint16_t GetSecond() { return second; }
		uint16_t GetThird() { return third; }

	private:
		void SortTopThree();

		bool min;

		uint16_t first;		
		uint16_t second;		
		uint16_t third;
	public:
		double firstEval;
		double secondEval;
		double thirdEval;

        double fiveBestPoints;
	};

	class CardAreaDetection
	{
	public:
		CardAreaDetection(int _id, int _playerID, int _sizeID, int _xPos, int _yPos, int _width, int _height, int imageWidth, int imageHeight, float mmInPixel, bool turn);
		~CardAreaDetection();

		void isCardChanged(uint16_t& errorCode, cv::Mat currentFrame, std::vector<std::pair<int, cv::Mat>>& cardDataReference, std::vector<std::pair<int, cv::Mat>>& cardDataGradientReference, cv::Mat meanCardGrad, uint16_t& cardType) const; // core function with template matching function

        void CardDetectionTester(std::vector<std::pair<int, cv::Mat>> cardDataReference);

	private:
		
		// data from previous frame
		cv::Mat previousFrameData;

		TopThree* results;

		int id;
		int playerID;
		int sizeID;
		int posX;
		int posY;

		int cardID;
		bool initState;

        bool turned;
        cv::Mat rot;
        cv::Size sizeTM;

	    // avarage pixel value for background
        float backGroundMean;
        cv::Mat background;

		// the position where the card in image is
		cv::Rect roi;
	};
}

