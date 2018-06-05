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

/***************************************************************
* Author: Jiri Richter
* The CardAreaDetection.h contains the class which is performing
* card detection and classification for card games. 
****************************************************************/

#define MIN_MEAN_VALUE 20.0 // to override to high score of template matching mean card with empty space
#define TEMPLATE_MATCH_SCORE_MIN 0.7 // minimum score the template matching method with mean card must achieve to detect card
#define MAX_LINE_ANGLE_FOR_HOUGH 90 // limit of line agles the lines from hough transform can have, sometimes the incorrect line is detected and it must not be used in calculation of mean angle

namespace IDAP
{
    static enum BangCardTypes
    {
        NONE = 0,
    };


    /**
	 * \brief Class which holds the best results during the card classification process, 
	 * after matching with all templates is finished, the instance of class holds the 
	 * three best matches.
	 */
	class TopThree
	{
	public:
		TopThree()
		{
			min = false;
			Init();
		};
		void Init();
	    /**
		 * \brief The function enables to set the state when minimum is considered as best result.
		 * Default state is the bigger the evaluation, better the match.
		 * \param val True, if the smaller is consider better. False, otherwise.
		 */
		void SetMin(bool val)
		{
			min = val;
			Init();
		}
	    /**
		 * \brief Function add new match if its better than the ones already contained in instance.
		 * \param cardType Class of newly detected card.
		 * \param eval The score evaluation of card and template similarity.
		 */
		void TryAddNew(uint16_t cardType, float eval);

	    /**
         * \brief Return true if value is better than current bestPoints value. Used for Nearest neighbor search.
         * Meaning of etter depends on the setting of class - value of min.
         * \param val Value which has to be compared with bestPoints value.
         * \return 
         */
        bool isBetter(float val);

        // getters
		uint16_t GetFirst() const { return first; }
		uint16_t GetSecond() const { return second; }
		uint16_t GetThird() const { return third; }

	private:
	    /**
		 * \brief When new type of class is given to top three instance of class,
		 * it need to be sorted again.
		 */
		void SortTopThree();

		bool min;
		uint16_t first;		
		uint16_t second;		
		uint16_t third;

	public:
		double firstEval;
		double secondEval;
		double thirdEval;
        double bestPoints;
	};

    /**
	 * \brief The class is providing functions to detect and classify card objects in image from camera.
	 */
	class CardAreaDetection
	{
	public:
	    /**
		 * \brief CardAreaDetection class constructor.
		 * \param _id Position ID of card in image.
		 * \param _playerID ID of player to which the card is associated.
		 * \param _sizeID ID of size of card from configuration file.
		 * \param _xPos Position of card in image in X-axis.
		 * \param _yPos Position of card in image in Y-axis.
		 * \param _width Width of card in configuration file.
		 * \param _height Height of card in configuration file. 
		 * \param imageWidth Width of frame on which the detection and classification will be performed.
		 * \param imageHeight Height of frame on which the detection and classification will be performed.
		 * \param mmInPixel The value used to compute the pixel size of card from millimeter defined in configuration file.
		 * \param turn Determine if the card is turn by 90 degrees in input image.
		 */
		CardAreaDetection(int _id, int _playerID, int _sizeID, int _xPos, int _yPos, int _width, int _height, int imageWidth, int imageHeight, float mmInPixel, bool turn);
		~CardAreaDetection();

	    /**
		 * \brief Function perform the card detection and classification.
		 * \param errorCode Reference to uint16 value in which the error code is returned, in case error occur.
		 * \param currentFrame The frame from camera, in which the detection and classification need to be performed.
		 * \param cardDataReference Templates of all card classes the system is able to recongized.
		 * \param cardDataGradientReference Gradient of templates of all card classes the system is able to recognize.
		 * \param meanCardGrad Gradient of mean card from all templates.
		 * \param cardType Reference to uint16 in which the id of class to which the card was classified is returned.
		 */
		void isCardChanged(uint16_t& errorCode, cv::Mat currentFrame, std::vector<std::pair<int, cv::Mat>>& cardDataReference, std::vector<std::pair<int, cv::Mat>>& cardDataGradientReference, cv::Mat meanCardGrad, uint16_t& cardType) const; // core function with template matching function

	    /**
         * \brief Auxilary function to evaluate method Template matching and SURF with FLANN in their ability to classify card correctly.
         * \param cardDataReference Templates of all card classes the system is able to recongized.
         */
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

		// the position where the card in image is
		cv::Rect roi;
	};
}

