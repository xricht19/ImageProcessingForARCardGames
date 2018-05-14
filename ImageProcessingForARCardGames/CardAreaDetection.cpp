#include "CardAreaDetection.h"
#include <opencv2/videostab/ring_buffer.hpp>
// defines includes
#include <opencv2\imgcodecs\imgcodecs_c.h>
#include <opencv2\imgproc\types_c.h>

using namespace cv;


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
	results = new TopThree();
}


IDAP::CardAreaDetection::~CardAreaDetection()
{
	delete(results);
}

void IDAP::CardAreaDetection::isCardChanged(uint16_t& errorCode, cv::Mat currentFrame, std::vector<std::pair<int, cv::Mat>> cardDataReference, uint16_t cardType)
{
	std::cout << "Last known card type: " << cardType << std::endl;
	std::cout << "Template count: " << cardDataReference.size() << std::endl;

	/*cv::namedWindow("Template");
	cv::namedWindow("Checking");

	cv::namedWindow("image_window", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("result_window", cv::WINDOW_AUTOSIZE);*/

	auto match_method = CV_TM_CCORR;
	results->SetMin(true);
	bool use_mask = false;
	
	// list all files in given folder and load them to memory and provide them to card area detection for template matching
	for (auto &file : std::experimental::filesystem::directory_iterator("CardDetectionData/BANG"))
	{
		cv::Mat img = cv::imread(file.path().string().c_str(), CV_LOAD_IMAGE_COLOR);

		// hought transform to turn right -------------------------------------------------
		/*cv::Mat edges, gray, dil, ero, dst, cdst, cdstP;
		cv::cvtColor(img, gray, CV_BGR2GRAY);
		cv::Canny(gray, edges, 85, 255);

		cv::cvtColor(edges, cdst, COLOR_GRAY2BGR);
		cdstP = cdst.clone();

		std::vector<Vec2f> lines; // will hold the results of the detection
		HoughLines(edges, lines, 1, CV_PI / 180, 150, 0, 0); // runs the actual detection
														   // Draw the lines
		for (size_t i = 0; i < lines.size(); i++)
		{
			float rho = lines[i][0], theta = lines[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000 * (-b));
			pt1.y = cvRound(y0 + 1000 * (a));
			pt2.x = cvRound(x0 - 1000 * (-b));
			pt2.y = cvRound(y0 - 1000 * (a));
			line(cdst, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);
		}
		// Probabilistic Line Transform
		std::vector<Vec4i> linesP; // will hold the results of the detection
		HoughLinesP(edges, linesP, 1, CV_PI / 180, 50, 50, 10); // runs the actual detection
		// Draw the lines
		for (size_t i = 0; i < linesP.size(); i++)
		{
			Vec4i l = linesP[i];
			line(cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
		}
		// Show results
		imshow("Source", img);
		imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
		imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);

		for (auto point : linesP)
		{
			std::cout << point << std::endl;
		}*/

		// perform template matching
		for (std::vector<std::pair<int, cv::Mat>>::iterator it = cardDataReference.begin(); it != cardDataReference.end(); ++it)
		{
			cv::imshow("Template", it->second);
			cv::imshow("Checking", img);

			uint16_t templCardType = static_cast<uint16_t>(it->first);
			cv::Mat rawtempl = it->second;
			cv::Mat mask;	

			/*cv::Scalar diff = cv::mean(img) - cv::mean(rawtempl);
			cv::subtract(img, diff, img);*/

			// ----------------------------------- FLANN ------------------------------------- 
			cv::Mat img_1, img_2;
			cv::cvtColor(img, img_1, CV_BGR2GRAY);
			cv::cvtColor(rawtempl, img_2, CV_BGR2GRAY);

			//-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
			int minHessian = 500;
			Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
			detector->setHessianThreshold(minHessian);
			std::vector<KeyPoint> keypoints_1, keypoints_2;
			Mat descriptors_1, descriptors_2;
			detector->detectAndCompute(img_1, Mat(), keypoints_1, descriptors_1);
			detector->detectAndCompute(img_2, Mat(), keypoints_2, descriptors_2);
			//-- Step 2: Matching descriptor vectors using FLANN matcher
			FlannBasedMatcher matcher;
			std::vector< DMatch > matches;
			matcher.match(descriptors_1, descriptors_2, matches);
			double max_dist = 0; double min_dist = 100;
			//-- Quick calculation of max and min distances between keypoints
			for (int i = 0; i < descriptors_1.rows; i++)
			{
				double dist = matches[i].distance;
				if (dist < min_dist) min_dist = dist;
				if (dist > max_dist) max_dist = dist;
			}
			printf("-- Max dist : %f \n", max_dist);
			printf("-- Min dist : %f \n", min_dist);

			//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
			//-- or a small arbitrary value ( 0.02 ) in the event that min_dist is very
			//-- small)
			//-- PS.- radiusMatch can also be used here.
			std::vector< DMatch > good_matches;
			for (int i = 0; i < descriptors_1.rows; i++)
			{
				if (matches[i].distance <= max(1.2 * min_dist, 0.02))
				{
					good_matches.push_back(matches[i]);
				}
			}

			results->TryAddNew(templCardType, min_dist);

			//-- Draw only "good" matches
			Mat img_matches;
			drawMatches(img_1, keypoints_1, img_2, keypoints_2,
				good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			//-- Show detected matches
			imshow("Good Matches", img_matches);
			for (int i = 0; i < (int)good_matches.size(); i++)
			{
				printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx);
			}
			waitKey(0);


			// ----------------------------- TEMPLATE MATCHING -------------------------------
			/*cv::Scalar diff = cv::mean(img) - cv::mean(rawtempl);
			cv::subtract(img, diff, img);

			cv::Mat templ;
			for (float i = 1; i > 0.3; i-=0.1)
			{
				// scale template				
				cv::Size newSize(rawtempl.cols*i, rawtempl.rows*i);
				//std::cout << "SIZE: " << newSize << i << std::endl;
				cv::resize(rawtempl, templ, newSize);

				cv::Mat img_display, result;
				img.copyTo(img_display);
				int result_cols = img.cols - templ.cols + 1;
				int result_rows = img.rows - templ.rows + 1;


				result.create(result_rows, result_cols, CV_64FC1);
				bool method_accepts_mask = (CV_TM_SQDIFF == match_method || match_method == CV_TM_CCORR_NORMED);
				if (use_mask && method_accepts_mask)
				{
					matchTemplate(img, templ, result, match_method, mask);
				}
				else
				{
  					matchTemplate(img, templ, result, match_method);
				}
				double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
				cv::Point matchLoc;
				cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
				if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED)
				{
					matchLoc = minLoc;
					results->TryAddNew(templCardType, minVal);
					//std::cout << "--template " << it->first << " : " << minVal << std::endl;
				}
				else
				{
					matchLoc = maxLoc;
					results->TryAddNew(templCardType, maxVal);
					//std::cout << "--template " << it->first << " : " << maxVal << std::endl;
				}
				// just for image show
				cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1);
				rectangle(img_display, matchLoc, cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), cv::Scalar::all(0), 2, 8, 0);
				rectangle(result, matchLoc, cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), cv::Scalar::all(0), 2, 8, 0);
				imshow("blurTempl", templ);
				imshow("image_window", img_display);
				imshow("result_window", result);
			}*/
		}


		std::cout << "result for " << file.path().string() << ": " << results->GetFirst() << ", " << results->GetSecond() << ", " << results->GetThird() << std::endl;
		printf("eval: %f, %f, %f\n", results->firstEval, results->secondEval, results->thirdEval);		

		results->Init();
		cv::waitKey(0);
	}


	// cut roi from frame, settle up card in it, set the right direction and perform template matching
    /*const cv::Mat area = currentFrame(roi);

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
    const cv::Size ss(CARD_MATCHING_WIDTH, CARD_MATCHING_WIDTH*ratio);
    cv::resize(area, subArea, ss);

    cv::Mat greyArea;
    cv::cvtColor(area, greyArea, CV_BGR2GRAY);


    cv::imshow("cardArea", area);
    cv::imshow("subArea", subArea);
    cv::imshow("greyArea", greyArea);

    cv::Mat edgeArea;
    cv::Canny(greyArea, edgeArea, 100, 200);

    cv::imshow("edges", edgeArea);*/


	return;
}

void IDAP::TopThree::Init()
{
	first = 0;
	second = 0;
	third = 0;
	if (min)
	{
		firstEval = std::numeric_limits<float>::max();
		secondEval = std::numeric_limits<float>::max();
		thirdEval = std::numeric_limits<float>::max();
	}
	else
	{
		firstEval = std::numeric_limits<float>::min();
		secondEval = std::numeric_limits<float>::min();
		thirdEval = std::numeric_limits<float>::min();
	}
}

void IDAP::TopThree::TryAddNew(uint16_t cardType, float eval)
{
	if (min)
	{
		if (eval < thirdEval)
		{
			// new top three
			third = cardType;
			thirdEval = eval;
			SortTopThree();
		}
	}
	else
	{
		if (eval > thirdEval) 
		{
			// new top three
			third = cardType;
			thirdEval = eval;
			SortTopThree();
		}
	}
}

void IDAP::TopThree::SortTopThree()
{
  	if (min)
	{
		if (thirdEval < secondEval)
		{
			std::swap(third, second);
			std::swap(thirdEval, secondEval);
		}
		if(secondEval < firstEval)
		{
			std::swap(first, second);
			std::swap(firstEval, secondEval);
		}
	}
	else
	{
		if (thirdEval > secondEval)
		{
			std::swap(third, second);
			std::swap(thirdEval, secondEval);
		}
		if (secondEval > firstEval)
		{
			std::swap(first, second);
			std::swap(firstEval, secondEval);
		}
	}
}
