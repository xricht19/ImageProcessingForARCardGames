#include "CardAreaDetection.h"
#include <opencv2/videostab/ring_buffer.hpp>
// defines includes
#include <opencv2\imgcodecs\imgcodecs_c.h>
#include <opencv2\imgproc\types_c.h>

using namespace cv;

IDAP::CardAreaDetection::CardAreaDetection(int _id, int _playerID, int _sizeID, int _xPos, int _yPos, int _width, int _height, int imageWidth, int imageHeight, float mmInPixel, bool turn)
{
    id = _id;
    playerID = _playerID;
    sizeID = _sizeID;
    posX = _xPos;
    posY = _yPos;
    turned = turn;

    // to over come the inaccuracy of calibration, the roi is 40 % bigger
    const float bigger = 0.4;
    const float deltaWidth = (_width / mmInPixel) * bigger;
    const float deltaHeight = (_height / mmInPixel) * bigger;


    const float newPosX = (posX * (imageWidth / 100.f)) - deltaWidth / 2.f;
    const float newPosY = (posY * (imageHeight / 100.f)) - deltaHeight / 2.f;

    if (!turned)
    {
        roi = cv::Rect(newPosX, newPosY, (_width / mmInPixel) + deltaWidth, (_height / mmInPixel) + deltaHeight);
        const float ratio = static_cast<float>(roi.height) / static_cast<float>(roi.width);
        sizeTM = cv::Size(CARD_MATCHING_WIDTH*(1.f + bigger), CARD_MATCHING_WIDTH*(1.f + bigger)*ratio);
    }
    else
    {
        roi = cv::Rect(newPosX, newPosY, (_height / mmInPixel) + deltaHeight, (_width / mmInPixel) + deltaWidth);
        const float ratio = static_cast<float>(roi.width) / static_cast<float>(roi.height);
        sizeTM = cv::Size(CARD_MATCHING_WIDTH*(1.f + bigger)*ratio, CARD_MATCHING_WIDTH*(1.f + bigger));        
    }
	
	initState = true;
	results = new TopThree();
}


IDAP::CardAreaDetection::~CardAreaDetection()
{
	delete(results);
}

void IDAP::CardAreaDetection::isCardChanged(uint16_t& errorCode, cv::Mat currentFrame, std::vector<std::pair<int, cv::Mat>>& cardDataReference, std::vector<std::pair<int, cv::Mat>>& cardDataGradientReference, cv::Mat meanCardGrad, uint16_t& cardType) const
{   
	// cut roi from frame, settle up card in it, set the right direction and perform template matching
    const cv::Mat area = currentFrame(roi);
    // grayscale
    cv::Mat gray;
    cv::cvtColor(area, gray, cv::COLOR_RGB2GRAY);
    // resize area
    cv::Mat areaTM;
    cv::resize(gray, areaTM, sizeTM);
    // turn area if needed
    if(turned)
    {
        cv::rotate(areaTM, areaTM, ROTATE_90_COUNTERCLOCKWISE);
    }    
    // create gradient version
    const int scale = 1;
    const int delta = 0;
    const int ddepth = CV_16S;
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;
    cv::Mat grad;
    /// Gradient X
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    Sobel(areaTM, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    convertScaleAbs(grad_x, abs_grad_x);
    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel(areaTM, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    convertScaleAbs(grad_y, abs_grad_y);
    /// Total Gradient (approximate)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    //for (std::vector<std::pair<int, cv::Mat>>::iterator it = cardDataGradientReference.begin(); it != cardDataGradientReference.end(); ++it)
    //{
    cv::Mat img_display, result;
    area.copyTo(img_display);
    const int result_cols = areaTM.cols - meanCardGrad.cols + 1;
    const int result_rows = areaTM.rows - meanCardGrad.rows + 1;

    //cv::imshow("img", grad);
    //cv::imshow("templ", meanCardGrad);

    result.create(result_rows, result_cols, CV_32FC1);
    cv::matchTemplate(grad, meanCardGrad, result, CV_TM_CCORR_NORMED);

    double minVal; double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
    //const cv::Point matchLoc = maxLoc;

    const float meanValue = cv::mean(grad)[0];
    //}

    // get bounding box around biggest contour to quickly determine if there is any card
    /*cv::Mat gray, blurr, thresh;
    cv::cvtColor(area, gray, CV_BGR2GRAY);
    GaussianBlur(gray, blurr, Size(3, 3), 1000);

    adaptiveThreshold(blurr, thresh, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, 11, 2);
    
    // get contours
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    cv::findContours(thresh, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    int largest_area = 0;
    int largest_contour_index = 0;
    Rect bounding_rect;

    for (int i = 0; i < contours.size(); i++) // iterate through each contour. 
    {
        const Rect curBounding_rect = boundingRect(contours[i]);
        const int currentArea = curBounding_rect.size().width*curBounding_rect.size().height;
        if (currentArea > largest_area) {
            largest_area = currentArea;
            largest_contour_index = i;                //Store the index of largest contour
            bounding_rect = boundingRect(contours[i]);
        }
    }
    /// Draw contours
    Mat drawing = Mat::zeros(thresh.size(), CV_8UC3);
    const Scalar color(0, 0, 255);
    drawContours(drawing, contours, largest_contour_index, color, 2, 8, hierarchy, 0, Point());
    rectangle(drawing, bounding_rect, Scalar(0, 255, 0));

    /// Show in a window
    //namedWindow("Contours", CV_WINDOW_AUTOSIZE);
    //imshow("Contours", drawing);
    //imshow("thresh", thresh);

    const float contourRatio = static_cast<float>(bounding_rect.size().width*bounding_rect.size().height) / static_cast<float>(roi.size().width*roi.size().height);*/
    if(meanValue >= MIN_MEAN_VALUE && maxVal >= TEMPLATE_MATCH_SCORE_MIN)
    {
        //std::cout << "Card, classifing" << std::endl;
        /*if(cardDataReference.size() > 0)
        {
            imshow("result", gray);
        }*/       
        
        // THINK ABOUT: may cut by bounding box of biggest contour

        // classify using SURF and FLANN
        results->SetMin(true);
        for (std::vector<std::pair<int, cv::Mat>>::iterator it = cardDataReference.begin(); it != cardDataReference.end(); ++it)
        {
            // ----------------------------------- FLANN ------------------------------------- 
            const uint16_t templCardType = static_cast<uint16_t>(it->first);
            cv::Mat img_1, img_2;
            area.copyTo(img_1);
            it->second.copyTo(img_2);

            //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
            const int minHessian = 500;
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
            std::map<float, int> goodMatchesDistance;
            //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
            //-- or a small arbitrary value ( 0.02 ) in the event that min_dist is very
            //-- small)
            //-- PS.- radiusMatch can also be used here.
            std::vector< DMatch > good_matches;
            for (int i = 0; i < descriptors_1.rows; i++)
            {
                if (matches[i].distance <= max(2 * min_dist, 0.02))
                {
                    good_matches.push_back(matches[i]);
                    goodMatchesDistance.insert(std::make_pair(matches[i].distance, i));
                }
            }
            if (goodMatchesDistance.size() >= 8)
            {
                float bestFive = 0.f;
                int count = 0;
                for (std::map<float, int>::iterator it = goodMatchesDistance.begin(); it != goodMatchesDistance.end(); ++it)
                {
                    if (++count > 8)
                        break;
                    bestFive += it->first;
                }
                if (results->isBetter(bestFive))
                {
                    results->TryAddNew(templCardType, bestFive);
                }
            }
        }
        // save classified card
        cardType = results->GetFirst();
        results->Init();
    }
    else
    {
        // no card
        cardType = IDAP::BangCardTypes::NONE;
    }
}

void IDAP::CardAreaDetection::CardDetectionTester(std::vector<std::pair<int, cv::Mat>> cardDataReference)
{
    std::map<int, int> classifications;
    for(int i = 1; i < 32; ++i)
    {
        classifications.insert(std::make_pair(i, 0));
    }

    const bool templateMethod = false;
    const std::string path = "CardDetectionData/BANG_A";

    const auto match_method = CV_TM_CCORR_NORMED;
    if (templateMethod)
        results->SetMin(false);
    else
        results->SetMin(true);

    // variable settings for gradient
    const int scale = 1;
    const int delta = 0;
    const int ddepth = CV_16S;
    
    // list all files in given folder and load them to memory and provide them to card area detection for template matching
    for (auto &file : std::experimental::filesystem::directory_iterator(path))
    {
    	cv::Mat img = cv::imread(file.path().string().c_str(), CV_LOAD_IMAGE_COLOR);
        std::cout << file.path().string().c_str() << std::endl;
        if (file.path().string().find("Bang_06.png") == std::string::npos)
            continue;
        // variables
        cv::Mat rotImg, templ, imgGray, templGray, imgGrayDenoise;
        cv::Mat gradTempl, gradImage;
        cv::Mat imgGradFinal;

    	// perform classification
        if(templateMethod)
        {
            //----------------------------- TEMPLATE MATCHING -------------------------------
            // hought transform to turn right -------------------------------------------------
            cv::Mat edges, gray, dil, ero, dst, cdst;
            cv::cvtColor(img, gray, CV_BGR2GRAY);
            cv::Canny(gray, edges, 85, 255);

            cv::cvtColor(edges, cdst, COLOR_GRAY2BGR);
            cv::Mat cdstP = cdst.clone();

            std::vector<Vec2f> lines; // will hold the results of the detection
            // Probabilistic Line Transform
            std::vector<Vec4i> linesP; // will hold the results of the detection
            HoughLinesP(edges, linesP, 1, CV_PI / 180, 50, 50, 10); // runs the actual detection
            // Draw the lines

            float angleFin = 0.f;
            float linesUsed = 0;
            for (size_t i = 0; i < linesP.size(); i++)
            {
                Vec4i l = linesP[i];
                line(cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);

                Point p1, p2;
                p1 = Point(l[0], l[1]);
                p2 = Point(l[2], l[3]);
                // calculate angle in radian, to degrees: angle * 180 / PI
                float angle = atan2(p1.y - p2.y, p1.x - p2.x) * 180 / CV_PI;                
                if (angle > 0)
                    angle -= 90;
                else
                    angle += 90;
                // limit the lines which will be used, to limit errors
                if ((angle < MAX_LINE_ANGLE_FOR_HOUGH && angle > 0) ||
                    (angle > -MAX_LINE_ANGLE_FOR_HOUGH && angle < 0))
                {
                    angleFin += angle;
                    ++linesUsed;
                }
            }
            if(linesUsed > 0)
                angleFin /= linesUsed;

            // rotate img
            const cv::Mat rot = getRotationMatrix2D(Point2f(img.cols / 2, img.rows / 2), angleFin, 1);
            cv::warpAffine(img, rotImg, rot, Size(img.cols, img.rows));

            cv::cvtColor(rotImg, imgGray, CV_BGR2GRAY);
            // gradient of input image
            cv::Mat imgGradX, imgGradY;
            cv::Mat absGradX, absGradY;

            /// Gradient X
            Sobel(imgGray, imgGradX, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
            convertScaleAbs(imgGradX, absGradX);
            /// Gradient Y
            //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
            Sobel(imgGray, imgGradY, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
            convertScaleAbs(imgGradY, absGradY);
            // combine gradient
            cv::addWeighted(absGradX, 0.5, absGradY, 0.5, 0, imgGradFinal);

        }

    	for (std::vector<std::pair<int, cv::Mat>>::iterator it = cardDataReference.begin(); it != cardDataReference.end(); ++it)
    	{
    		uint16_t templCardType = static_cast<uint16_t>(it->first);
    		cv::Mat rawtempl = it->second;
            cv::Mat mask;

            // ----------------------------------- FLANN ------------------------------------- 
            if (!templateMethod)
            {
                cv::Mat img_1, img_2;

                img.copyTo(img_1);
                rawtempl.copyTo(img_2);

                //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
                const int minHessian = 500;
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
                    const double dist = matches[i].distance;
                    if (dist < min_dist) min_dist = dist;
                    if (dist > max_dist) max_dist = dist;
                }
                //printf("-- Max dist : %f \n", max_dist);
                //printf("-- Min dist : %f \n", min_dist);

                std::map<float, int> goodMatchesDistance;

                //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
                //-- or a small arbitrary value ( 0.02 ) in the event that min_dist is very
                //-- small)
                //-- PS.- radiusMatch can also be used here.
                std::vector< DMatch > good_matches;
                for (int i = 0; i < descriptors_1.rows; i++)
                {
                    if (matches[i].distance <= max(2 * min_dist, 0.02))
                    {
                        good_matches.push_back(matches[i]);
                        goodMatchesDistance.insert(std::make_pair(matches[i].distance, i));
                    }
                }
                double evalNew = static_cast<double>(good_matches.size()) / static_cast<double>(matches.size());
                //printf("EVAL: %d | %d : %f\n", static_cast<int>(good_matches.size()), static_cast<int>(matches.size()), eval);

                if (goodMatchesDistance.size() >= 8)
                {
                    float bestFive = 0.f;
                    int count = 0;
                    for (std::map<float, int>::iterator it2 = goodMatchesDistance.begin(); it2 != goodMatchesDistance.end(); ++it2)
                    {
                        if (++count > 8)
                            break;
                        bestFive += it2->first;
                    }
                    //std::cout << "Chesking add: " << templCardType << ", " << bestFive << std::endl;
                    if (results->isBetter(bestFive))
                    {
                        results->TryAddNew(templCardType, bestFive);
                    }
                }
                //-- Draw only "good" matches
                Mat img_matches;
                drawMatches(img_1, keypoints_1, img_2, keypoints_2,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                //-- Show detected matches
                //imshow("Good Matches", img_matches);
                //imwrite("goodMatches.png", img_matches);
                for (int i = 0; i < (int)good_matches.size(); i++)
                {
                    //printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx);
                }
                //cv::waitKey(0);
            }
            else
            {
                cv::cvtColor(rawtempl, templGray, CV_BGR2GRAY);
                // gradient of input image
                cv::Mat templGradX, templGradY;
                cv::Mat absGradXTempl, absGradYTempl;
                cv::Mat templGradFinal;

                /// Gradient X
                Sobel(templGray, templGradX, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
                convertScaleAbs(templGradX, absGradXTempl);
                /// Gradient Y
                //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
                Sobel(templGray, templGradY, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
                convertScaleAbs(templGradY, absGradYTempl);
                // combine gradient
                cv::addWeighted(absGradXTempl, 0.5, absGradYTempl, 0.5, 0, templGradFinal);

                cv::Point matchLoc;
                /*for (float i = 1; i > 0.5; i -= 0.1)
                {
                    // scale template
                    cv::Size newSize(rawtempl.cols*i, rawtempl.rows*i);
                    cv::resize(templGradFinal, templGradFinal, newSize);*/

                cv::Mat img_display, result;
                gradImage.copyTo(img_display);
                int result_cols = imgGradFinal.cols - templGradFinal.cols + 1;
                int result_rows = imgGradFinal.rows - templGradFinal.rows + 1;


                result.create(result_rows, result_cols, CV_32FC1);
                matchTemplate(imgGradFinal, templGradFinal, result, match_method);
                double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
                cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
                matchLoc = maxLoc;
                results->TryAddNew(templCardType, maxVal);
                cv::Mat locationMatch;
                imgGradFinal.copyTo(locationMatch);
                const cv::Rect rect(matchLoc.x, matchLoc.y, templGradFinal.size().width, templGradFinal.size().height);
                cv::rectangle(locationMatch, rect, Scalar(255,0,0));

                //}
            }
    	}

    	//std::cout << "result for " << file.path().string() << ": " << results->GetFirst() << ", " << results->GetSecond() << ", " << results->GetThird() << std::endl;
    	//printf("eval: %f, %f, %f\n", results->firstEval, results->secondEval, results->thirdEval);

        classifications.at(results->GetFirst()) += 1;

    	results->Init();
    	//cv::waitKey(0);
    }

    for(int i = 1; i < 32; ++i)
    {
        if(classifications.at(i) > 0)
            std::cout << "template ID:" << i << " -> " << classifications.at(i) << std::endl;
    }
    //cv::waitKey(0);
}


void IDAP::TopThree::Init()
{
	first = 0;
	second = 0;
	third = 0;

    fiveBestPoints = std::numeric_limits<double>::max();

	if (min)
	{
		firstEval = std::numeric_limits<double>::max();
		secondEval = std::numeric_limits<double>::max();
		thirdEval = std::numeric_limits<double>::max();
	}
	else
	{
		firstEval = std::numeric_limits<double>::min();
		secondEval = std::numeric_limits<double>::min();
		thirdEval = std::numeric_limits<double>::min();
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

bool IDAP::TopThree::isBetter(float val)
{
    double curVal = static_cast<double>(val);
    if (curVal < fiveBestPoints)
    {
        fiveBestPoints = curVal;
        return true;
    }
    return false;

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
