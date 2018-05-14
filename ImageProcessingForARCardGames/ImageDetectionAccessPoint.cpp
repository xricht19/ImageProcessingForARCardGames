#include "ImageDetectionAccessPoint.h"

namespace IDAP
{
	void ImageDetectionAccessPoint::LoadCardData(std::string path)
	{
		// list all files in given folder and load them to memory and provide them to card area detection for template matching
		for (auto &file : std::experimental::filesystem::directory_iterator(path))
		{
			cv::Mat img = cv::imread(file.path().string().c_str(), CV_LOAD_IMAGE_COLOR);
			// TO-DO: PERFORM SUBSAMPLING, MAYBE SAVE SUBSAMPLED (MAYBE ONLY GREYSCALE) TO NEW FOLDER AND LOOK FOR IT ON LOADING,
			//		  WE DO NOT HAVE TO LOAD LARGE IMAGES AND SUBSAMPLE THEM WHENEVER THE APP IS STARTED
			gameCardData.insert(std::make_pair(file.path().string(), img));
		}
	}

	// private function area ------------------------------------------------------------------
	// --------- SETTINGS ---------------------------------------------------------------------
	void ImageDetectionAccessPoint::loadSettingsFromXml(const char* path, int tableID)
	{
		// open and read xml file
		rapidxml::file<> xmlSettingsFile(path);
		rapidxml::xml_document<> doc;
		doc.parse<0>(xmlSettingsFile.data());

		rapidxml::xml_node<> *rootNode = doc.first_node();
		std::cout << " root Node: " << rootNode->name() << std::endl;

		// load setting of 
		rapidxml::xml_node<> *tableSettingNode = getTableSettingNodeByID(tableID, rootNode);
		if (tableSettingNode == NULL) {
			errorCode = CANNOT_LOAD_SETTINGS_FROM_XML;
			return;
		}			
		rapidxml::xml_node<> *centralAreaNode = tableSettingNode->first_node("CentralArea");
		// save info about all players for this settings -------------------------
		rapidxml::xml_node<> *playersNode = tableSettingNode->first_node("Players");
		loadNumberOfPlayers(playersNode->first_attribute("numberOf")->value());
		for (rapidxml::xml_node<> *child = playersNode->first_node(); child; child = child->next_sibling()) {
			if (child == NULL)
				break;
			PlayerInfo *newPLayerInfo = new PlayerInfo(child->first_attribute("id")->value(),
				child->first_node("ActiveArea")->first_attribute("start_x")->value(),
				child->first_node("ActiveArea")->first_attribute("start_y")->value(),
				child->first_node("ActiveArea")->first_attribute("width")->value(),
				child->first_node("ActiveArea")->first_attribute("height")->value());
			playersInfo.push_back(newPLayerInfo);
		}

		// save all available positions of cards -----------------------------
		rapidxml::xml_node<> *cardsPositionsNode = tableSettingNode->first_node("CardsPosition");
		for (rapidxml::xml_node<> *child = cardsPositionsNode->first_node(); child; child = child->next_sibling()) {
			if (child == NULL)
				break;
			CardPosition *newPositionOfCard = new CardPosition(child->first_attribute("id")->value(), 
															child->first_attribute("player_id")->value(), 
															child->first_attribute("size_id")->value(),
															child->first_attribute("left_top_corner_x")->value(),
															child->first_attribute("left_top_corner_y")->value());
			cardPositions.push_back(newPositionOfCard);
		}

		// save the size of all types of cards with it's id ---------------------------------
		rapidxml::xml_node<> *gameSettingsNode = rootNode->first_node("ARBangGameSettings");
		for (rapidxml::xml_node<> *child = gameSettingsNode->first_node(); child; child = child->next_sibling()) {
			if (child == NULL) 
				break;
			CardSize *newTypeOfCard = new CardSize(child->first_attribute("id")->value(), child->first_node("width")->value(), child->first_node("height")->value());
			cardTypes.push_back(newTypeOfCard);
		}

#ifdef _DEBUG
		std::cout << "NoP: " << numberOfPlayers << std::endl;

		for each (auto playerInfo in playersInfo)
		{
			std::cout << "PID: " << playerInfo->getID() << "| X: " << playerInfo->getAreaX() << "| Y: " << playerInfo->getAreaY() << "| W: " << playerInfo->getAreaWidth() << "| H: " << playerInfo->getAreaHeight() << std::endl;
		}

		for each (auto cardPos in cardPositions)
		{
			std::cout << "CPosID: " << cardPos->getID() << "| P_ID: " << cardPos->getPlayerID() << "| S_ID: " << cardPos->getCardSizeID() << "| X: " << cardPos->getLeftTopX() << "| Y: " << cardPos->getLeftTopY() << std::endl;
		}

		for each (auto card in cardTypes)
		{
			std::cout << "CID: " << card->getID() << "| H: " << card->getHeight() << "| W: " << card->getWidth() << std::endl;
		}
#endif
	}

	rapidxml::xml_node<>* ImageDetectionAccessPoint::getTableSettingNodeByID(int ID, rapidxml::xml_node<>* rootNode)
	{
		std::stringstream ss;
		int nodeID = -1;
		for (rapidxml::xml_node<> *child = rootNode->first_node("ARBangTableSettings"); child; child = child->next_sibling("ARBangTableSettings")) {
			if (child == NULL)
				break;
			// char * to integer
			ss << child->first_attribute("id")->value();
			ss >> nodeID;
			if (nodeID == ID)
				return child;

			ss.clear();
		}

		return NULL;
	}

	void ImageDetectionAccessPoint::loadNumberOfPlayers(char *_number)
	{
		std::stringstream ss;
		ss << _number;
		ss >> numberOfPlayers;
	}

	// --------- ACTIVE AREA DETECTORS --------------------------------------------------------
	void ImageDetectionAccessPoint::initPlayerActiveAreaDetectors()
	{
		// go over all player and create detector for their area; 
		for (std::vector<PlayerInfo*>::iterator it = playersInfo.begin(); it != playersInfo.end(); ++it)
		{
            
            // create new PlayerAreaActiveDetector
            PlayerInfo* info = *it;
            PlayerAreaActiveDetector* newDetector = new PlayerAreaActiveDetector(info->getID(),
                info->getAreaX(), info->getAreaY(), info->getAreaWidth(), info->getAreaHeight());

            isPlayerActiveDetectors.insert(std::pair<int, PlayerAreaActiveDetector*>(info->getID(), newDetector));
            
		}
	}

	void ImageDetectionAccessPoint::initCardAreaDetectors()
	{
		// go over all card position and check if changed
		for (std::vector<CardPosition*>::iterator it = cardPositions.begin(); it != cardPositions.end(); ++it)
		{
            if (GetTableCalibration()->IsCalibrationDone())
            {
			    CardPosition* cPos = *it;
			    CardSize* cSize = getCardSizeByID(cPos->getCardSizeID());
			    CardAreaDetection* newDetector = new CardAreaDetection(cPos->getID(), cPos->getPlayerID(), cPos->getCardSizeID(),
			                                                           cPos->getLeftTopX(), cPos->getLeftTopY(), cSize->getWidth(), cSize->getHeight(),
                                                                       frame.cols, frame.rows, GetTableCalibration()->GetTableCalibrationResult()->mmInPixels);

			    cardAreaDetectors.insert(std::pair<int, CardAreaDetection*>(cPos->getID(), newDetector));
            }
            else
            {
                fprintf(stderr, "initCardAreaDetectors -> The table calibration must be finished apriori!\n");
            }
		}
	}

	CardSize* ImageDetectionAccessPoint::getCardSizeByID(int id)
	{
		for (auto &item : cardTypes)
		{
			if (item->getID() == id)
				return item;
		}
		return nullptr;
	}


	uint16_t ImageDetectionAccessPoint::GetNumberOfPlayers()
	{
		return numberOfPlayers;
	}

	cv::Mat ImageDetectionAccessPoint::getFrame()
	{
		return frame;
	}

	cv::Mat ImageDetectionAccessPoint::getSubSampledFrame()
	{
		return subSampledFrame;
	}

	void ImageDetectionAccessPoint::freeSettings()
	{
		// free std::vector<CardSize*> cardTypes;
		if (!cardTypes.empty()) { 
			for (const auto &cardType : cardTypes) { 
				delete(cardType); 
			}
			cardTypes.clear();
		}
		// free std::vector<CardPosition*> cardPositions;
		if (!cardPositions.empty()) {
			for (const auto &cardPos : cardPositions) {
				delete(cardPos);
			}
			cardPositions.clear();
		}
		// free std::vector<PlayerInfo*> playersInfo;
		if (!playersInfo.empty()) {
			for (const auto &playerInfo : playersInfo) {
				delete(playerInfo);
			}
			playersInfo.clear();
		}
		// free std::map<int, PlayerAreaActiveDetector> isPlayerActiveDetectors;
		for (std::map<int, PlayerAreaActiveDetector*>::iterator it = isPlayerActiveDetectors.begin(); it != isPlayerActiveDetectors.end(); it++) {
			if (it->second != NULL) {
				delete(it->second);
				it->second = NULL;
			}
		}
		// free std::map<int, CardAreaDetection*> cardAreaDetectors;
		for (std::map<int, CardAreaDetection*>::iterator it = cardAreaDetectors.begin(); it != cardAreaDetectors.end(); it++) {
			if (it->second != NULL) {
				delete(it->second);
				it->second = NULL;
			}
		}
	}

    CameraCalibration* ImageDetectionAccessPoint::GetCameraCalibration()
    {
        // prepare structure for calibration
        if (_cameraCalib == nullptr)
        {
            _cameraCalib = new CameraCalibration();
        }
        return _cameraCalib;
    }

    TableCalibration* ImageDetectionAccessPoint::GetTableCalibration()
    {
        // prepare structure for calibration
        if (_tableCalib == nullptr)
        {
            _tableCalib = new TableCalibration();
        }
        return _tableCalib;
    }

	ProjectorCalibration * ImageDetectionAccessPoint::GetProjectorCalibration()
	{
		// prepare structure for calibration
		if (_projectorCalib == nullptr)
		{
			_projectorCalib = new ProjectorCalibration();
		}
		return _projectorCalib;
	}

    // public function area -------------------------------------------------------------------
	ImageDetectionAccessPoint::ImageDetectionAccessPoint()
	{
        _cameraCalib = nullptr;
        _tableCalib = nullptr;
		errorCode = OK;
        _flipHorizontally = false;
        _flipVertically = false;
	}


	ImageDetectionAccessPoint::~ImageDetectionAccessPoint()
	{
        if(_cameraCalib != nullptr)
            delete(_cameraCalib);
        if (_tableCalib != nullptr)
            delete(_tableCalib);

		freeSettings();
	}

	void ImageDetectionAccessPoint::GetVideoResolution(uint16_t& errorCode, uint16_t& width, uint16_t& height)
	{
		if (!openedStream.isOpened()) {
			errorCode = ErrorCodes::VIDEO_STREAM_IS_NOT_OPENED;
			width = 0;
			height = 0;
		}
		else {
			height = (uint16_t)openedStream.get(cv::CAP_PROP_FRAME_HEIGHT);
			width = (uint16_t)openedStream.get(cv::CAP_PROP_FRAME_WIDTH);
		}


	}

	void ImageDetectionAccessPoint::PrepareNextFrame(uint16_t &errorCode)
	{
		if (!openedStream.isOpened()) {
			errorCode = ErrorCodes::VIDEO_STREAM_IS_NOT_OPENED;
		}
		else {
			// get next frame
            cv::Mat rawFrame, flippedFrame;
			openedStream >> rawFrame;
			if (rawFrame.empty()) {
				errorCode = ErrorCodes::CANNOT_GET_IMAGE_FROM_CAMERA;
			}
            // flip image if requested
            if (_flipVertically && _flipHorizontally)
                cv::flip(rawFrame, flippedFrame, -1);
            else if (_flipVertically)
                cv::flip(rawFrame, flippedFrame, 1);
            else if (_flipHorizontally)
                cv::flip(rawFrame, flippedFrame, 0);
            else
                flippedFrame = rawFrame.clone();

            // if the camera is calibrated, apply to the frame
            if (GetCameraCalibration()->IsCalibrationDone())
            {
                cv::undistort(flippedFrame, frame, GetCameraCalibration()->GetCameraMatrix(), GetCameraCalibration()->GetDistanceCoeff());
            }
            // apply table calibration on frame if available
            if (GetTableCalibration()->IsCalibrationDone())
            {
                GetTableCalibration()->ApplyTableCalibrationMatrixOnInput(frame);
            }

			// sub sample frame for faster processing
			cv::Size s100x100(100, 100);
			cv::resize(frame, subSampledFrame, s100x100);
		}
	}

	void ImageDetectionAccessPoint::GetCurrentFrameSize(uint16_t &errorCode, uint16_t &rows, uint16_t &columns, uint16_t &channels)
	{
		if (!openedStream.isOpened() && !usingROS) {
			errorCode = ErrorCodes::VIDEO_STREAM_IS_NOT_OPENED;
		}
		else if (frame.empty()) {
			errorCode = ErrorCodes::FRAME_WAS_NOT_READ;
		}
		else {
			rows = (uint16_t)frame.rows;
			columns = (uint16_t)frame.cols;
			channels = (uint16_t)frame.channels();
		}
	}

	void ImageDetectionAccessPoint::GetCurrentFrameData(uint16_t &errorCode, uint16_t &rows, uint16_t &columns, uint16_t &channels, uchar* dataBytes)
	{
		if (!openedStream.isOpened() && !usingROS) {
			errorCode = ErrorCodes::VIDEO_STREAM_IS_NOT_OPENED;
		}
		else if (frame.empty()) {
			errorCode = ErrorCodes::FRAME_WAS_NOT_READ;
		}
		else {
			rows = static_cast<uint16_t>(frame.rows);
			columns = static_cast<uint16_t>(frame.cols);
			channels = static_cast<uint16_t>(frame.channels());
			//unsigned char *buffer = new unsigned char[rows*columns*channels];
			//std::memcpy(dataBytes, frame.data, rows*columns*channels);
			//dataBytes = buffer;
            //Convert from BGR to RGBA
            cv::Mat rotImg, rgbRotImg;
            /*const cv::Mat rot = cv::getRotationMatrix2D(cv::Point2f(static_cast<float>(frame.cols/2), static_cast<float>(frame.rows/2)), 180, 1);
            cv::warpAffine(frame, rotImg, rot, cv::Size(frame.cols, frame.rows));*/
            cv::cvtColor(frame, rgbRotImg, CV_BGR2RGBA);

            std::memcpy(dataBytes, rgbRotImg.data, rgbRotImg.total() * rgbRotImg.elemSize());
		}
	}

	void ImageDetectionAccessPoint::IsPlayerActiveByID(uint16_t &errorCode, uint16_t &ID, double& intensity)
	{
        intensity = isPlayerActiveDetectors[ID]->isAreaActive(subSampledFrame);
        errorCode = 0;
	}

	/**
	 * \brief Go through all card positions and check if change occure.
	 * @param cardID Sent by reference, contain the card id found in roi for the given ID.
	 */
	void ImageDetectionAccessPoint::IsCardChangedByID(uint16_t& errorCode, uint16_t&ID, uint16_t&cardID)
	{
		cardID = cardAreaDetectors[ID]->isCardChanged(frame);
	}

	std::map<std::string, cv::Mat>* ImageDetectionAccessPoint::GetGameCardData()
	{
		return &gameCardData;
	}

	uint16_t ImageDetectionAccessPoint::GetNumberOfCardAreas()
	{
		return static_cast<uint16_t>(cardPositions.size());
	}


	void ImageDetectionAccessPoint::GetNumberOfAllAvailableDevices(uint16_t &errorCode, uint16_t &numOfDevices)
	{
		numOfDevices = 0;
		// try open all video stream, until not available
		uint16_t deviceID = 0;
		while (true) {
            cv::VideoCapture testIfOpenPoss = cv::VideoCapture(deviceID);
			if (!testIfOpenPoss.isOpened()) { // no more devices
				break;
			}
			numOfDevices++;
			deviceID++;
		}
	}

    void ImageDetectionAccessPoint::IDAPPrintError(uint16_t errorCode, std::string data)
    {
        ErrorCodes code = static_cast<ErrorCodes>(errorCode);
        switch (code)
        {
        case OK:
            break;
        case CANNOT_OPEN_VIDEO_STREAM:
            fprintf(stderr, "Cannot open video stream for input: %s", data.c_str());
            break;
        default:
            fprintf(stderr, "Unknown error: %s", data.c_str());
            break;
        }
    }

    void ImageDetectionAccessPoint::InitImageDetectionAccessPointCamera(uint16_t& errorCode, uint16_t& cameraId)
	{
		this->usingROS = false;

        if (openedStream.isOpened())
            openedStream.release();

		openedStream = cv::VideoCapture(cameraId);
        if (!openedStream.isOpened()) {
            errorCode = ErrorCodes::CANNOT_OPEN_VIDEO_STREAM;
            return;
        }

        // prepare structure for table calibration
        //_tableCalib = new TableCalibration();
	}

    void ImageDetectionAccessPoint::InitImageDetectionAccessPointDataAndDetection(uint16_t& errorCode, int tableID)
    {
        // TODO: load settings from xml generated by Unity
        this->loadSettingsFromXml(GAME_DATA_PATH, tableID);
        // prepare active area detector for every player
        initPlayerActiveAreaDetectors();
        // prepare card change detectors
        initCardAreaDetectors();
    }

    void ImageDetectionAccessPoint::InitImageDetectionAccessPointROS(uint16_t &errorCode, uchar *ipAdress, uint16_t &port, const char* &settingsPath)
	{
#ifdef _WIN32
		this->usingROS = true;
#else
		errorCode = ErrorCodes::CONNECTION_TO_ROS_IS_AVAILABLE_ONLY_ON_WINDOWS;
#endif
	}


// -------- CARD --------------------------------------------
	CardSize::CardSize(char * _id, char * _width, char * _height)
	{
		// convert to int
		std::stringstream strValue;
		strValue << _id;
		strValue >> id;

		strValue.clear();
		strValue << _width;
		strValue >> width;
		
		strValue.clear();
		strValue << _height;
		strValue >> height;
	}

// ------- CARD POSITION ------------------------------------
	CardPosition::CardPosition(char *_id, char* _playerID, char* _cardSizeID, char* _leftTop_x, char* _leftTop_y)
	{
		// convert to int
		std::stringstream ss;
		ss << _id;
		ss >> id;

		ss.clear();
		ss << _playerID;
		ss >> playerID;

		ss.clear();
		ss << _cardSizeID;
		ss >> cardSizeID;

		ss.clear();
		ss << _leftTop_x;
		ss >> leftTop_x;

		ss.clear();
		ss << _leftTop_y;
		ss >> leftTop_y;
	}

// ----------- PLAYER INFO ----------------------------------
	PlayerInfo::PlayerInfo(char * _id, char * _areaX, char * _areaY, char * _areaWidth, char * _areaHeight)
	{
		std::stringstream ss;
		ss << _id;
		ss >> id;

		ss.clear();
		ss << _areaX;
		ss >> areaX;

		ss.clear();
		ss << _areaY;
		ss >> areaY;

		ss.clear();
		ss << _areaWidth;
		ss >> areaWidth;

		ss.clear();
		ss << _areaHeight;
		ss >> areaHeight;
	}
}