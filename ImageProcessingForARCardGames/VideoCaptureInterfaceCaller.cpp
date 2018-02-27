#include "VideoCaptureInterfaceCaller.h"

namespace IDAP
{
	// create instance of VideoCaptureInterface
	ImageDetectionAccessPoint* CreateImageDetectionAccessPoint() {
		return new ImageDetectionAccessPoint();
	}

	// destroy instance of VideoCaptureInterface
	void DestroyImageDetectionAccessPoint(ImageDetectionAccessPoint* instance) {
		if (instance != NULL) {
			cv::destroyAllWindows();
			delete instance;
			instance = NULL;
		}
	}

	// returns number of camera devices connected to computer

	void GetNumberOfAllAvailableDevicesCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& numberOfDevices) {
		instance->GetNumberOfAllAvailableDevices(errorCode, numberOfDevices);
	}

	void InitImageDetectionAccessPointCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& cameraId, const char* path) {
		instance->InitImageDetectionAccessPoint(errorCode, cameraId, path);
	}

	void InitImageDetectionAccessPointROSCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uchar* ipAdress, uint16_t& port, const char* path) {
		instance->InitImageDetectionAccessPointROS(errorCode, ipAdress, port, path);
	}

	void GetVideoResolutionCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& width, uint16_t& height) {
		instance->GetVideoResolution(errorCode, width, height);
	}

	void PrepareNextFrameCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode) {
		instance->PrepareNextFrame(errorCode);
	}

	void GetCurrentFrameDataCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& rows, uint16_t& columns, uint16_t& channels, uchar*& dataBytes) {
		instance->GetCurrentFrameData(errorCode, rows, columns, channels, dataBytes);
	}

	void IsPlayerActiveByIDCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& playerID) {
		instance->IsPlayerActiveByID(errorCode, playerID);
	}

	void HasGameObjectChangedCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& positionID, uint16_t& objectID) {
		instance->HasGameObjectChanged(errorCode, positionID, objectID);
	}
}