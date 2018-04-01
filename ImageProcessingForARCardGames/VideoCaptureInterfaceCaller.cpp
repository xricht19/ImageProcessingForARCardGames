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
		ImageDetectionAccessPoint::GetNumberOfAllAvailableDevices(errorCode, numberOfDevices);
	}

	void InitImageDetectionAccessPointCameraCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& cameraId) {
		instance->InitImageDetectionAccessPointCamera(errorCode, cameraId);
	}

    void InitImageDetectionAccessPointDataCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, const char* path, uint16_t tableID) {
        instance->InitImageDetectionAccessPointData(errorCode, path, tableID);
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

	void GetCurrentFrameSizeCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& rows, uint16_t& columns, uint16_t& channels) {
		instance->GetCurrentFrameSize(errorCode, rows, columns, channels);
	}

	void GetCurrentFrameDataCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& rows, uint16_t& columns, uint16_t& channels, uchar* dataBytes) {
		instance->GetCurrentFrameData(errorCode, rows, columns, channels, dataBytes);
	}

	void IsPlayerActiveByIDCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& playerID, uint16_t &isActive) {
		instance->IsPlayerActiveByID(errorCode, playerID, isActive);
	}

	void IsCardChangedByIDCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& positionID, uint16_t& cardID) {
		instance->IsCardChangedByID(errorCode, positionID, cardID);
	}
}