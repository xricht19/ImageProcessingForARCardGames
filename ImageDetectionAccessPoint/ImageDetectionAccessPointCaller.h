#pragma once

#include "ImageDetectionAccessPoint.h"

/***************************************************************
* Author: Jiri Richter
* In this file the wrapper around IDAP main classs is defined.
* This is required to successfully load the dll into c# project.
* Only the pointer to created instance of IDAP is needed in every 
* function.
****************************************************************/

#define DllExport __declspec( dllexport )

// TO-DO: these params should be obtained from user in Unity
#define CAMERA_CALIBRATION_FILE "IDAP_cameraCalibFile_ARBang"
#define MARKERS_SAVE_PATH "Markers"

namespace IDAP
{
	// wrappers functions accessible from Unity C# code
	extern "C" {
		extern DllExport ImageDetectionAccessPoint* CreateImageDetectionAccessPoint();
		extern DllExport void DestroyImageDetectionAccessPoint(ImageDetectionAccessPoint*);

		extern DllExport void GetNumberOfAllAvailableDevicesCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&);
		extern DllExport void InitImageDetectionAccessPointCameraCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&);
        extern DllExport void InitImageDetectionAccessPointDataAndDetectionCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& tableID);
		extern DllExport void GetVideoResolutionCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&, uint16_t&);
        extern DllExport void SetFlipHorizontallyCaller(ImageDetectionAccessPoint*, uint16_t&);
        extern DllExport void SetFlipVerticallyCaller(ImageDetectionAccessPoint*, uint16_t&);

		extern DllExport void PrepareNextFrameCaller(ImageDetectionAccessPoint*, uint16_t&);
		extern DllExport void GetCurrentFrameSizeCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&, uint16_t&, uint16_t&);
		extern DllExport void GetCurrentFrameDataCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&, uint16_t&, uint16_t&, uchar*);
		extern DllExport void IsPlayerActiveByIDCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&, double& intensity);
		extern DllExport void IsCardChangedByIDCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& positionID, uint16_t& cardID);

        // calibrate camera functions
        extern DllExport void AddImageWithChessboardCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode);
        extern DllExport void IsEnoughDataCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& isEnough);
        extern DllExport void SetSquareDimensionCameraCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& dim);  // dimension set in nm
        extern DllExport void SetChessboardDimensionCameraCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& width, uint16_t& height);
        extern DllExport void SaveCameraCalibCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode);
        extern DllExport void LoadCameraCalibCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode);  // error code hold the success if any
        extern DllExport void CalibrateCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode);
        extern DllExport void GetCalibrationCameraImageCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& imageNumber, uint16_t& width, uint16_t& height, uint16_t& channels, uchar* data);

        // table calibration functions
        extern DllExport void CreateArucoMarkersCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode);
        extern DllExport void DetectMarkersCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode);
        extern DllExport void CalculateTableCalibrationResultCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode);
        extern DllExport void SetChessboardDimensionProjectionCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& width, uint16_t& height);
        extern DllExport void GetProjectionTranformMatrixCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& dataSizeAvailable, double &cmInPixels, double* data, double* tableCorners);
	}
}