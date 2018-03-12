#pragma once

#include "ImageDetectionAccessPoint.h"

#define DllExport __declspec( dllexport )

namespace IDAP
{
	// wrappers functions accessible from Unity C# code
	extern "C" {
		extern DllExport ImageDetectionAccessPoint* CreateImageDetectionAccessPoint();
		extern DllExport void DestroyImageDetectionAccessPoint(ImageDetectionAccessPoint*);

		extern DllExport void GetNumberOfAllAvailableDevicesCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&);
		extern DllExport void InitImageDetectionAccessPointCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&, const char*, int);
		extern DllExport void InitImageDetectionAccessPointROSCaller(ImageDetectionAccessPoint*, uint16_t&, uchar*, uint16_t&, const char*);
		extern DllExport void GetVideoResolutionCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&, uint16_t&);

		extern DllExport void PrepareNextFrameCaller(ImageDetectionAccessPoint*, uint16_t&);
		extern DllExport void GetCurrentFrameDataCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&, uint16_t&, uint16_t&, uchar*&);
		extern DllExport void IsPlayerActiveByIDCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&, uint16_t&);
		extern DllExport void HasGameObjectChangedCaller(ImageDetectionAccessPoint*, uint16_t&, uint16_t&, uint16_t&);
	}
}