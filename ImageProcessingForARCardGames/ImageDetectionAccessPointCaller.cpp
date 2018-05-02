#include "ImageDetectionAccessPointCaller.h"

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

    void SetFlipHorizontallyCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode)
    {
        instance->SetFlipHorizontally();
        errorCode = 0;
    }

    void SetFlipVerticallyCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode)
    {
        instance->SetFlipVertically();
        errorCode = 0;
    }

    // ------------------------------- CAMERA CALIBRATION -----------------------------------------------------------
    void AddImageWithChessboardCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode)
    {
        instance->GetCameraCalibration()->AddImageWithChessboard(instance->getFrame());
        errorCode = 0;
    }

    void IsEnoughDataCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& isEnough)
    {
        instance->GetCameraCalibration()->IsEnoughData(isEnough);
        if (isEnough > 0)
            errorCode = 0;
    }

    void SetSquareDimensionCameraCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& dim)
    {
        // dim in nm, to cover more precision in uint16_t, need to convert to mm
        instance->GetCameraCalibration()->SetSquareDimension(static_cast<float>(dim) / 1000.f);
        errorCode = 0;
    }

    void SetChessboardDimensionCameraCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode, uint16_t& width,
        uint16_t &height)
    {
        instance->GetCameraCalibration()->SetChessboardDimension(static_cast<int>(width), static_cast<int>(height));
        errorCode = 0;
    }

    void SaveCameraCalibCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode)
    {
        const bool ret = instance->GetCameraCalibration()->SaveCameraCalib(CAMERA_CALIBRATION_FILE);
        if (ret)
            errorCode = 0;
        else
            errorCode = 1;
    }

    void LoadCameraCalibCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode)
    {
        const bool ret = instance->GetCameraCalibration()->LoadCameraCalib(CAMERA_CALIBRATION_FILE);
        if (ret)
            errorCode = 0;
        else
            errorCode = 1;
    }

    void CalibrateCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode)
    {
        IDAP::CameraCalibration* cameraCalib = instance->GetCameraCalibration();
        cameraCalib->Calibrate();
        if (cameraCalib->IsErrorOccure())
            errorCode = cameraCalib->GetErrorCode();
        else
            errorCode = 0;
    }

    void GetCalibrationCameraImageCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode,
        uint16_t& imageNumber, uint16_t& width, uint16_t& height, uint16_t& channels, uchar* data)
    {
        instance->GetCameraCalibration()->GetCameraCalibImage(errorCode, imageNumber, width, height, channels, data);
    }

    void CreateArucoMarkersCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode)
    {
        TableCalibration::CreateArucoMarkers(MARKERS_SAVE_PATH);
        errorCode = 0;
    }

    void DetectMarkersCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode)
    {
        instance->GetTableCalibration()->DetectMarkers(instance->getFrame());
		if (instance->GetTableCalibration()->HasFourPoints())
			errorCode = 0;
		else
			errorCode = 401;
    }

    void CalculateTableCalibrationResultCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode)
    {
        instance->GetTableCalibration()->CalculateTableCalibrationResults(instance->getFrame());
    }

    void GetProjectionTranformMatrixCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode,
        uint16_t& dataSizeAvailable, double& cmInPixels, double* data, double* tableCorners)
    {
        const bool success = instance->GetProjectorCalibration()->GetProjectionMatrix(data, cmInPixels, tableCorners, instance->getFrame(), instance->GetTableCalibration()->GetTableCalibrationResult());
        if(success)
        {
            errorCode = 0;
        }
        else
        {
            errorCode = 402;
        }
    }

    void SetChessboardDimensionProjectionCaller(ImageDetectionAccessPoint* instance, uint16_t& errorCode,
        uint16_t& width, uint16_t& height)
    {
        instance->GetProjectorCalibration()->SetChessboardDimension(static_cast<int>(width), static_cast<int>(height));
        errorCode = 0;
    }
}
