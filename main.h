#ifndef MAIN_H
#define MAIN_H


#include <iostream>
#include <libcam2opencv.h>
#include "calibration.h"
#include "tracking.h"
#include <chrono>
#include <thread>

enum class AppState {
	STARTUP,
    CALIBRATION,
    TRACKING,
    EXIT
};

void updateAppState(AppState newAppState);


struct CalibrationCameraCallback : Libcam2OpenCV::Callback {
    Calibration* calibration = nullptr;
    virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) override;
};

struct TrackingCameraCallback : Libcam2OpenCV::Callback {
    Tracking* tracking = nullptr;
    virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) override;
};

bool cameraRunning = false;

extern AppState appState;

extern Calibration calibration;
extern Libcam2OpenCV calibrationCamera;
extern CalibrationCameraCallback calibrationCameraCallback;
Libcam2OpenCVSettings getCalibrationCameraSettings();


extern Tracking tracking;
extern Libcam2OpenCV trackingCamera;
extern TrackingCameraCallback trackingCameraCallback;
Libcam2OpenCVSettings getTrackingCameraSettings();



#endif // MAIN_H
