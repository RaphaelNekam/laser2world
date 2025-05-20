#include "calibration.h"
#include <iostream>
#include <libcam2opencv.h>

Libcam2OpenCVSettings getCalibrationCameraSettings() {
    Libcam2OpenCVSettings settings;
    settings.width = 4608 / 2;
    settings.height = 2592 / 2;
    return settings;
}

struct CalibrationCameraCallback : Libcam2OpenCV::Callback {
    Calibration* calibration = nullptr;
    virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) override;
};

void CalibrationCameraCallback::hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
    calibration->handleFrame(frame);
}


Calibration calibration;
Libcam2OpenCV calibrationCamera;
CalibrationCameraCallback calibrationCameraCallback;


int main() {
    std::cout << "Start Calibration" << std::endl;
    
	calibrationCameraCallback.calibration = &calibration;
	calibrationCamera.registerCallback(&calibrationCameraCallback);
	calibrationCamera.start(getCalibrationCameraSettings());
    
	while(!calibration.getCalibrationDone()) {}
	
	calibrationCamera.stop();
    
    return 0;
}
