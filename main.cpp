#include "main.h"

void CalibrationCameraCallback::hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
    if (calibration->handleFrame(frame)) {
		updateAppState(AppState::TRACKING);
    }
}

void TrackingCameraCallback::hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
    if (tracking->handleFrame(frame)) {
		std::cout << "TARGET FOUND" << std::endl;
    }
}

Libcam2OpenCVSettings getCalibrationCameraSettings() {
    Libcam2OpenCVSettings settings;
    settings.width = 4608 / 2;
    settings.height = 2592 / 2;
    return settings;
}

Libcam2OpenCVSettings getTrackingCameraSettings() {
    Libcam2OpenCVSettings settings;
    settings.width = 4608 / 2;
    settings.height = 2592 / 2;
    settings.framerate = 1;
    return settings;
}

void updateAppState(AppState newAppState) {
	AppState current = newAppState;
	AppState prev = appState;
	appState = newAppState;
		
	switch(current) {
		case AppState::CALIBRATION: {
			calibrationCameraCallback.calibration = &calibration;
			calibrationCamera.registerCallback(&calibrationCameraCallback);
			calibrationCamera.start(getCalibrationCameraSettings());
			break;
		}
		
		case AppState::TRACKING: {
			trackingCameraCallback.tracking = &tracking;
			calibrationCamera.registerCallback(&trackingCameraCallback);
			break;
		}
			
		default:
			break;
	}
}




AppState appState;

Calibration calibration;
Libcam2OpenCV calibrationCamera;
CalibrationCameraCallback calibrationCameraCallback;

Tracking tracking;
Libcam2OpenCV trackingCamera;
TrackingCameraCallback trackingCameraCallback;

int main() {	
	
	appState = AppState::STARTUP;
		
	updateAppState(AppState::CALIBRATION);
	  
	
	while (appState != AppState::EXIT) {
	}
	
	return 0;
}
