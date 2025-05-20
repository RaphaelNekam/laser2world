#include "tracking.h"
#include <iostream>
#include <libcam2opencv.h>
#include <string>

Libcam2OpenCVSettings getTrackingCameraSettings() {
    Libcam2OpenCVSettings settings;
    settings.width = 4608 / 2;
    settings.height = 2592 / 2;
    settings.framerate = 2;
    settings.saturation = 3.0;
    settings.brightness = -0.25;
    return settings;
}

struct TrackingCameraCallback : Libcam2OpenCV::Callback {
    Tracking* tracking = nullptr;
    virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) override;
};

void TrackingCameraCallback::hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
    tracking->handleFrame(frame);
}



cv::Scalar targetRGB(255, 0, 0);
int targetTolerance = 70;
Tracking tracking(targetRGB, targetTolerance);

Libcam2OpenCV trackingCamera;
TrackingCameraCallback trackingCameraCallback;


int main(int argc, char* argv[]) {
    std::cout << "Start Tracking" << std::endl;
    
    for (int i = 1; i < argc; ++i) {
		std::string arg = argv[i];
		if (arg == "--debug") {
			tracking.debug = true;
		}
	}
    
	trackingCameraCallback.tracking = &tracking;
	trackingCamera.registerCallback(&trackingCameraCallback);
	trackingCamera.start(getTrackingCameraSettings());
    
	while(!tracking.getTrackingDone()) {}
	
	std::cout << "Target at: " << tracking.getTargetLocation() << std::endl;
	
	trackingCamera.stop();
    
    return 0;
}
