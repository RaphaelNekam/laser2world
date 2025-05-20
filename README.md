# Raspberry Pi Laser Pointer Detection and Real-World Mapping
## Overview
This project detects a laser pointer in a video stream from a Raspberry Pi camera and maps its location from pixel coordinates to real-world coordinates (in centimeters), using the camera as the origin of the coordinate system.

It forms part of a larger robotics path-finding system developed by Giulia Lafratta and Bernd Porr.

## Prerequisites
### Hardware and OS
- Raspberry Pi running Raspbian OS Bookworm
- Compatible Raspberry Pi camera module

### libcamera2opencv
- The project uses libcamera2opencv for accessing video frames. It relies on modifying the saturation parameter, which is not yet available in the main branch (as of 20/05/2025). Please use the version from the pending pull request (link to be added once available).
- Follow the installation instructions provided in the libcamera2opencv repository.

## Project Structure
- calibration.h/.cpp: Handles manual camera calibration and homography computation.
- tracking.h/.cpp: Detects the laser pointer and applies the homography to compute real-world coordinates.
- main_calibration.cpp: Example program for calibration.
- main_tracking.cpp: Example program for laser tracking.
- build/: Contains compiled executables and output files such as homography.yaml.

## Building the Project
To build the project, navigate to the build directory:

```
cd build
cmake ..
make
```

This produces two executables:
- `./Calibration` – for manual camera calibration

- `./Tracking` – for detecting the laser pointer and computing its real-world location

## Running example executables
### Running Calibration
To perform calibration, run:
```./build/Calibration```
A camera feed will open. You'll be asked to click on four known points in the image corresponding to known real-world positions. The resulting homography matrix is saved to homography.yaml. You should recalibrate if the camera’s position or lens changes.

### Running Tracking
To detect the laser pointer and obtain its real-world coordinates:
```
./Tracking
```
The application will process the video stream, detect the laser pointer, and print its [x, y] position in centimeters. Use the --debug flag to enable a live preview with the detected point marked in blue.

## Integration with Custom Code
You can integrate the tracking functionality into your own application using the Tracking class and libcamera2opencv callbacks. Example:
```
struct TrackingCameraCallback : Libcam2OpenCV::Callback {
    Tracking* tracking = nullptr;
    void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) override {
        tracking->handleFrame(frame);
    }
};

Tracking tracking(cv::Scalar(255, 0, 0), 70);  // target RGB and tolerance
TrackingCameraCallback cb{&tracking};

Libcam2OpenCV cam;
cam.registerCallback(&cb);
cam.start(getTrackingCameraSettings());

while (!tracking.getTrackingDone()) {}

std::cout << "Target at: " << tracking.getTargetLocation() << std::endl;
cam.stop();
```
- A file with suitable homography parameters (such as the `./build/homography.yaml` file) should be present as the tracking functionalities require this. You can create this matrix with the example calibration executable.
- It is recommended, to spawn a seperate thread for `while(!tracking.getTrackindDone()){}` to use the CPU effectively. However, this is not done in this exapmle as it was not deemed required to showcase the project.

## Adjustments
You may need to tune `targetRGB` and `targetTolerance` based on your laser pointer color and ambient lighting. Adjusting camera parameters (such as increased saturation or decreased brightness as in `./main_tracking.cpp` is helpful to perform a better detection of the laser pointer. These parameters should also be tweaked to match your lightning environement.

