# Raspberry Pi Laser Pointer Detection and Real-World Mapping
## Overview
This project detects a laser pointer in a video stream from a Raspberry Pi camera and maps its location from pixel coordinates to real-world coordinates (in centimeters), using the camera as the origin of the coordinate system.

It forms part of a larger robotics path-finding system developed by [Giulia Lafratta](https://github.com/glafratta) and [Bernd Porr](https://github.com/berndporr).

## Report
Please find the full report [here](https://github.com/RaphaelNekam/laser2world/blob/main/laser2world_Report_Nekam.pdf) (Parts about LiDAR removed as not done by myself). The full document explores further techniques for tracking and pixel-to-real-world coordinate translation and compares them.

The most important pages are shown here:

<img width="660" height="856" alt="image" src="https://github.com/user-attachments/assets/586545a6-a8d5-4bb2-bc0b-1f06019b9c4f" />
<img width="662" height="857" alt="image" src="https://github.com/user-attachments/assets/ab66e5a6-2e38-4aee-a2de-0192e7cd3a5c" />
<img width="660" height="859" alt="image" src="https://github.com/user-attachments/assets/f33d17b6-5a1b-414f-ade7-225551e039a6" />
<img width="664" height="858" alt="image" src="https://github.com/user-attachments/assets/d6d54a2f-01bb-446c-b808-d7fa33efa7a4" />
<img width="664" height="857" alt="image" src="https://github.com/user-attachments/assets/cf22ee68-4612-441d-98b3-ae182db9e772" />
<img width="663" height="857" alt="image" src="https://github.com/user-attachments/assets/6725425a-6075-4308-b68b-35b3a36599b3" />
<img width="663" height="856" alt="image" src="https://github.com/user-attachments/assets/85618850-c422-46eb-a9cd-c8c988341c7e" />
<img width="664" height="287" alt="image" src="https://github.com/user-attachments/assets/92064fff-ac60-4468-b119-5a3aa9645bf3" />

## Prerequisites
### Hardware and OS
- Raspberry Pi 3B+ running Raspbian OS Bookworm
- Compatible Raspberry Pi camera module

### libcamera2opencv
- The project uses [libcamera2opencv](https://github.com/berndporr/libcamera2opencv/) by [Bernd Porr](https://github.com/berndporr) for accessing video frames.
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

