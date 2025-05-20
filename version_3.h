#ifndef VERSION_3_H
#define VERSION_3_H

#include <libcam2opencv.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera.h>
#include <vector>
#include <thread>
#include <chrono>
#include <string>



struct ScaledMat {
	cv::Mat mat;
	float scalingFactor;
};

class Laserpointer2World {
	public:
		Laserpointer2World();
		void start();
		void stop();
		void calibrate();
		cv::Mat getLatestImageScaled();
		
	private:
		Libcam2OpenCV camera;
		Libcam2OpenCVSettings cameraSettings;
		float m_imageScaleFactor = 0.4;
		
		
		ScaledMat m_CALIBRATION_worldPoints = {
			(cv::Mat_<float>(4, 3) << 40, 110, 1,
									 -40, 110, 1,
									 -20, 30, 1,
									  20, 30, 1),
			1
		};
		
		ScaledMat m_CALIBRATION_imagePoints = {
			cv::Mat::ones(4, 3, CV_32F),
			m_imageScaleFactor
		};
		
		cv::Mat m_CALIBRATE_homographyMatrix;

		int m_CALIBRATION_nImagePointsSelected = 0;
		
		struct CameraCallback : Libcam2OpenCV::Callback {
			void hasFrame(const cv::Mat &frame, const libcamera::ControlList &metadata) override;
		} cameraCallback;

		void configureSettings();
		void startCamera();
		void stopCamera();
		
		void CALIBRATE_showImageWithMarkers();
		static void calibrationMouseCallback(int event, int x, int y, int flags, void* param);
		void CALIBRATE_getImagePoints();
		void CALIBRATE_computeHomography(cv::Mat imagePoints, cv::Mat worldPoints);
		void CALIBRATE_loadHomography();
		cv::Mat CALIBRATE_getNormalizeScaledMat(ScaledMat scaledMat);
		static cv::Mat latestImage; 
};

#endif // VERSION_3_HPP
