#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

class Calibration {
	public:
		Calibration();
		bool handleFrame(const cv::Mat& frame); // called by the camera callback
		bool loadHomography();
		const cv::Mat& getHomography() const;
		bool getCalibrationDone();
		

	private:
		void computeHomography();
		static void onMouse(int event, int x, int y, int, void* userdata);
		cv::Mat getHomographyFormatFromPoints(std::vector<cv::Point2f> points);
		cv::Mat addGridToImage(const cv::Mat& inputImage);

		std::vector<cv::Point2f> imagePoints;
		std::vector<cv::Point2f> worldPoints;
		cv::Mat currentFrame;
		cv::Mat homography;
		bool calibrationDone = false;
		std::string windowName = "Calibration";
		float scaleFactor = 0.5;
};


#endif // CALIBRATION_H
