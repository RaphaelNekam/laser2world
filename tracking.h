#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <chrono>


class PointRingBuffer {
private:
	static const int bufferLength = 5;
    cv::Point2f buffer[bufferLength];
    int index = 0;
    int size = 0;

public:
    PointRingBuffer();
	void add(const cv::Point2f& pt);
	cv::Point2f get(int i);
	bool allWithinTolerance(float tolerance_x = 15, float tolerance_y = 15);
	cv::Point2f getAverage();
};

class Tracking {
	public:
		Tracking(cv::Scalar targetBGR = cv::Scalar(255, 0, 118), int tolerance = 70);
		bool handleFrame(const cv::Mat& frame); // called by the camera callback
		cv::Point2f getTargetLocation();
		bool getTrackingDone();
		bool debug = false;

	private:
		cv::Scalar targetRGB;
		int tolerance;
		cv::Mat homography;
		cv::Mat currentFrame;
		PointRingBuffer ringBuffer;	
		bool trackingDone = false;	
		
		
		bool loadHomography();
		cv::Mat markColor(const cv::Mat& image);
		cv::Mat closeGaps(const cv::Mat& binary_mask, int kernel_size = 5);
		cv::Mat filterRoundClustersByShape(const cv::Mat& binaryMask, std::pair<double, double> aspectRatioRange = {0.5, 2.33});
		cv::Mat keepLargestFeature(const cv::Mat& binary_mask);
		cv::Point findCenter(const cv::Mat& mask);
		cv::Point2f pixelCoord2WorldCoord(const cv::Point pixelCoord);
		void showImage(const cv::Mat& image, const cv::Point& center = cv::Point(-1, -1));
};

#endif // TRACKING_H
