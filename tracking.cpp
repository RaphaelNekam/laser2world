#include "tracking.h"

PointRingBuffer::PointRingBuffer() {
	for (int i = 0; i < bufferLength; ++i) {
		buffer[i] = cv::Point2f(-1.0f, -1.0f);
	}
}

void PointRingBuffer::add(const cv::Point2f& pt) {
	buffer[index] = pt;
	index = (index + 1) % bufferLength;
	if (size < bufferLength) size++;
}

cv::Point2f PointRingBuffer::get(int i) {
	if (i < 0 || i >= size) throw std::out_of_range("Invalid index");
	int pos = (index + i) % bufferLength;
	return buffer[pos];
}

cv::Point2f PointRingBuffer::getAverage() {
	if (size == 0) {
		return cv::Point2f(-1.0, -1.0);
	}

	float sumX = 0.0f;
	float sumY = 0.0f;

	for (int i = 0; i < size; ++i) {
		cv::Point2f pt = get(i);
		sumX += pt.x;
		sumY += pt.y;
	}

	return cv::Point2f(sumX / size, sumY / size);
}

bool PointRingBuffer::allWithinTolerance(float tolerance_x, float tolerance_y) {
	if (size < bufferLength) {
		return false;
	}
	
	for (int i = 0; i < size; ++i) {
		for (int j = i + 1; j < size; ++j) {
			cv::Point2f a = get(i);
			cv::Point2f b = get(j);
			
			if (a.x == -1 || a.y == -1) {
				return false;
			}
			if (b.x == -1 || b.y == -1) {
				return false;
			}

			if (std::abs(a.x - b.x) > tolerance_x || std::abs(a.y - b.y) > tolerance_y) {
				return false;
			}
		}
	}
	return true;
}

Tracking::Tracking(cv::Scalar targetRGB, int tolerance)
    : targetRGB(targetRGB), tolerance(tolerance) {
		if (!loadHomography()) {
			std::cout << "An error reading homography.yaml has occurred" << std::endl;
		}
}

bool Tracking::handleFrame(const cv::Mat& frame) {
	
	auto start = std::chrono::high_resolution_clock::now();
	
    if (getTrackingDone()) {
		return true;
	}
	
	currentFrame = frame.clone();

    currentFrame = markColor(currentFrame);
    currentFrame = closeGaps(currentFrame);
    currentFrame = filterRoundClustersByShape(currentFrame);
    currentFrame = keepLargestFeature(currentFrame);
   
    cv::Point center = findCenter(currentFrame);
    cv::Point realWorldCenter = pixelCoord2WorldCoord(center);
    
    ringBuffer.add(realWorldCenter);
        
    if (debug) {
		showImage(frame, center);
	}

    if(ringBuffer.allWithinTolerance() && !debug) {
		trackingDone = true;
	}
	
	if (debug) {
		auto end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		std::cout << "Function handleFrame() took " << duration.count() << " ms" << std::endl;
		std::cout << "Pixelcenter: " << center << std::endl;
		std::cout << "Worldcenter: " << realWorldCenter << std::endl;
	}
	    
    return getTrackingDone();
}


cv::Mat Tracking::markColor(const cv::Mat& image) {	
	cv::Scalar lowerBound(
		std::max(0.0, targetRGB[0] - tolerance),
		std::max(0.0, targetRGB[1] - tolerance),
		std::max(0.0, targetRGB[2] - tolerance)
	);

	cv::Scalar upperBound(
		std::min(255.0, targetRGB[0] + tolerance),
		std::min(255.0, targetRGB[1] + tolerance),
		std::min(255.0, targetRGB[2] + tolerance)
	);

	cv::Mat mask;
	cv::inRange(image, lowerBound, upperBound, mask);
	
	return mask;
}

cv::Mat Tracking::closeGaps(const cv::Mat& binaryMask, int kernel_size) {
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));

	cv::Mat mask = binaryMask.clone();
	cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 1);

	cv::erode(mask, mask, kernel, cv::Point(-1, -1), 1);
	
	return mask;
}


cv::Mat Tracking::filterRoundClustersByShape(const cv::Mat& binaryMask, std::pair<double, double> aspectRatioRange){
	cv::Mat labels, stats, centroids;
	int numLabels = cv::connectedComponentsWithStats(binaryMask, labels, stats, centroids, 8);

	cv::Mat filtered = cv::Mat::zeros(binaryMask.size(), CV_8UC1);

	// Filter components by aspect ratio
	for (int i = 1; i < numLabels; ++i) { // Skip background (label 0)
		int x = stats.at<int>(i, cv::CC_STAT_LEFT);
		int y = stats.at<int>(i, cv::CC_STAT_TOP);
		int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
		int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
		
		double aspectRatio = (h != 0) ? static_cast<double>(w) / h : 0.0;

		if (aspectRatio >= aspectRatioRange.first && aspectRatio <= aspectRatioRange.second) {
			filtered.setTo(255, labels == i);
		}
	}
	
	return filtered;
}


cv::Mat Tracking::keepLargestFeature(const cv::Mat& binary_mask) {
	cv::Mat labels, stats, centroids;
	int num_labels = cv::connectedComponentsWithStats(binary_mask, labels, stats, centroids, 8, CV_32S);

	if (num_labels <= 1) {
		return binary_mask.clone();
	}

	// Find the largest component (excluding background)
	int largest_label = 1;
	int largest_area = stats.at<int>(1, cv::CC_STAT_AREA);
	for (int i = 2; i < num_labels; ++i) {
		int area = stats.at<int>(i, cv::CC_STAT_AREA);
		if (area > largest_area) {
			largest_area = area;
			largest_label = i;
		}
	}

	cv::Mat largest_mask = cv::Mat::zeros(binary_mask.size(), CV_8UC1);
	largest_mask.setTo(255, labels == largest_label);

	return largest_mask;
}

cv::Point Tracking::findCenter(const cv::Mat& mask) {
	if (mask.empty()) {
		std::cerr << "Error: Input mask is empty." << std::endl;
		return cv::Point(-1, -1);
	}

	if (mask.type() != CV_8U) {
		std::cerr << "Error: Input mask must be of type CV_8U." << std::endl;
		return cv::Point(-1, -1);
	}

	for (int r = 0; r < mask.rows; ++r) {
		for (int c = 0; c < mask.cols; ++c) {
			uchar pixelValue = mask.at<uchar>(r, c);
			if (pixelValue != 0 && pixelValue != 255) {
				std::cerr << "Error: Input mask must be binary (only 0 and 255 values allowed)." << std::endl;
				return cv::Point(-1, -1);
			}
		}
	}

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	if (contours.empty()) {
		return cv::Point(-1, -1);
	}

	size_t largestIndex = 0;
	double maxArea = 0.0;
	for (size_t i = 0; i < contours.size(); ++i) {
		double area = cv::contourArea(contours[i]);
		if (area > maxArea) {
			maxArea = area;
			largestIndex = i;
		}
	}

	cv::Moments M = cv::moments(contours[largestIndex]);
	if (M.m00 == 0) {
		return cv::Point(-1, -1);
	}

	int cX = static_cast<int>(M.m10 / M.m00);
	int cY = static_cast<int>(M.m01 / M.m00);
	return cv::Point(cX, cY);
}

bool Tracking::loadHomography() {
    cv::FileStorage fs("homography.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open homography file." << std::endl;
        return false;
    }

    fs["homography"] >> homography;
    return !homography.empty();
}

cv::Point2f Tracking::pixelCoord2WorldCoord(const cv::Point pixelCoord) {
	
	if (pixelCoord.x == -1 || pixelCoord.y == -1) {
		return cv::Point2f(-1.0, -1.0);
	}
	
	cv::Mat pixelHomogeneous = (cv::Mat_<double>(3,1) << pixelCoord.x, pixelCoord.y, 1.0);
	cv::Mat worldHomogeneous = homography * pixelHomogeneous;
	
	double w = worldHomogeneous.at<double>(2, 0);
	double x_cm = worldHomogeneous.at<double>(0, 0) / w;
	double y_cm = worldHomogeneous.at<double>(1, 0) / w;
	
	return cv::Point2f(x_cm, y_cm);
}


void Tracking::showImage(const cv::Mat& image, const cv::Point& center) {
	cv::Mat img;
	cv::cvtColor(image, img, cv::COLOR_RGB2BGR); // Fix problem with libcamera2opencv formatting
	
	
	if (center.x != -1 && center.y != -1) {
		int crossLength = 40;
		int crossThickness = 10;

		cv::line(img, cv::Point(center.x - crossLength, center.y), 
					  cv::Point(center.x + crossLength, center.y), 
					  cv::Scalar(255, 0, 0), crossThickness);

		cv::line(img, cv::Point(center.x, center.y - crossLength), 
					  cv::Point(center.x, center.y + crossLength), 
					  cv::Scalar(255, 0, 0), crossThickness);
	};
		
	float scaleFactor = 0.5;
	int width = static_cast<int>(img.cols * scaleFactor);
	int height = static_cast<int>(img.rows * scaleFactor);
	cv::resize(img, img, cv::Size(width, height));
	cv::imshow("Tracking", img);
	cv::waitKey(1);
}

cv::Point2f Tracking::getTargetLocation() {
	return ringBuffer.getAverage();
}


bool Tracking::getTrackingDone() {
	return trackingDone;
}





