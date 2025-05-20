#include "calibration.h"

Calibration::Calibration() {
	std::string windowName = "Calibration";
	
	worldPoints = {
        {0, 140}, {0, 100}, {0, 60}, {0, 20}, {40, 100}, {-40, 100}, {10, 20}, {-10, 20}
    };
    
	cv::namedWindow(windowName);
	cv::setMouseCallback(windowName, onMouse, this);

}

bool Calibration::getCalibrationDone() {
	return calibrationDone;
}

bool Calibration::handleFrame(const cv::Mat& frame) {
    if (calibrationDone) return true;

    cv::cvtColor(frame, currentFrame, cv::COLOR_RGB2BGR); // Fix problem with libcamera2opencv formatting
    
	std::string msg = "Place @ x=";
	msg += std::to_string(static_cast<int>(worldPoints[imagePoints.size()].x));
	msg += "cm, y=";
	msg += std::to_string(static_cast<int>(worldPoints[imagePoints.size()].y));
	msg += "cm";
	cv::putText(currentFrame, msg, cv::Point(10,100), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);


	int width = currentFrame.cols;
	int height = currentFrame.rows;
	int centerX = width / 2;
	cv::Scalar lineColor(0, 0, 255);
	int lineThickness = 2;
	cv::Point top(centerX, 0);                     
	cv::Point bottom(centerX, currentFrame.rows);
	cv::line(currentFrame, top, bottom, lineColor, lineThickness);
	
	if (imagePoints.size() >= 4) {
		cv::Point left(0, imagePoints[1].y);
		cv::Point right(width, imagePoints[1].y);
		cv::line(currentFrame, left, right, lineColor, lineThickness);
	}
	
	if (imagePoints.size() >= 6) {
		cv::Point left(0, imagePoints[5].y);
		cv::Point right(width, imagePoints[5].y);
		cv::line(currentFrame, left, right, lineColor, lineThickness);
	}


    // Draw selected points
    for (const auto& pt : imagePoints) {
        cv::circle(currentFrame, pt, 10, cv::Scalar(0, 255, 0), -1);
    }

    int scaled_width = static_cast<int>(currentFrame.cols * scaleFactor);
    int scaled_height = static_cast<int>(currentFrame.rows * scaleFactor);
    
    cv::Mat resizedFrame;
    cv::resize(currentFrame, resizedFrame, cv::Size(scaled_width, scaled_height));
    
    cv::imshow(windowName, resizedFrame);
    cv::waitKey(1); // allow GUI to update

    return calibrationDone;
}

void Calibration::onMouse(int event, int x, int y, int, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) return;

    Calibration* self = static_cast<Calibration*>(userdata);
    if (self->imagePoints.size() >= self->worldPoints.size()) return;
        
    if (self->imagePoints.size() < 4) {
		x = self->currentFrame.cols / 2;
	} else {
		x = x / self->scaleFactor;
	}
	
	y = y / self->scaleFactor; // Unscale as image clicked on is scaled
    
     // Unscale as image clicked on is scaled

    self->imagePoints.push_back(cv::Point2f(x, y));
    std::cout << "Clicked: " << x << ", " << y << std::endl;

    if (self->imagePoints.size() == self->worldPoints.size()) {
        self->computeHomography();
    }
}

void Calibration::computeHomography() {	
    homography = cv::findHomography(getHomographyFormatFromPoints(imagePoints), getHomographyFormatFromPoints(worldPoints));
    cv::FileStorage fs("homography.yaml", cv::FileStorage::WRITE);
    fs << "homography" << homography;
    fs.release();

    calibrationDone = true;
    std::cout << homography << std::endl;
    std::cout << "Calibration done. Homography saved." << std::endl;
    
    cv::Mat frameWithGrid = addGridToImage(currentFrame);
    cv::imwrite("calibration_grid.jpg", frameWithGrid);
    
    cv::destroyWindow(windowName);
}

bool Calibration::loadHomography() {
    cv::FileStorage fs("homography.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open homography file." << std::endl;
        return false;
    }

    fs["homography"] >> homography;
    return !homography.empty();
}

cv::Mat Calibration::getHomographyFormatFromPoints(std::vector<cv::Point2f> points) {
	cv::Mat homographyMat(points.size(), 3, CV_32F);
	
	for (int i = 0; i < points.size(); i++) {
		homographyMat.at<float>(i, 0) = points[i].x;
		homographyMat.at<float>(i, 1) = points[i].y;
		homographyMat.at<float>(i, 2) = 1;
	}
	
	return homographyMat;
}

cv::Mat Calibration::addGridToImage(const cv::Mat& inputImage) {
    cv::Mat image = inputImage.clone();

    cv::Mat H_inv = homography.inv();

    std::vector<int> x_range, y_range;
    for (int x = -40; x <= 40; x += 10) x_range.push_back(x);
    for (int y = 20; y <= 100; y += 10) y_range.push_back(y);

    for (int x : x_range) {
        for (int y : y_range) {
            std::vector<cv::Point2f> world_point = { cv::Point2f(x, y) };
            std::vector<cv::Point2f> img_point;
            cv::perspectiveTransform(world_point, img_point, H_inv);
            int px = static_cast<int>(img_point[0].x);
            int py = static_cast<int>(img_point[0].y);

            cv::drawMarker(image, cv::Point(px, py), cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);
        }
    }

    for (int x : x_range) {
        std::vector<cv::Point> line;
        for (int y : y_range) {
            std::vector<cv::Point2f> world_point = { cv::Point2f(x, y) };
            std::vector<cv::Point2f> img_point;
            cv::perspectiveTransform(world_point, img_point, H_inv);
            line.emplace_back(static_cast<int>(img_point[0].x), static_cast<int>(img_point[0].y));
        }
        for (size_t i = 0; i < line.size() - 1; ++i) {
            cv::line(image, line[i], line[i + 1], cv::Scalar(255, 0, 0), 1);
        }
    }

    for (int y : y_range) {
        std::vector<cv::Point> line;
        for (int x : x_range) {
            std::vector<cv::Point2f> world_point = { cv::Point2f(x, y) };
            std::vector<cv::Point2f> img_point;
            cv::perspectiveTransform(world_point, img_point, H_inv);
            line.emplace_back(static_cast<int>(img_point[0].x), static_cast<int>(img_point[0].y));
        }
        for (size_t i = 0; i < line.size() - 1; ++i) {
            cv::line(image, line[i], line[i + 1], cv::Scalar(255, 0, 0), 1);
        }
    }

    return image;
}
