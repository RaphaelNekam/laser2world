

#include "version_3.h"

cv::Mat Laserpointer2World::latestImage;

Laserpointer2World::Laserpointer2World() {
    configureSettings();
    camera.registerCallback(&cameraCallback);
}

void Laserpointer2World::configureSettings() {
    cameraSettings.brightness = 0.0;       // 0.0 lets libcamera decide
    cameraSettings.contrast = 1.0;         // standard is 1.0
    cameraSettings.saturation = 1.0;       // standard is 1.0, 0.0 is greyscale
    cameraSettings.exposureValue = 0.0;    // standard is 0.0
    cameraSettings.exposureTime = 0;       // in µs, 0 lets libcamera decide
    cameraSettings.width = 4608 / 2;       // 0 lets libcamera decide
    cameraSettings.height = 2592 / 2;      // 0 lets libcamera decide
    cameraSettings.framerate = 1;          // 0 lets libcamera decide
    cameraSettings.lensPosition = 0.0;     // -1 for auto
}


void Laserpointer2World::CameraCallback::hasFrame(const cv::Mat &frame, const libcamera::ControlList &metadata) {
    latestImage = frame.clone();
}


void Laserpointer2World::start() {
    calibrate();
    stop();
}

void Laserpointer2World::startCamera() {
    camera.start(cameraSettings);
}

void Laserpointer2World::stopCamera() {
    cv::destroyAllWindows();
    camera.stop();
}

void Laserpointer2World::stop() {
    cv::destroyAllWindows();
    stopCamera();
}

void Laserpointer2World::calibrationMouseCallback(int event, int x, int y, int flags, void* param) {
    auto* self = static_cast<Laserpointer2World*>(param);

    if (event == cv::EVENT_LBUTTONDOWN) {
	self->m_CALIBRATION_imagePoints.mat.at<float>(self->m_CALIBRATION_nImagePointsSelected, 0) = x;
	self->m_CALIBRATION_imagePoints.mat.at<float>(self->m_CALIBRATION_nImagePointsSelected, 1) = y;
	self->m_CALIBRATION_nImagePointsSelected++;
    }

}

cv::Mat Laserpointer2World::CALIBRATE_getNormalizeScaledMat(ScaledMat scaledMat){
    cv::Mat normalized = scaledMat.mat.clone();
    
    for (int i = 0; i < normalized.rows; i++) {
	normalized.at<float>(i, 0) /= scaledMat.scalingFactor; // x
        normalized.at<float>(i, 1) /= scaledMat.scalingFactor; // y
        normalized.at<float>(i, 2) = 1.0f;                     // keep homogeneous coordinate
    }
    return normalized;
}



void Laserpointer2World::CALIBRATE_getImagePoints() {
    startCamera();

    while(m_CALIBRATION_nImagePointsSelected < 4) {
	cv::Mat calibrationImage = getLatestImageScaled().clone();
	
	if (!calibrationImage.empty()) {
	    cv::Point worldPoint(
		m_CALIBRATION_worldPoints.mat.at<float>(m_CALIBRATION_nImagePointsSelected, 0),
		m_CALIBRATION_worldPoints.mat.at<float>(m_CALIBRATION_nImagePointsSelected, 1)
	    );
	    std::string msg = "Place @ x=" + std::to_string(worldPoint.x) + "cm, y=" + std::to_string(worldPoint.y) + "cm";
	    cv::putText(calibrationImage, msg, cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
	    
	    for (int i = 0; i < m_CALIBRATION_nImagePointsSelected; i++) {
		cv::Point imagePoint(
		    m_CALIBRATION_imagePoints.mat.at<float>(i, 0),
		    m_CALIBRATION_imagePoints.mat.at<float>(i, 1)
		);
		cv::drawMarker(calibrationImage, imagePoint, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
	    }
	    
	    cv::imshow("Calibration", calibrationImage);
	    cv::setMouseCallback("Calibration", calibrationMouseCallback, this);
	    cv::waitKey(500);
	}
    }
    
    cv::Mat calibrationImage = getLatestImageScaled().clone();
    for (int i = 0; i < m_CALIBRATION_nImagePointsSelected; i++) {
	cv::Point imagePoint(
	    m_CALIBRATION_imagePoints.mat.at<float>(i, 0),
	    m_CALIBRATION_imagePoints.mat.at<float>(i, 1)
	);
	cv::drawMarker(calibrationImage, imagePoint, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
    }
    cv::imshow("Calibration", calibrationImage);
    cv::waitKey(1);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void Laserpointer2World::CALIBRATE_computeHomography(cv::Mat imagePoints, cv::Mat worldPoints) {
    m_CALIBRATE_homographyMatrix = cv::findHomography(imagePoints, worldPoints);
    cv::FileStorage fs("../homography.yaml", cv::FileStorage::WRITE);
    fs << "m_CALIBRATE_homographyMatrix" << m_CALIBRATE_homographyMatrix;
    fs.release();
    std::cout << "Homography saved in 'homography.yaml'." << std::endl;
}

void Laserpointer2World::CALIBRATE_loadHomography() {
    cv::FileStorage fs("../homography.yaml", cv::FileStorage::READ);
    
    if (!fs.isOpened()) {
        std::cerr << "Failed to open homography.yaml" << std::endl;
        return;
    }

    fs["m_CALIBRATE_homographyMatrix"] >> m_CALIBRATE_homographyMatrix;
    fs.release();

    std::cout << "Homography loaded from 'homography.yaml':\n" << m_CALIBRATE_homographyMatrix << std::endl;
}


void Laserpointer2World::calibrate() {    
    CALIBRATE_getImagePoints();
    CALIBRATE_computeHomography(CALIBRATE_getNormalizeScaledMat(m_CALIBRATION_imagePoints), CALIBRATE_getNormalizeScaledMat(m_CALIBRATION_worldPoints));
}


cv::Mat Laserpointer2World::getLatestImageScaled() {
        
    int width = static_cast<int>(latestImage.cols * m_imageScaleFactor);
    int height = static_cast<int>(latestImage.rows * m_imageScaleFactor);
    
    if (latestImage.empty() || width <= 0 || height <= 0) {
	return cv::Mat();
    }
    
    cv::Mat resized_image;
    cv::resize(latestImage, resized_image, cv::Size(width, height));
    return resized_image;
}



int main() {
    Laserpointer2World laserpointer2World;
    laserpointer2World.start();    
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return 0;
}
