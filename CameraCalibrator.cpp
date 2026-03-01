#include "CameraCalibrator.hpp"
#include <iostream>

CameraCalibrator::CameraCalibrator() {
    boardSize = cv::Size(9, 6);
    squareSize = 25.0f;
}

void CameraCalibrator::setBoardSize(int width, int height, float squareSize) {
    this->boardSize = cv::Size(width, height);
    this->squareSize = squareSize;
}

bool CameraCalibrator::findChessboardCorners(const cv::Mat& image, std::vector<cv::Point2f>& corners) {
    cv::Mat gray;
    if (image.channels() == 3)
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else
        gray = image.clone();
    
    bool found = cv::findChessboardCorners(gray, boardSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
    
    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }
    return found;
}

bool CameraCalibrator::captureImages(int cameraId, int& capturedCount, 
                                    std::vector<std::vector<cv::Point2f>>& allCorners,
                                    cv::Size& imageSize) {
    cv::VideoCapture cap(cameraId);
    if (!cap.isOpened()) return false;
    
    cv::Mat frame;
    std::vector<cv::Point2f> corners;
    capturedCount = 0;
    allCorners.clear();
    
    std::cout << "\nКалибровка камеры " << cameraId << std::endl;
    std::cout << "Пробел - захватить, ESC - завершить" << std::endl;
    
    while (true) {
        cap >> frame;
        if (frame.empty()) continue;
        
        if (imageSize.width == 0) imageSize = frame.size();
        
        cv::Mat display = frame.clone();
        bool found = findChessboardCorners(frame, corners);
        
        if (found)
            cv::drawChessboardCorners(display, boardSize, corners, found);
        
        cv::putText(display, "Captured: " + std::to_string(capturedCount), 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
        cv::imshow("Camera " + std::to_string(cameraId), display);
        
        int key = cv::waitKey(1);
        if (key == ' ' && found) {
            allCorners.push_back(corners);
            capturedCount++;
            std::cout << "Захвачено: " << capturedCount << std::endl;
        } else if (key == 27) break;
    }
    
    cv::destroyWindow("Camera " + std::to_string(cameraId));
    return capturedCount > 0;
}

bool CameraCalibrator::calibrateCamera(const std::vector<std::vector<cv::Point2f>>& imagePoints,
                                      const cv::Size& imageSize,
                                      cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
    
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<cv::Point3f> obj;
    
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
    
    for (size_t i = 0; i < imagePoints.size(); i++)
        objectPoints.push_back(obj);
    
    if (imagePoints.size() < 5) {
        std::cerr << "Нужно минимум 5 изображений" << std::endl;
        return false;
    }
    
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                     cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout << "RMS ошибка: " << rms << std::endl;
    return true;
}

void CameraCalibrator::saveCalibration(const std::string& filename, const cv::Mat& cameraMatrix, 
                                      const cv::Mat& distCoeffs, const cv::Size& imageSize) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs << "imageSize" << imageSize;
    fs.release();
    std::cout << "Сохранено в " << filename << std::endl;
}

bool CameraCalibrator::loadCalibration(const std::string& filename, cv::Mat& cameraMatrix, 
                                      cv::Mat& distCoeffs, cv::Size& imageSize) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs["imageSize"] >> imageSize;
    fs.release();
    return true;
}
