#include "StereoCalibrator.hpp"
#include <iostream>

StereoCalibrator::StereoCalibrator() {
    boardSize = cv::Size(9, 6);
    squareSize = 25.0f;
}

void StereoCalibrator::setBoardSize(int width, int height, float squareSize) {
    this->boardSize = cv::Size(width, height);
    this->squareSize = squareSize;
}

bool StereoCalibrator::captureStereoPairs(int leftId, int rightId, 
                                         StereoImagePoints& points, int& count) {
    cv::VideoCapture capLeft(leftId);
    cv::VideoCapture capRight(rightId);
    if (!capLeft.isOpened() || !capRight.isOpened()) return false;
    
    CameraCalibrator detector;
    detector.setBoardSize(boardSize.width, boardSize.height, squareSize);
    
    cv::Mat left, right;
    std::vector<cv::Point2f> cornersLeft, cornersRight;
    count = 0;
    
    std::cout << "\nКалибровка стереопары" << std::endl;
    std::cout << "Пробел - захватить, ESC - завершить" << std::endl;
    
    while (true) {
        capLeft >> left;
        capRight >> right;
        if (left.empty() || right.empty()) continue;
        
        if (points.imageSize.width == 0) points.imageSize = left.size();
        
        cv::Mat displayLeft = left.clone();
        cv::Mat displayRight = right.clone();
        
        bool foundLeft = detector.findChessboardCorners(left, cornersLeft);
        bool foundRight = detector.findChessboardCorners(right, cornersRight);
        
        if (foundLeft) 
            cv::drawChessboardCorners(displayLeft, boardSize, cornersLeft, foundLeft);
        if (foundRight) 
            cv::drawChessboardCorners(displayRight, boardSize, cornersRight, foundRight);
        
        cv::Mat stereo;
        cv::hconcat(displayLeft, displayRight, stereo);
        cv::putText(stereo, "Captured: " + std::to_string(count), 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
        cv::imshow("Stereo Pair", stereo);
        
        int key = cv::waitKey(1);
        if (key == ' ' && foundLeft && foundRight) {
            points.points1.push_back(cornersLeft);
            points.points2.push_back(cornersRight);
            count++;
            std::cout << "Захвачено: " << count << std::endl;
        } else if (key == 27) break;
    }
    
    cv::destroyWindow("Stereo Pair");
    return count > 0;
}

bool StereoCalibrator::calibrateStereo(const StereoImagePoints& points,
                                      const cv::Mat& cm1, const cv::Mat& dist1,
                                      const cv::Mat& cm2, const cv::Mat& dist2,
                                      StereoParams& params) {
    
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<cv::Point3f> obj;
    
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
    
    for (size_t i = 0; i < points.points1.size(); i++)
        objectPoints.push_back(obj);
    
    if (points.points1.size() < 3) {
        std::cerr << "Нужно минимум 3 стереопары" << std::endl;
        return false;
    }
    
    double rms = cv::stereoCalibrate(objectPoints, points.points1, points.points2,
                                    cm1, dist1, cm2, dist2,
                                    points.imageSize, params.R, params.T, 
                                    params.E, params.F, cv::CALIB_FIX_INTRINSIC);
    std::cout << "RMS ошибка: " << rms << std::endl;
    
    cv::stereoRectify(cm1, dist1, cm2, dist2, points.imageSize,
                     params.R, params.T, params.R1, params.R2, 
                     params.P1, params.P2, params.Q, cv::CALIB_ZERO_DISPARITY, 0);
    
    params.cameraMatrix1 = cm1;
    params.distCoeffs1 = dist1;
    params.cameraMatrix2 = cm2;
    params.distCoeffs2 = dist2;
    params.imageSize = points.imageSize;
    
    return true;
}

void StereoCalibrator::saveCalibration(const std::string& filename, const StereoParams& params) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "cameraMatrix1" << params.cameraMatrix1;
    fs << "distCoeffs1" << params.distCoeffs1;
    fs << "cameraMatrix2" << params.cameraMatrix2;
    fs << "distCoeffs2" << params.distCoeffs2;
    fs << "R" << params.R;
    fs << "T" << params.T;
    fs << "E" << params.E;
    fs << "F" << params.F;
    fs << "R1" << params.R1;
    fs << "R2" << params.R2;
    fs << "P1" << params.P1;
    fs << "P2" << params.P2;
    fs << "Q" << params.Q;
    fs << "imageSize" << params.imageSize;
    fs.release();
    std::cout << "Сохранено в " << filename << std::endl;
}

bool StereoCalibrator::loadCalibration(const std::string& filename, StereoParams& params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["cameraMatrix1"] >> params.cameraMatrix1;
    fs["distCoeffs1"] >> params.distCoeffs1;
    fs["cameraMatrix2"] >> params.cameraMatrix2;
    fs["distCoeffs2"] >> params.distCoeffs2;
    fs["R"] >> params.R;
    fs["T"] >> params.T;
    fs["E"] >> params.E;
    fs["F"] >> params.F;
    fs["R1"] >> params.R1;
    fs["R2"] >> params.R2;
    fs["P1"] >> params.P1;
    fs["P2"] >> params.P2;
    fs["Q"] >> params.Q;
    fs["imageSize"] >> params.imageSize;
    fs.release();
    return true;
}
