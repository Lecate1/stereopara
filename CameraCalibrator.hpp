#ifndef CAMERA_CALIBRATOR_HPP
#define CAMERA_CALIBRATOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <string>

/**
 * Класс для калибровки одной камеры по шахматной доске
 */
class CameraCalibrator {
private:
    cv::Size boardSize;      // Размер шахматной доски (количество углов)
    float squareSize;        // Размер клетки в мм
    
public:
    CameraCalibrator();
    
    // Установка параметров калибровки
    void setBoardSize(int width, int height, float squareSize);
    
    // Поиск углов шахматной доски на изображении
    bool findChessboardCorners(const cv::Mat& image, std::vector<cv::Point2f>& corners);
    
    // Захват изображений с камеры и поиск углов в реальном времени
    bool captureImages(int cameraId, int& capturedCount, 
                      std::vector<std::vector<cv::Point2f>>& allCorners,
                      cv::Size& imageSize);
    
    // Калибровка камеры по найденным углам
    bool calibrateCamera(const std::vector<std::vector<cv::Point2f>>& imagePoints,
                        const cv::Size& imageSize,
                        cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
    
    // Сохранение/загрузка параметров калибровки
    void saveCalibration(const std::string& filename, const cv::Mat& cameraMatrix, 
                        const cv::Mat& distCoeffs, const cv::Size& imageSize);
    bool loadCalibration(const std::string& filename, cv::Mat& cameraMatrix, 
                        cv::Mat& distCoeffs, cv::Size& imageSize);
};

#endif
