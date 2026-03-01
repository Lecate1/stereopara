#ifndef STEREO_CALIBRATOR_HPP
#define STEREO_CALIBRATOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <string>
#include "CameraCalibrator.hpp"

// Структура для хранения всех параметров стереокалибровки
struct StereoParams {
    cv::Mat cameraMatrix1, distCoeffs1;  // Параметры левой камеры
    cv::Mat cameraMatrix2, distCoeffs2;  // Параметры правой камеры
    cv::Mat R, T;        // Поворот и перенос между камерами
    cv::Mat E, F;        // Essential и Fundamental матрицы
    cv::Mat R1, R2;      // Матрицы ректификации
    cv::Mat P1, P2;      // Матрицы проекции
    cv::Mat Q;           // Матрица для преобразования disparity в глубину
    cv::Size imageSize;   // Размер изображения
};

// Структура для хранения углов с обеих камер
struct StereoImagePoints {
    std::vector<std::vector<cv::Point2f>> points1;  // Углы с левой камеры
    std::vector<std::vector<cv::Point2f>> points2;  // Углы с правой камеры
    cv::Size imageSize;
};

/**
 * Класс для калибровки стереопары (двух камер вместе)
 */
class StereoCalibrator {
private:
    cv::Size boardSize;    // Размер шахматной доски
    float squareSize;      // Размер клетки
    
public:
    StereoCalibrator();
    
    // Установка параметров доски
    void setBoardSize(int width, int height, float squareSize);
    
    // Захват стереопар для калибровки
    bool captureStereoPairs(int leftCameraId, int rightCameraId,
                           StereoImagePoints& imagePoints,
                           int& capturedCount);
    
    // Выполнение стереокалибровки
    bool calibrateStereo(const StereoImagePoints& imagePoints,
                        const cv::Mat& cameraMatrix1, const cv::Mat& distCoeffs1,
                        const cv::Mat& cameraMatrix2, const cv::Mat& distCoeffs2,
                        StereoParams& params);
    
    // Сохранение/загрузка стереокалибровки
    void saveCalibration(const std::string& filename, const StereoParams& params);
    bool loadCalibration(const std::string& filename, StereoParams& params);
};

#endif
