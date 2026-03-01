#ifndef DISPARITY_CALCULATOR_HPP
#define DISPARITY_CALCULATOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

/**
 * Класс для вычисления карты глубины в реальном времени
 */
class DisparityCalculator {
private:
    // Карты для ректификации (предвычисленные)
    cv::Mat mapLeft1, mapLeft2;    // Для левой камеры
    cv::Mat mapRight1, mapRight2;  // Для правой камеры
    cv::Mat Q;                      // Матрица для глубины
    bool mapsReady;                  // Готовы ли карты
    
    // Алгоритм SGBM для вычисления disparity
    cv::Ptr<cv::StereoSGBM> sgbm;
    
public:
    DisparityCalculator();
    
    // Инициализация с параметрами калибровки
    bool initialize(const cv::Mat& cameraMatrix1, const cv::Mat& distCoeffs1,
                   const cv::Mat& cameraMatrix2, const cv::Mat& distCoeffs2,
                   const cv::Mat& R1, const cv::Mat& R2,
                   const cv::Mat& P1, const cv::Mat& P2,
                   const cv::Mat& Q, const cv::Size& imageSize);
    
    // Ректификация изображений (исправление искажений и выравнивание)
    void rectifyImages(const cv::Mat& left, const cv::Mat& right,
                       cv::Mat& leftRect, cv::Mat& rightRect);
    
    // Вычисление карты disparity
    cv::Mat computeDisparity(const cv::Mat& left, const cv::Mat& right);
    
    // Визуализация (близко = белый, далеко = черный)
    cv::Mat visualizeDepth(const cv::Mat& disparity);
    
    // Настройка параметров алгоритма
    void setParameters(int numDisparities, int blockSize, int uniquenessRatio);
};

#endif
