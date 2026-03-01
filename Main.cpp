#include <iostream>
#include "CameraCalibrator.hpp"
#include "StereoCalibrator.hpp"
#include "DisparityCalculator.hpp"

// Прототипы функций
void calibrateSingleCamera(int cameraId);
void calibrateStereo();
void runDepthVideo();
void showMenu();

int main() {
    while (true) {
        showMenu();
        
        int choice;
        std::cin >> choice;
        
        switch (choice) {
            case 1: calibrateSingleCamera(0); break;
            case 2: calibrateSingleCamera(1); break;
            case 3: calibrateStereo(); break;
            case 4: runDepthVideo(); break;
            case 5: 
                std::cout << "До свидания!" << std::endl; 
                return 0;
            default: 
                std::cout << "Неверный выбор" << std::endl;
        }
    }
    return 0;
}

void calibrateSingleCamera(int cameraId) {
    std::cout << "\n=== Калибровка камеры " << cameraId << " ===" << std::endl;
    
    CameraCalibrator calibrator;
    int count = 0;
    std::vector<std::vector<cv::Point2f>> allCorners;
    cv::Size imageSize;
    
    if (!calibrator.captureImages(cameraId, count, allCorners, imageSize)) {
        std::cerr << "Ошибка захвата" << std::endl;
        return;
    }
    
    if (allCorners.size() < 5) {
        std::cerr << "Мало изображений" << std::endl;
        return;
    }
    
    cv::Mat cameraMatrix, distCoeffs;
    if (calibrator.calibrateCamera(allCorners, imageSize, cameraMatrix, distCoeffs)) {
        std::string filename = "camera" + std::to_string(cameraId) + "_calib.yml";
        calibrator.saveCalibration(filename, cameraMatrix, distCoeffs, imageSize);
    }
}

void calibrateStereo() {
    std::cout << "\n=== Калибровка стереопары ===" << std::endl;
    
    CameraCalibrator checker;
    cv::Mat cm1, dist1, cm2, dist2;
    cv::Size size1, size2;
    
    if (!checker.loadCalibration("camera0_calib.yml", cm1, dist1, size1) ||
        !checker.loadCalibration("camera1_calib.yml", cm2, dist2, size2)) {
        std::cerr << "Сначала откалибруйте обе камеры" << std::endl;
        return;
    }
    
    StereoCalibrator stereo;
    StereoImagePoints points;
    int count = 0;
    
    if (!stereo.captureStereoPairs(0, 1, points, count)) {
        std::cerr << "Ошибка захвата" << std::endl;
        return;
    }
    
    if (points.points1.size() < 3) {
        std::cerr << "Мало стереопар" << std::endl;
        return;
    }
    
    StereoParams params;
    if (stereo.calibrateStereo(points, cm1, dist1, cm2, dist2, params)) {
        stereo.saveCalibration("stereo_calib.yml", params);
    }
}

void runDepthVideo() {
    std::cout << "\n=== Запуск видео глубины ===" << std::endl;
    std::cout << "ESC - выход" << std::endl;
    
    StereoParams params;
    StereoCalibrator loader;
    
    if (!loader.loadCalibration("stereo_calib.yml", params)) {
        std::cerr << "Нет файла калибровки. Сначала выполните калибровку." << std::endl;
        return;
    }
    
    DisparityCalculator depthCalc;
    depthCalc.initialize(params.cameraMatrix1, params.distCoeffs1,
                        params.cameraMatrix2, params.distCoeffs2,
                        params.R1, params.R2, params.P1, params.P2,
                        params.Q, params.imageSize);
    
    cv::VideoCapture capLeft(0);
    cv::VideoCapture capRight(1);
    
    if (!capLeft.isOpened() || !capRight.isOpened()) {
        std::cerr << "Не удалось открыть камеры" << std::endl;
        return;
    }
    
    capLeft.set(cv::CAP_PROP_FRAME_WIDTH, params.imageSize.width);
    capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, params.imageSize.height);
    capRight.set(cv::CAP_PROP_FRAME_WIDTH, params.imageSize.width);
    capRight.set(cv::CAP_PROP_FRAME_HEIGHT, params.imageSize.height);
    
    cv::Mat left, right, leftRect, rightRect, disparity, depthVis;
    
    while (true) {
        capLeft >> left;
        capRight >> right;
        
        if (left.empty() || right.empty()) break;
        
        depthCalc.rectifyImages(left, right, leftRect, rightRect);
        disparity = depthCalc.computeDisparity(leftRect, rightRect);
        depthVis = depthCalc.visualizeDepth(disparity);
        
        cv::imshow("Левая камера", leftRect);
        cv::imshow("Правая камера", rightRect);
        cv::imshow("Карта глубины", depthVis);
        
        if (cv::waitKey(1) == 27) break;
    }
    
    capLeft.release();
    capRight.release();
    cv::destroyAllWindows();
}

void showMenu() {
    std::cout << "\n=================================" << std::endl;
    std::cout << "    СТЕРЕО ГЛУБИНА" << std::endl;
    std::cout << "=================================" << std::endl;
    std::cout << "1. Калибровка левой камеры (0)" << std::endl;
    std::cout << "2. Калибровка правой камеры (1)" << std::endl;
    std::cout << "3. Калибровка стереопары" << std::endl;
    std::cout << "4. Запуск видео глубины" << std::endl;
    std::cout << "5. Выход" << std::endl;
    std::cout << "Выбор: ";
}

