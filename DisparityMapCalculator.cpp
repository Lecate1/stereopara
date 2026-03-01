#include "DisparityCalculator.hpp"
#include <iostream>

DisparityCalculator::DisparityCalculator() {
    mapsReady = false;
    sgbm = cv::StereoSGBM::create();
    
    // Параметры по умолчанию
    sgbm->setNumDisparities(80);
    sgbm->setBlockSize(5);
    sgbm->setP1(8 * 3 * 5 * 5);
    sgbm->setP2(32 * 3 * 5 * 5);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
}

bool DisparityCalculator::initialize(const cv::Mat& cm1, const cv::Mat& dist1,
                                    const cv::Mat& cm2, const cv::Mat& dist2,
                                    const cv::Mat& R1, const cv::Mat& R2,
                                    const cv::Mat& P1, const cv::Mat& P2,
                                    const cv::Mat& Q, const cv::Size& size) {
    
    cv::initUndistortRectifyMap(cm1, dist1, R1, P1, size, CV_32FC1, mapLeft1, mapLeft2);
    cv::initUndistortRectifyMap(cm2, dist2, R2, P2, size, CV_32FC1, mapRight1, mapRight2);
    
    this->Q = Q.clone();
    mapsReady = true;
    return true;
}

void DisparityCalculator::rectifyImages(const cv::Mat& left, const cv::Mat& right,
                                       cv::Mat& leftRect, cv::Mat& rightRect) {
    if (!mapsReady) {
        leftRect = left.clone();
        rightRect = right.clone();
        return;
    }
    cv::remap(left, leftRect, mapLeft1, mapLeft2, cv::INTER_LINEAR);
    cv::remap(right, rightRect, mapRight1, mapRight2, cv::INTER_LINEAR);
}

cv::Mat DisparityCalculator::computeDisparity(const cv::Mat& left, const cv::Mat& right) {
    cv::Mat leftGray, rightGray;
    
    if (left.channels() == 3) 
        cv::cvtColor(left, leftGray, cv::COLOR_BGR2GRAY);
    else 
        leftGray = left;
    
    if (right.channels() == 3) 
        cv::cvtColor(right, rightGray, cv::COLOR_BGR2GRAY);
    else 
        rightGray = right;
    
    cv::Mat disparity;
    sgbm->compute(leftGray, rightGray, disparity);
    disparity.convertTo(disparity, CV_32F, 1.0 / 16.0);
    
    return disparity;
}

cv::Mat DisparityCalculator::visualizeDepth(const cv::Mat& disparity) {
    cv::Mat normalized;
    double minVal, maxVal;
    cv::minMaxLoc(disparity, &minVal, &maxVal);
    
    if (maxVal - minVal > 0) {
        disparity.convertTo(normalized, CV_8U, 
                            255.0 / (maxVal - minVal), 
                            -minVal * 255.0 / (maxVal - minVal));
    } else {
        normalized = cv::Mat::zeros(disparity.size(), CV_8U);
    }
    
    cv::bitwise_not(normalized, normalized);
    return normalized;
}

void DisparityCalculator::setParameters(int numDisparities, int blockSize, int uniquenessRatio) {
    sgbm->setNumDisparities(numDisparities);
    sgbm->setBlockSize(blockSize);
    sgbm->setP1(8 * 3 * blockSize * blockSize);
    sgbm->setP2(32 * 3 * blockSize * blockSize);
    sgbm->setUniquenessRatio(uniquenessRatio);
}
