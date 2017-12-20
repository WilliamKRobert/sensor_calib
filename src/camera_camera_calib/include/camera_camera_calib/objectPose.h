#ifndef OBJECTPOSE_H
#define OBJECTPOSE_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "loadSettings.h"

void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners,
                  Settings::Pattern patternType);
bool findBoardCorner(cv::Mat gray, cv::Size patternsize, std::vector<Point2f>& corners, bool drawcorner);
void featureMatching(const cv::Mat& img_1, const cv::Mat& img_2, std::vector<cv::KeyPoint> &kp1, std::vector<cv::KeyPoint> &kp2);


#endif