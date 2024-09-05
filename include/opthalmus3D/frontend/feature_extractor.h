#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <string>
#include <iostream>
#include <utility>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"

class FeatureExtractor {
    public:
        struct CornerHarrisParams {
            int block_size;
            int aperature_size;
            double k;
        };

        FeatureExtractor();
        ~FeatureExtractor();

        void setParams(CornerHarrisParams params);
        std::vector<cv::Point> getCorners(const cv::Mat &input_img, cv::Mat &output_img, bool viz_feat=false);

        std::vector<cv::KeyPoint> extractFeaturesAndMatches(const cv::Mat &input_img, cv::Mat &output_img, bool viz_feat=false);
        std::pair<std::vector<double>, std::vector<double>> estimateMotion(std::vector<std::vector<double>> imu_data_queue = {0., 0., 0.});
    private:
        CornerHarrisParams _corner_params;
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints;
        std::vector<cv::KeyPoint> prevKeypoints;
        cv::Mat prevDescriptors;
        cv::Mat prevFrame;
        std::vector<cv::DMatch> _matches;

        double _trajectory[3] = {0., 0., 0.};
        double _orientation[3] = {0., 0., 0.};


};

#endif // FEATURE_EXTRACTOR_Hs