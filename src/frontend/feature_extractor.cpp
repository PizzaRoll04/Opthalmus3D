#include <iostream>
#include <utility>
#include <chrono>
#include <thread>

#include "feature_extractor.h"

FeatureExtractor::FeatureExtractor() {}

FeatureExtractor::~FeatureExtractor() {}

void FeatureExtractor::setParams(CornerHarrisParams params)
{
    _corner_params = params;
}

std::vector<cv::Point> FeatureExtractor::getCorners(const cv::Mat &input_img, cv::Mat &output_img, bool viz_feat)
{
    std::vector<cv::Point> corners;
    if (input_img.channels() > 1)
    {
        cv::cvtColor(input_img, output_img, cv::COLOR_BGR2GRAY);
    }
    else
    {
        // std::cout << "before rows: " << output_img.rows <<  std::endl;
        input_img.copyTo(output_img);
        // std::cout << "after rows: " << output_img.rows <<  std::endl;
    }
    // cv::imshow("Frame", output_img);
    // std::this_thread::sleep_for(std::chrono::seconds(5));

    cv::cornerHarris(output_img, output_img, _corner_params.block_size, _corner_params.aperature_size, _corner_params.k);
    cv::normalize(output_img, output_img, 0, 255, cv::NORM_MINMAX, CV_32FC1);
    cv::convertScaleAbs(output_img, output_img);

    for (int i = 0; i < output_img.rows; i++)
    {
        for (int j = 0; j < output_img.cols; j++)
        {
            if (static_cast<int>(output_img.at<uint8_t>(i, j)) > 150)
            {
                cv::Point point(j, i);
                corners.push_back(point);
                if (viz_feat)
                {
                    cv::circle(output_img, point, 5, cv::Scalar(0), 1);
                }
            }
        }
    }
    return corners;
}

std::vector<cv::KeyPoint> FeatureExtractor::extractFeaturesAndMatches(const cv::Mat &input_img, cv::Mat &output_img, bool viz_feat)
{
    cv::Mat descriptors;
    orb->detectAndCompute(input_img, cv::noArray(), keypoints, descriptors);

    // If we have previous descriptors, match them with the current descriptors
    if (!prevDescriptors.empty())
    {
        // Match descriptors between the previous and current frame
        _matches.clear();
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.match(prevDescriptors, descriptors, _matches);

        // Draw matches
        cv::Mat imgMatches;
        cv::drawMatches(prevFrame, prevKeypoints, input_img, keypoints, _matches, imgMatches);

        if (viz_feat)
        {
            // Show the matches
            cv::imshow("Matches", imgMatches);
        }

        // You can also process the matches here for VIO or trajectory estimation
        // For example, estimating a homography or essential matrix, or tracking features.
    }

    prevFrame = input_img.clone();
    prevKeypoints = keypoints;
    prevDescriptors = descriptors.clone();

    if (viz_feat)
    {
        cv::drawKeypoints(input_img, keypoints, output_img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    }

    return keypoints;
}

std::pair<std::vector<double>, std::vector<double>> FeatureExtractor::estimateMotion(std::vector<std::vector<double>> imu_data_queue)
{
    // Focal length and principal point (for an example camera)
    double focal = 800.0;
    cv::Point2d pp(prevFrame.cols / 2.0, prevFrame.rows / 2 / 0); // Assuming an image size of 1280x720

    std::sort(_matches.begin(), _matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
              { return a.distance < b.distance; });

    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;

    for (cv::DMatch match : _matches)
    {
        pts1.push_back(prevKeypoints[match.queryIdx].pt);
        pts2.push_back(keypoints[match.trainIdx].pt);
    }

    cv::Mat E = cv::findEssentialMat(pts1, pts2, focal, pp, cv::RANSAC, 0.999, 1.0);
    cv::Mat R, t;
    cv::recoverPose(E, pts1, pts2, R, t, focal, pp);

    std::vector<double> orientation_delta = {0., 0., 0.};
}