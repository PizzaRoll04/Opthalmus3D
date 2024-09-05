#include <string>
#include <iostream>
#include <chrono>
#include <functional>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>

#include "EUROCDataStreamer.h"
#include "feature_extractor.h"

EUROCDataStreamer streamer = EUROCDataStreamer("/home/pizzaroll04/Documents/DatasetsVIO/MH_01_easy/mav0", true);
EUROCDataStreamer::Frame *img = new EUROCDataStreamer::Frame;
FeatureExtractor feature_extractor;
int k = static_cast<float>(std::round((0.02f - 0.02f) * 2500));
const int max_int_val = 100;
cv::Mat corny;
double position[] = {0., 0., 0.};
double velocity[] = {0., 0., 0.};
double orientation[] = {0., 0., 0.};
std::vector<std::pair<double, EUROCDataStreamer::IMUData>> imu_queue;

EUROCDataStreamer::IMUData prev_imu;
EUROCDataStreamer::IMUData imu_data;

std::vector<cv::KeyPoint> prev_keypoints;
std::vector<cv::KeyPoint> keypoints;

// void trackbar_cb(int, void*) {
//     std::cout << "k: " << 0.02 + k/2500.0 << std::endl;
//     feature_extractor.setParams(FeatureExtractor::CornerHarrisParams{
//         2, // block_size
//         3, // aperature_size
//         0.02 + k*0.04 // kkey
//     });
//     feature_extractor.getCorners(img->img, corny, true);
//     cv::imshow("Frame", corny);
// }

int main() {

    feature_extractor.setParams(FeatureExtractor::CornerHarrisParams{
        2, // block_size
        3, // aperature_size
        0.02 + k*0.04 // k
    });
    auto start = std::chrono::high_resolution_clock::now();
    // streamer.loadNextImage();
    // streamer.getNextImage(nullptr, img);
    // std::cout << "channels: " << img->img.channels() << std::endl;
    // cv::goodFeaturesToTrack();
    // // std::cout << img->img.rows << std::endl;
    cv::namedWindow("Frame", cv::WINDOW_AUTOSIZE);
    // cv::createTrackbar("k", "Frame", &k, max_int_val, trackbar_cb);
    while (true) {                ;
        // if (!streamer.loadNextImage() || !streamer.getNextImage(nullptr, img)) {
        //     break;KeyPoint
        if (cv::waitKey(20) == 32) {
            // Spacebar was pressed
            if (!streamer.loadNextImage() || !streamer.getNextImage(nullptr, img)) {
                break;
            }        
            // std::vector<std::vector<double>> imu_data_queue;
            // while (streamer.loadNextIMUData() && streamer.getNextIMUData(&imu_data))
            // {   
            //     double dt = imu_data.time_stamp - prev_imu.time_stamp;
            //     imu_data_queue.push_back(imu_data.);
            //     // orientation[0] += prev_imu.ang_velocity[0] * dt;
            //     // orientation[1] += prev_imu.ang_velocity[1] * dt;
            //     // orientation[2] += prev_imu.ang_velocity[2] * dt;

            //     // velocity[0] += prev_imu.lin_acceleration[0] * dt;
            //     // velocity[1] += prev_imu.lin_acceleration[1] * dt;
            //     // velocity[2] += prev_imu.lin_acceleration[2] * dt;

            //     // position[0] += velocity[0] * dt;
            //     // position[1] += velocity[1] * dt;
            //     // position[2] += velocity[2] * dt;

            //     prev_imu = imu_data;
            // }
            keypoints = feature_extractor.extractFeaturesAndMatches(img->img, corny, true);
            std::pair<std::vector<double>, std::vector<double>> delta_pos_rot = feature_extractor.estimateMotion();
            position[0] += delta_pos_rot.first[0];
            position[1] += delta_pos_rot.first[1];
            position[2] += delta_pos_rot.first[2];

            orientation[0] += delta_pos_rot.second[0];
            orientation[1] += delta_pos_rot.second[1];
            orientation[2] += delta_pos_rot.second[2];

            std::cout << "Position: {" << position << std::endl;

            cv::imshow("Frame", corny);
        }

        if (corny.rows > 0 && corny.cols > 0) {
            cv::imshow("Frame", corny);
        } else {
            std::cout << "fuck" << std::endl;
        }     
    }
    std::cout << "done. took: " 
        << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count()/1000000.0f << "seconds" << std::endl;
    cv::waitKey(0);
    delete(img);
}