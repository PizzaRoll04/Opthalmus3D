#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

class PoseEstimator 
{
    public:
        PoseEstimator();

        void estimatePose();
    private:
};

#endif // POSE_ESTIMATOR_H