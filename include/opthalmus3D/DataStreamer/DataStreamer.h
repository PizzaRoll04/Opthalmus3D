#ifndef DATASTREAMER_H
#define DATASTREAMER_H

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "csv.hpp"

class DataStreamer {
    public:
        struct Frame {
            double time_stamp;
            cv::Mat img;
        };
        
        struct IMUData {
            double time_stamp;
            Eigen::Vector3d ang_velocity;
            Eigen::Vector3d lin_acceleration;
        };

        struct GroundTruthData {
            double time_stamp;
            Eigen::Isometry3d transform_RS_R;
            Eigen::Vector3d velocity_RS_R;
            Eigen::Vector3d bias_gyroscope_RS_R;
            Eigen::Vector3d bias_accelerometer_RS_R;                    
        };

        DataStreamer();
        ~DataStreamer();

        virtual bool addIMUCallback(std::function<void(IMUData[])> callback);
        virtual bool addFrameCallback(std::function<void(Frame[])> callback);
        
        void enableCallbacks(bool enable);

    private:        
        virtual bool executeFrameCallbacks(Frame frames[]);
        virtual bool executeIMUCallbacks(IMUData imu_data[]);

        std::vector<std::function<void(IMUData[])>> _callbacks_imu;
        std::vector<std::function<void(Frame[])>> _callbacks_frame;

        bool _b_enable_callbacks = true;
}; 

#endif // DATASTREAMER_H