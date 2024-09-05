#ifndef DATASTREAMER_H
#define DATASTREAMER_H

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "csv.hpp"

class DataStreamer {
    public:
        struct Frame {
            double time_stamp;
            cv::Mat img;
        };
        
        struct IMUData {
            double time_stamp;
            std::vector<float> ang_velocity;
            std::vector<float> lin_acceleration;
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