#ifndef EUROC_DATA_STREAMER_H
#define EUROC_DATA_STREAMER_H

#include <string>
#include <vector>
#include <utility>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "DataStreamer.h"

namespace IMUDataIndex {
    enum IMUDataIndex {
        TIMESTAMP = 0,
        OMEGA_X = 1,
        OMEGA_Y = 2,
        OMEGA_Z = 3,
        ACCEL_X = 4,
        ACCEL_Y = 5,
        ACCEL_Z = 6
    };
};

namespace CamRowIndex {
    enum CamRowIndex {
        TIMESTAMP = 0,
        IMG_NAME = 1
    };
};

class EUROCDataStreamer : public DataStreamer {
    public:
        class EUROCImageStreamer {
            public:
                struct ImgEpoch  {
                    std::string img_name;
                    double timestamp;
                };

                EUROCImageStreamer(std::string path_cam_data);
                ~EUROCImageStreamer();
                
                bool getCurrentImage(Frame *frame);
                bool getNextImage(Frame *frame);                
            private:
                IMUData _current_imu_data;

                csv::CSVReader *_reader_cam;
                csv::CSVReader::iterator _iter_cam;

                std::string _path_cam_data;

                double _current_epoch_timestamp;
        };

        class EUROCIMUStreamer {
            public:
                EUROCIMUStreamer(std::string path_imu_data);
                ~EUROCIMUStreamer();

                bool getCurrentIMUData(IMUData *imu_data);
                bool getNextIMUData(IMUData *imu_data);
            private:
                Frame _current_frame;

                csv::CSVReader *_reader_cam;
                csv::CSVReader::iterator _iter_cam;

                std::string _path_imu_data;

                double _current_epoch_timestamp;
        };
        // cam_priority restricts access to IMUData that is timestamped beyond the current image streamed
        EUROCDataStreamer(std::string dataset_path, bool cam_priority = true);
        ~EUROCDataStreamer();

        // cam_priority restricts access to IMUData that is timestamped beyond the current image streamed
        void setEpochPriorityCamera(bool cam_priority);

        bool loadNextImage();
        bool getNextImage(Frame* left_frame, Frame* right_frame);
        // bool getLeftImage(Frame* left_frame);
        // bool getRightImage(Frame* right_frame);

        bool loadNextIMUData();
        bool getNextIMUData(IMUData* imu_data);

        std::vector<double> getGroundTruth();

        double getEpochLimit();
    private:        
        void updateCamEpochs();

        struct ImgEpoch {
            std::string img_name;
            double timestamp;
        };

        csv::CSVReader *_reader_ground_truth;
        csv::CSVReader::iterator _iter_ground_truth;

        csv::CSVReader *_reader_cam_left;
        csv::CSVReader *_reader_cam_right;

        csv::CSVReader::iterator _iter_cam_left;
        csv::CSVReader::iterator _iter_cam_right;

        std::string _path_dataset;
        std::string _path_imgs_left;
        std::string _path_imgs_right;

        double _current_time_stamp_limit;

        double _current_time_stamp_imu;
        ImgEpoch _epoch_cam_right;
        ImgEpoch _epoch_cam_left;

        Frame _right_frame;
        Frame _left_frame;

        bool _b_prioritize_cam;

        // IMU stuff
        csv::CSVReader *_reader_imu_data;

        csv::CSVReader::iterator _iter_imu;

        double _imu_epoch_timestamp;
        IMUData _imu_data;        
}; 
#endif // EUROC_DATA_STREAMER_H