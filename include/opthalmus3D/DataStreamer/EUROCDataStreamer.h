#ifndef EUROC_DATA_STREAMER_H
#define EUROC_DATA_STREAMER_H

#include <string>
#include <vector>
#include <utility>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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
                
                bool getCurrentImage(Frame &frame);
                bool getNextImage(Frame &frame);                
            private:
                Frame _current_frame;

                csv::CSVReader *_reader_cam;
                csv::CSVReader::iterator _iter_cam;

                std::string _path_cam_data;

                double _current_epoch_timestamp;
        };

        class EUROCIMUStreamer {
            public:
                EUROCIMUStreamer(std::string path_imu_data);
                ~EUROCIMUStreamer();

                bool getCurrentIMUData(IMUData &imu_data);
                bool getNextIMUData(IMUData &imu_data);
            private:
                IMUData _current_imu_data;

                csv::CSVReader *_reader_imu;
                csv::CSVReader::iterator _iter_imu;

                std::string _path_imu_data;

                double _current_epoch_timestamp;
        };

        class EUROCGroundTruthStreamer {
            public:
                EUROCGroundTruthStreamer(std::string path_ground_truth_data);
                ~EUROCGroundTruthStreamer();

                bool getCurrentGroundTruthData(GroundTruthData &ground_truth_data);
                bool getNextGroundTruthData(GroundTruthData &ground_truth_data);
            private:
                GroundTruthData _current_ground_truth_data;

                csv::CSVReader *_reader_ground_truth;
                csv::CSVReader::iterator _iter_ground_truth;

                std::string _path_ground_truth_data;

                double _current_epoch_timestamp;
        };
        // cam_priority restricts access to IMUData that is timestamped beyond the current image streamed
        EUROCDataStreamer(std::string dataset_path, bool cam_priority = true);
        ~EUROCDataStreamer();

        // cam_priority restricts access to IMUData that is timestamped beyond the current image streamed
        void setEpochPriorityCamera(bool cam_priority);
        
        bool getCurrentImages(Frame *left_image, Frame *right_image);
        bool getCurrentGroundTruth(GroundTruthData *ground_truth);
        bool getNextImages(Frame *left_image, Frame *right_image);
        bool getNextGroundTruth(GroundTruthData *ground_truth);

        double getEpochLimit();
    private:       
        EUROCImageStreamer _image_streamers[2];
        EUROCIMUStreamer _imu_streamer;
        EUROCGroundTruthStreamer _ground_truth_streamer;

        std::string _path_dataset;

        bool _b_prioritize_cam;

        double _current_max_timestamp;    
}; 
#endif // EUROC_DATA_STREAMER_H