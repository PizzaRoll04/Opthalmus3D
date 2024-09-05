#include <iostream>
#include <filesystem>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "EUROCDataStreamer.h"

using namespace IMUDataIndex;
using namespace CamRowIndex;



EUROCDataStreamer::EUROCDataStreamer(std::string dataset_path, bool cam_priority)
{
    if (!std::filesystem::exists(dataset_path))
    {
        throw std::filesystem::filesystem_error("File does not exist", dataset_path, std::make_error_code(std::errc::no_such_file_or_directory));
    }
    _path_dataset = dataset_path;

    _image_streamers[0] = EUROCImageStreamer(dataset_path + "/cam0");
    _image_streamers[1] = EUROCImageStreamer(dataset_path + "/cam1");

    _imu_streamer = EUROCIMUStreamer(dataset_path + "/imu0");

    _ground_truth_streamer = EUROCGroundTruthStreamer(dataset_path + "/state_groundtruth_estimate0");

    setEpochPriorityCamera(cam_priority);
}

EUROCDataStreamer::~EUROCDataStreamer()
{

}


void EUROCDataStreamer::setEpochPriorityCamera(bool cam_priority)
{
    _b_prioritize_cam = cam_priority;
}

///////////////////////////////////////
// EUROCDataStreamer::EUROCImageStramer
///////////////////////////////////////

EUROCDataStreamer::EUROCImageStreamer::EUROCImageStreamer(std::string path_cam_data)
{
    _path_cam_data = path_cam_data;

    _reader_cam = new csv::CSVReader(path_cam_data + "/data.csv");
    _iter_cam = _reader_cam->begin();
}

EUROCDataStreamer::EUROCImageStreamer::~EUROCImageStreamer()
{
    delete(_reader_cam);
}

bool EUROCDataStreamer::EUROCImageStreamer::getCurrentImage(Frame &frame)
{   
    if (_current_epoch_timestamp == 0) {
        return false;
    }
    frame.time_stamp = _current_frame.time_stamp;
    frame.img = _current_frame.img;
    return true;
}

/**
 * @warning Do not modify the passed pointer, the members may be shared elsewhere. Copy cv::Mat types 
 * if you need to modify them.
 */
bool EUROCDataStreamer::EUROCImageStreamer::getNextImage(Frame &frame)
{
    if (_current_epoch_timestamp == 0) {
        _iter_cam = _reader_cam->begin();
    } else if (_iter_cam == _reader_cam->end()) {
        return false;
    }
    _iter_cam++;
    _current_frame.time_stamp = std::stod((*_iter_cam)[CamRowIndex::TIMESTAMP].get<>());
    _current_frame.img = cv::imread(_path_cam_data + "/" + (*_iter_cam)[CamRowIndex::IMG_NAME].get<>(), cv::IMREAD_GRAYSCALE);
    frame.time_stamp = _current_frame.time_stamp;
    frame.img = _current_frame.img;
    return true;
}

//////////////////////////////////////
// EUROCDataStreamer::EUROCIMUStreamer
//////////////////////////////////////

EUROCDataStreamer::EUROCIMUStreamer::EUROCIMUStreamer(std::string path_imu_data) 
{
    _path_imu_data = path_imu_data;

    _reader_imu = new csv::CSVReader(path_imu_data + "/data.csv");
    _iter_imu = _reader_imu->begin();
}

EUROCDataStreamer::EUROCIMUStreamer::~EUROCIMUStreamer()
{
    delete(_reader_imu);
}

bool EUROCDataStreamer::EUROCIMUStreamer::getCurrentIMUData(IMUData &imu_data)
{
    if (_current_epoch_timestamp == 0) {
        return false;
    }
    imu_data.time_stamp = _current_imu_data.time_stamp;
    imu_data.lin_acceleration = _current_imu_data.lin_acceleration;
    imu_data.ang_velocity = _current_imu_data.ang_velocity;
    return true;
}

bool EUROCDataStreamer::EUROCIMUStreamer::getNextIMUData(IMUData &imu_data)
{
    if (_current_epoch_timestamp == 0) {
        _iter_imu = _reader_imu->begin();
    } else if (_iter_imu == _reader_imu->end()) {
        return false;
    }
    _current_imu_data.time_stamp = std::stod((*_iter_imu)[IMUDataIndex::TIMESTAMP].get<>());
    // _current_imu_data.lin_acceleration
}

//////////////////////////////////////////////
// EUROCDataStreamer::EUROCGroundTruthStreamer
//////////////////////////////////////////////

EUROCDataStreamer::EUROCGroundTruthStreamer::EUROCGroundTruthStreamer(std::string path_ground_truth_data)
{
    _path_ground_truth_data = path_ground_truth_data;

    _reader_ground_truth = new csv::CSVReader(path_ground_truth_data + "/data.csv");
}

EUROCDataStreamer::EUROCGroundTruthStreamer::~EUROCGroundTruthStreamer()
{
    delete(_reader_ground_truth);
}

bool EUROCDataStreamer::getNextImages(Frame *left_frame, Frame *right_frame)
{
    if (_iter_cam_right->empty())
    {
        return false;
    }
    *right_frame = _right_frame;
    return true;
}

void EUROCDataStreamer::updateCamEpochs()
{
    if (_epoch_cam_right.timestamp != 0 &&
        std::fabs(std::stod((*_iter_cam_right)[CamRowIndex::TIMESTAMP].get<>()) - _epoch_cam_right.timestamp) < 1e-6)
    {
        _iter_cam_right++;
    }
    // for (auto field : *_iter_cam_right) {
    //     std::cout << field.get<>() << " test";
    // }
    // std::cout << std::endl;
    _epoch_cam_right.timestamp = std::stod((*_iter_cam_right)[CamRowIndex::TIMESTAMP].get<>());
    _epoch_cam_right.img_name = (*_iter_cam_right)[CamRowIndex::IMG_NAME].get<>();

    // std::cout << _epoch_cam_right.img_name << "ugh" << std::endl;
}

double EUROCDataStreamer::getEpochLimit()
{
    return _current_time_stamp_limit;
}

std::vector<double> EUROCDataStreamer::getCurrentGroundTruth()
{
    while (std::stod((*_iter_ground_truth)[IMUDataIndex::TIMESTAMP].get<>()) < _current_time_stamp_limit)
    {
        _iter_ground_truth;
    }
}
