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

    _reader_cam_left = new csv::CSVReader(dataset_path + "/cam0/data.csv");
    _reader_cam_right = new csv::CSVReader(dataset_path + "/cam1/data.csv");
    _reader_imu_data = new csv::CSVReader(dataset_path + "/imu0/data.csv");
    _reader_ground_truth = new csv::CSVReader(dataset_path + "/mav0/state_groundtruth_estimate0/data.csv");

    _iter_cam_left = _reader_cam_left->begin();
    _iter_cam_right = _reader_cam_right->begin();
    _iter_imu = _reader_imu_data->begin();
    _iter_ground_truth = _reader_ground_truth->begin();

    _path_imgs_left = _path_dataset + "/cam0/data";
    _path_imgs_right = _path_dataset + "/cam1/data";

    setEpochPriorityCamera(cam_priority);
}

EUROCDataStreamer::~EUROCDataStreamer()
{
    delete (_reader_cam_left);
    delete (_reader_cam_right);
    delete (_reader_imu_data);
}

void EUROCDataStreamer::setEpochPriorityCamera(bool cam_priority)
{
    _b_prioritize_cam = cam_priority;
}

bool EUROCDataStreamer::loadNextImage()
{
    if (_iter_cam_right == _reader_cam_right->end())
    {
        return false;
    }
    if (_b_prioritize_cam)
    {
        updateCamEpochs();
        if (_iter_cam_right->empty())
        {
            return false;
        }
        _right_frame.time_stamp = _epoch_cam_right.timestamp;
        // std::string img_dir = _path_imgs_right + "/" + _epoch_cam_right.img_name;
        // std::cout << img_dir << std::endl;
        _right_frame.img = cv::imread(_path_imgs_right + "/" + _epoch_cam_right.img_name, cv::IMREAD_GRAYSCALE);
        return true;
    }
    return false;
}

bool EUROCDataStreamer::loadNextIMUData()
{
    if (_iter_imu == _reader_imu_data->end() || _iter_imu->empty())
    {
        return false;
    }
    if (!_b_prioritize_cam && std::fabs(_imu_data.time_stamp - _epoch_cam_right.timestamp) < 1e-6)
    {
        _iter_imu++;
    }

    _imu_data.ang_velocity.clear();
    _imu_data.lin_acceleration.clear();

    _imu_data.time_stamp = std::stod((*_iter_imu)[IMUDataIndex::TIMESTAMP].get<>());

    _imu_data.ang_velocity.push_back(std::stod((*_iter_imu)[IMUDataIndex::OMEGA_X].get<>()));
    _imu_data.ang_velocity.push_back(std::stod((*_iter_imu)[IMUDataIndex::OMEGA_Y].get<>()));
    _imu_data.ang_velocity.push_back(std::stod((*_iter_imu)[IMUDataIndex::OMEGA_Z].get<>()));
    _imu_data.lin_acceleration.push_back(std::stod((*_iter_imu)[IMUDataIndex::ACCEL_X].get<>()));
    _imu_data.lin_acceleration.push_back(std::stod((*_iter_imu)[IMUDataIndex::ACCEL_Y].get<>()));
    _imu_data.lin_acceleration.push_back(std::stod((*_iter_imu)[IMUDataIndex::ACCEL_Z].get<>()));
    return true;
}

bool EUROCDataStreamer::getNextIMUData(IMUData *imu_data)
{
    if (_iter_imu->empty())
    {
        return false;
    }
    *imu_data = _imu_data;
    return true;
}

bool EUROCDataStreamer::getNextImage(Frame *left_frame, Frame *right_frame)
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

std::vector<double> EUROCDataStreamer::getGroundTruth()
{
    while (std::stod((*_iter_ground_truth)[IMUDataIndex::TIMESTAMP].get<>()) < _current_time_stamp_limit)
    {
        _iter_ground_truth;
    }
}
