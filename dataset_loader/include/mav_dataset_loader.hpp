#ifndef MAV_DATASET_LOADER_HPP_
#define MAV_DATASET_LOADER_HPP_

#include <iomanip>
#include <iostream>
#include <fstream>
#include <map>
#include <string>

using namespace std;

struct IMUData {
  double ang_x, ang_y, ang_z;
  double acc_x, acc_y, acc_z;
};

class MavDatasetLoader {
 public:
  // In ns.
  typedef int64_t TimeStamp;
  explicit MavDatasetLoader(const std::string &dataset_path);
  MavDatasetLoader() = delete;

  bool IsDatasetValid();

 private:
  bool LoadCalibration();
  bool LoadCameraImages() {
    ifstream camera_file(dataset_path_ + "cam0.csv");
    if (camera_file.is_open()) {
      std::string line;
      // Skip first line
      getline(camera_file, line);
      while (getline(camera_file, line)) {
        const size_t split_pos = line.find_first_of(',');
        const std::string timestamp_str = line.substr(0, split_pos);
        const std::string image_name = line.substr(split_pos + 1, string::npos);
        TimeStamp time_stamp = std::stoll(timestamp_str, nullptr, 10);
        timestamp_to_image_name_[time_stamp] = image_name;
      }
    }
    camera_file.close();
    return true;
  }

  bool LoadIMUData() {
    ifstream imu_data_file(dataset_path_ + "imu.csv");
    if (imu_data_file.is_open()) {
      std::string line;
      // Skip first line
      getline(imu_data_file, line);
      while (getline(imu_data_file, line)) {
        size_t cur_pos = 0;
        // Time stamp.
        TimeStamp time_stamp = std::stoll(line, &cur_pos, 10);
        IMUData new_imu_data;
        // w_RS_S_x [rad s^-1]
        line = line.substr(cur_pos + 1);
        new_imu_data.ang_x = std::stod(line, &cur_pos);
        // w_RS_S_y [rad s^-1]
        line = line.substr(cur_pos + 1);
        new_imu_data.ang_y = std::stod(line, &cur_pos);
        // w_RS_S_z [rad s^-1]
        line = line.substr(cur_pos + 1);
        new_imu_data.ang_z = std::stod(line, &cur_pos);
        // a_RS_S_x [m s^-2]
        line = line.substr(cur_pos + 1);
        new_imu_data.acc_x = std::stod(line, &cur_pos);
        // a_RS_S_y [m s^-2]
        line = line.substr(cur_pos + 1);
        new_imu_data.acc_y = std::stod(line, &cur_pos);
        // a_RS_S_z [m s^-2]
        line = line.substr(cur_pos + 1);
        new_imu_data.acc_z = std::stod(line, &cur_pos);

        timestamp_to_imu_data_[time_stamp] = new_imu_data;
      }
    }
    return true;
  }

  std::map<TimeStamp, std::string> timestamp_to_image_name_;
  std::map<TimeStamp, IMUData> timestamp_to_imu_data_;

  std::string dataset_path_;
};

#endif  // MAV_DATASET_LOADER_HPP_
