//
// Created by xiang on 2021/10/9.
// Modified by Devansh Dhrafani on 2024/02/11.
//

#include <gflags/gflags.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unistd.h>
#include <csignal>

#include "laser_mapping.h"
#include "utils.h"

/// run faster-LIO in offline mode

DEFINE_string(config_file, "./src/faster-lio/config/velodyne_wildfire.yaml", "path to config file");
DEFINE_string(bag_file, "/media/devansh/t7shield/lidar_processing/1.bags/frick_1/run1.bag", "path to the ros bag");
DEFINE_string(save_dir, "./Log", "path to save directory");
DEFINE_string(time_log_file, "./Log/time.log", "path to time log file");
DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    const std::string bag_file = FLAGS_bag_file;
    const std::string config_file = FLAGS_config_file;

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    if (!laser_mapping->InitWithoutROS(FLAGS_config_file)) {
        LOG(ERROR) << "laser mapping init failed.";
        return -1;
    }

    /// handle ctrl-c
    signal(SIGINT, SigHandle);

    // just read the bag and send the data
    LOG(INFO) << "Opening rosbag, be patient";
    rosbag::Bag bag(FLAGS_bag_file, rosbag::bagmode::Read);

    LOG(INFO) << "Go!";
    for (const rosbag::MessageInstance &m : rosbag::View(bag)) {
        auto livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_msg) {
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &livox_msg]() {
                    laser_mapping->LivoxPCLCallBack(livox_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
            continue;
        }

        auto point_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (point_cloud_msg) {
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &point_cloud_msg]() {
                    laser_mapping->StandardPCLCallBack(point_cloud_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
            continue;
        }

        auto imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg) {
            laser_mapping->IMUCallBack(imu_msg);
            continue;
        }

        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
    }

    std::string save_dir = FLAGS_save_dir;
    if (save_dir.back() != '/') {
        save_dir += "/";
    }
    if (access(save_dir.c_str(), 0) == -1) {
        if (mkdir(save_dir.c_str(), 0777) == -1) {
            LOG(ERROR) << "create save directory failed.";
            return -1;
        }
    }

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish(save_dir);

    /// print the fps
    double fps = 1.0 / (faster_lio::Timer::GetMeanTime("Laser Mapping Single Run") / 1000.);
    LOG(INFO) << "Faster LIO average FPS: " << fps;

    std::string traj_log_file = save_dir + "traj.txt";
    LOG(INFO) << "save trajectory to: " << traj_log_file;
    laser_mapping->Savetrajectory(traj_log_file);

    faster_lio::Timer::PrintAll();

    std::string time_log_file = save_dir + "time.log";
    faster_lio::Timer::DumpIntoFile(time_log_file);

    return 0;
}
