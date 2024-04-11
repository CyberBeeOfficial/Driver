#ifndef DATA_RECEIVER_ROS_HPP
#define DATA_RECEIVER_ROS_HPP

#include "pose_message.hpp"
#include "serial_comm.hpp"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <ros/ros.h>

class DataReceiverRos
{
   public:
    explicit DataReceiverRos(ros::NodeHandle* nodeHandle, SerialPort* serialPort);
    void stop();
    void readData();
    void processData();
    void updateBaudRate(unsigned int newBaudRate);

   private:
    SerialPort* serialPort_;
    std::mutex mtx;  // For synchronizing access to a shared resource
    std::condition_variable cv;  // For signaling between threads
    bool stopThreads = false;    // Flag to control the stopping of threads

    // std::shared_ptr<rclcpp::Node> node_;
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr Pose_publisher_;
    // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr Imu_publisher_;

    ros::NodeHandle* nodeHandle_;
    ros::Publisher odometry_publisher_;
    ros::Publisher pose_publisher_;
    ros::Publisher imu_publisher_;

    void ParseOdometryMsg(const std::vector<uint8_t>& message);
    void ParsePoseStampedMsg(const std::vector<uint8_t>& message);
    void ParseImuMsg(const std::vector<uint8_t>& message);
    void ParseStatusMsg(const std::vector<uint8_t>& message);
    void ParseErrorMsg(const std::vector<uint8_t>& message);

    float bigEndianToFloat(const std::vector<uint8_t>& data, size_t offset);
    uint64_t bigEndianToUint64(const std::vector<uint8_t>& data, size_t offset);
};

#endif  // DATA_RECEIVER_ROS_HPP