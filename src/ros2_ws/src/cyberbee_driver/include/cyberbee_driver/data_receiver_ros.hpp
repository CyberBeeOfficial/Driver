#ifndef DATA_RECEIVER_ROS_HPP
#define DATA_RECEIVER_ROS_HPP

#include "pose_message.hpp"
#include "serial_comm.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

class DataReceiverRos
{
   public:
    explicit DataReceiverRos(std::shared_ptr<rclcpp::Node> node, SerialPort* serialPort);
    void stop();
    void readData();
    void processData();
    void updateBaudRate(unsigned int newBaudRate);
    
    void ParseOdometryMsg(const std::vector<uint8_t>& message);
    void ParsePoseStampedMsg(const std::vector<uint8_t>& message);
    void ParseImuMsg(const std::vector<uint8_t>& message);
    void ParseStatusMsg(const std::vector<uint8_t>& message);
    void ParseErrorMsg(const std::vector<uint8_t>& message);

    float bigEndianToFloat(const std::vector<uint8_t>& data, size_t offset);
    uint64_t bigEndianToUint64(const std::vector<uint8_t>& data, size_t offset);
    uint16_t calculateChecksumBinary(const std::vector<uint8_t>& data);
    
    void constructOdometryMessage(const std::vector<uint8_t>& message);
    void constructPoseStampedMessage(const std::vector<uint8_t>& message);
    void constructImuMessage(const std::vector<uint8_t>& message);


   private:
    SerialPort* serialPort_;
    std::mutex mtx;  // For synchronizing access to a shared resource
    std::condition_variable cv;  // For signaling between threads
    bool stopThreads = false;    // Flag to control the stopping of threads

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr Pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr Imu_publisher_;
};

#endif  // DATA_RECEIVER_ROS_HPP