#ifndef DATA_RECEIVER_ROS_HPP
#define DATA_RECEIVER_ROS_HPP

#include "pose_message.hpp"
#include "serial_comm.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class DataReceiverRos
{
   public:
    explicit DataReceiverRos(SerialPort* serialPort);
    void stop();
    void readData();
    void processData();
    void updateBaudRate(unsigned int newBaudRate);
    uint16_t calculateCheckSum(std::array<std::string, 8> &sliced_msg_parts);
    std::array<std::string, 8> ParseAndPrintMsg(const std::string& message);
    void ParseAndPrintMsgBinary(const std::vector<uint8_t>& message);
   
    float bigEndianToFloat(const std::vector<uint8_t>& data, size_t offset);
    uint64_t bigEndianToUint64(const std::vector<uint8_t>& data, size_t offset);
    uint16_t calculateChecksum(const std::vector<uint8_t>& data);
    uint16_t calculateChecksumBinary(const std::vector<uint8_t>& data);
    void constructOdometryMessage(const std::vector<uint8_t>& message);

   private:
    SerialPort* serialPort_;
    std::mutex mtx;  // For synchronizing access to a shared resource
    std::condition_variable cv;  // For signaling between threads
    bool stopThreads = false;    // Flag to control the stopping of threads

    std::vector<float> parseCSVFloats(const std::string& csv);

    uint64_t messageCountBinary = 0; // To count the number of messages processed
    std::chrono::steady_clock::time_point startTime; // To mark the start time of processing messages
    bool isTimeMarked = false; // To ensure start time is marked once

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
};

#endif  // DATA_RECEIVER_ROS_HPP