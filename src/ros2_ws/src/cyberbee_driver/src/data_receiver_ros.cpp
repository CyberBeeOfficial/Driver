/*
 * Copyright (c) 2024 CyberBee Ltd.
 * All rights reserved.
 *
 * This file is part of the CyberBee IMX8 C++ Driver.
 *
 * Description:
 * This code is part of the CyberBee  IMX8 C++ Driver layer, responsible for
 * managing communication between imx8 and other devices sucha s PI-4B.
 * This code meant to be compiled on the client device i.e. PI-4B,or other
 * ubuntu system.
 *
 */

#include "../include/cyberbee_driver/data_receiver_ros.hpp"

#include <iostream>
#include <sstream>

#include <thread>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <numeric>

#include <stdexcept> 
#include <string>
#include <tuple>
#include <vector>

// Define message types
enum MessageType {
    ODOMETRY = 0x01,
    POSITION = 0x02,
    IMU = 0x03,
    STATUS = 0x04,
    ERROR = 0x05
    // Add more message types as needed
};

DataReceiverRos::DataReceiverRos(std::shared_ptr<rclcpp::Node> node, SerialPort* serialPort) : serialPort_(serialPort), node_(node)
{
    odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("cyberbee/odometry", 10);
    Pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("cyberbee/pose", 10);
    Imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("cyberbee/imu/data_raw", 10);
}

void DataReceiverRos::readData()
{
    if (serialPort_ != nullptr)
    {
        while (!stopThreads)
        {
            serialPort_->ReadFromBuffer();
        }
    }
}

void DataReceiverRos::processData()
{
    while (!stopThreads)
    {
        std::vector<uint8_t> message = serialPort_->ExtractMessageBinary(); 

        uint16_t messageType = (static_cast<uint16_t>(message[0]) << 8) | message[1];

        if (!message.empty())
        {
            switch(messageType)
            {
                case ODOMETRY:
                    ParseOdometryMsg(message);
                    break;
                case POSITION:
                    ParsePoseStampedMsg(message);
                    break;
                case IMU:
                    ParseImuMsg(message);
                    break;
                case STATUS:
                    ParseStatusMsg(message);
                    break;
                case ERROR:
                    ParseErrorMsg(message);
                    break;
                default:
                    break;
            }
        }
    }
}

void DataReceiverRos::stop()
{
    stopThreads = true;
    serialPort_->NotifyDataAvailable();  // You might need to implement this
}

// ------------------------------- Parse functions ------------------------------------

void DataReceiverRos::ParseOdometryMsg(const std::vector<uint8_t>& message)
{
    if (message.size() < 52) { // Adjusted for the new message structure
        throw std::runtime_error("Incomplete message received.");
    }

    // Verifying and printing the 2-byte checksum
    uint16_t receivedChecksum = (static_cast<uint16_t>(message[message.size() - 2]) << 8) | message[message.size() - 1];
    uint16_t calculatedChecksum = calculateChecksumBinary(message);
    std::cout << "Calculated Checksum: " << calculatedChecksum 
              << " Received Checksum: " << receivedChecksum << std::endl;

    if (calculatedChecksum != receivedChecksum) {
        std::cerr << "Checksum mismatch." << std::endl;
    } else {
        std::cout << "Checksum verified successfully." << std::endl;
        constructOdometryMessage(message);
    }
}

void DataReceiverRos::ParsePoseStampedMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() < 40) { // Adjust based on your message structure
        throw std::runtime_error("Incomplete PoseStamped message received.");
    }

    // Checksum validation
    uint16_t receivedChecksum = (static_cast<uint16_t>(message[message.size() - 2]) << 8) | message[message.size() - 1];
    uint16_t calculatedChecksum = calculateChecksumBinary(message);
    std::cout << "Calculated Checksum: " << calculatedChecksum << " Received Checksum: " << receivedChecksum << std::endl;
    if (calculatedChecksum != receivedChecksum) {
        std::cerr << "Checksum mismatch." << std::endl;
    } else {
        std::cout << "Checksum verified successfully." << std::endl;
        constructPoseStampedMessage(message);
    }
}

void DataReceiverRos::ParseImuMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() < 52) { // Adjust based on the expected length
        throw std::runtime_error("Incomplete IMU message received.");
    }

    // Checksum validation
    uint16_t receivedChecksum = (static_cast<uint16_t>(message[message.size() - 2]) << 8) | message[message.size() - 1];
    uint16_t calculatedChecksum = calculateChecksumBinary(message);
    std::cout << "Calculated Checksum: " << calculatedChecksum << " Received Checksum: " << receivedChecksum << std::endl;
    if (calculatedChecksum != receivedChecksum) {
        std::cerr << "Checksum mismatch." << std::endl;
    } else {
        std::cout << "Checksum verified successfully." << std::endl;
        constructImuMessage(message);
    }
}

void DataReceiverRos::ParseStatusMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() != 20) { // Fixed size: 2 bytes type, 16 bytes data, 2 bytes checksum
        throw std::runtime_error("Incorrect status message size.");
    }

    // Extracting message type
    uint16_t messageType = (static_cast<uint16_t>(message[0]) << 8) | message[1];
    std::cout << "Message Type: " << messageType << std::endl;

    // Status information
    std::cout << "Status Information: ";
    for (size_t i = 2; i < 18; i++) { // Iterate through the 16 bytes of data
        std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(message[i]) << " ";
    }
    std::cout << std::dec << std::endl; // Switch back to decimal output

    // Checksum validation
    uint16_t receivedChecksum = (static_cast<uint16_t>(message[18]) << 8) | message[19];
    uint16_t calculatedChecksum = calculateChecksumBinary(message);
    std::cout << "Calculated Checksum: " << calculatedChecksum << " Received Checksum: " << receivedChecksum << std::endl;
    if (calculatedChecksum != receivedChecksum) {
        std::cerr << "Checksum mismatch." << std::endl;
    } else {
        std::cout << "Checksum verified successfully." << std::endl;
    }
}

void DataReceiverRos::ParseErrorMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() != 20) { // Fixed size: 2 bytes type, 16 bytes data, 2 bytes checksum
        throw std::runtime_error("Incorrect error message size.");
    }

    // Extracting message type
    uint16_t messageType = (static_cast<uint16_t>(message[0]) << 8) | message[1];
    std::cout << "Message Type: " << messageType << std::endl;

    // Error information
    std::cout << "Error Information: ";
    for (size_t i = 2; i < 18; i++) { // Iterate through the 16 bytes of data
        std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(message[i]) << " ";
    }
    std::cout << std::dec << std::endl; // Switch back to decimal output

    // Checksum validation
    uint16_t receivedChecksum = (static_cast<uint16_t>(message[18]) << 8) | message[19];
    uint16_t calculatedChecksum = calculateChecksumBinary(message);
    std::cout << "Calculated Checksum: " << calculatedChecksum << " Received Checksum: " << receivedChecksum << std::endl;
    if (calculatedChecksum != receivedChecksum) {
        std::cerr << "Checksum mismatch." << std::endl;
    } else {
        std::cout << "Checksum verified successfully." << std::endl;
    }
}

// ------------------------------- Helper functions ------------------------------------

// Helper function to convert from big endian to host endianess for floating-point numbers
float DataReceiverRos::bigEndianToFloat(const std::vector<uint8_t>& data, size_t offset) {
    uint32_t temp = 0;
    std::memcpy(&temp, &data[offset], sizeof(temp));
    temp = ntohl(temp); // Assumes your system has ntohl. Otherwise, implement a similar function
    float result;
    std::memcpy(&result, &temp, sizeof(result));
    return result;
}

// Helper function to convert from big endian to host endianess for 64-bit integers
uint64_t DataReceiverRos::bigEndianToUint64(const std::vector<uint8_t>& data, size_t offset) {
    uint64_t result = 0;
    for (int i = 0; i < 8; ++i) {
        result = (result << 8) | data[offset + i];
    }
    return result;
}

// Helper function for 16-bit checksum calculation
uint16_t DataReceiverRos::calculateChecksumBinary(const std::vector<uint8_t>& data) {
    uint32_t sum = std::accumulate(data.begin(), data.end() - 2, 0u); // Include all bytes for the sum except the last two checksum bytes
    // No need to convert sum to big endian, simply truncate to 16 bits
    uint16_t checksum = static_cast<uint16_t>(sum & 0xFFFF);
    return checksum; // Return the checksum without converting it to big endian
}

// ------------------------------- construct functions ------------------------------------

void DataReceiverRos::constructOdometryMessage(const std::vector<uint8_t>& message) {

    // Instantiate an Odometry message
    auto odometryMsg = std::make_shared<nav_msgs::msg::Odometry>();

    // Set the frame ID and child frame ID as per your configuration
    odometryMsg->header.frame_id = "odom";
    odometryMsg->child_frame_id = "base_link";

    // Convert and set the timestamp
    uint64_t timestamp = bigEndianToUint64(message, 2);
    rclcpp::Time rosTimestamp(timestamp);
    odometryMsg->header.stamp = rosTimestamp;

    // Extract, convert, and set position data
    size_t offset = 10;
    odometryMsg->pose.pose.position.x = bigEndianToFloat(message, offset);
    odometryMsg->pose.pose.position.y = bigEndianToFloat(message, offset + 4);
    odometryMsg->pose.pose.position.z = bigEndianToFloat(message, offset + 8);

    // Extract, convert, and set quaternion data
    offset += 12;
    odometryMsg->pose.pose.orientation.w = bigEndianToFloat(message, offset);
    odometryMsg->pose.pose.orientation.x = bigEndianToFloat(message, offset + 4);
    odometryMsg->pose.pose.orientation.y = bigEndianToFloat(message, offset + 8);
    odometryMsg->pose.pose.orientation.z = bigEndianToFloat(message, offset + 12);

    // Extract, convert, and set velocity data
    offset += 16;
    odometryMsg->twist.twist.linear.x = bigEndianToFloat(message, offset);
    odometryMsg->twist.twist.linear.y = bigEndianToFloat(message, offset + 4);
    odometryMsg->twist.twist.linear.z = bigEndianToFloat(message, offset + 8);

    // Additional fields like angular velocity can be set similarly if needed
    if(rclcpp::ok())
    {
        odometry_publisher_->publish(*odometryMsg);
    }
}

void DataReceiverRos::constructPoseStampedMessage(const std::vector<uint8_t>& message) 
{
    auto poseStampedMsg = std::make_shared<geometry_msgs::msg::PoseStamped>();

    // Set the frame ID as per your configuration
    poseStampedMsg->header.frame_id = "map"; // Adjust according to your TF frames

    // Convert and set the timestamp
    uint64_t timestamp = bigEndianToUint64(message, 2);
    rclcpp::Time rosTimestamp(timestamp);
    poseStampedMsg->header.stamp = rosTimestamp;

    // Pose position
    size_t offset = 10; // Starting byte for position data
    poseStampedMsg->pose.position.x = bigEndianToFloat(message, offset);
    poseStampedMsg->pose.position.y = bigEndianToFloat(message, offset + 4);
    poseStampedMsg->pose.position.z = bigEndianToFloat(message, offset + 8);

    // Pose orientation
    offset += 12; // Starting byte for orientation data
    poseStampedMsg->pose.orientation.w = bigEndianToFloat(message, offset);
    poseStampedMsg->pose.orientation.x = bigEndianToFloat(message, offset + 4);
    poseStampedMsg->pose.orientation.y = bigEndianToFloat(message, offset + 8);
    poseStampedMsg->pose.orientation.z = bigEndianToFloat(message, offset + 12);

    // Publish the message
    if(rclcpp::ok()) {
        Pose_publisher_->publish(*poseStampedMsg);
    }
}

void DataReceiverRos::constructImuMessage(const std::vector<uint8_t>& message) 
{
    auto imuMsg = std::make_shared<sensor_msgs::msg::Imu>();

    // Set the frame ID as per your configuration
    imuMsg->header.frame_id = "imu_link"; // Adjust according to your TF frames

    // Convert and set the timestamp
    uint64_t timestamp = bigEndianToUint64(message, 2);
    rclcpp::Time rosTimestamp(timestamp);
    imuMsg->header.stamp = rosTimestamp;

    // Orientation
    size_t offset = 10; // Adjust based on your actual message structure
    imuMsg->orientation.w = bigEndianToFloat(message, offset);
    imuMsg->orientation.x = bigEndianToFloat(message, offset + 4);
    imuMsg->orientation.y = bigEndianToFloat(message, offset + 8);
    imuMsg->orientation.z = bigEndianToFloat(message, offset + 12);

    // Angular velocity (assuming this follows immediately after orientation)
    offset += 16;
    imuMsg->angular_velocity.x = bigEndianToFloat(message, offset);
    imuMsg->angular_velocity.y = bigEndianToFloat(message, offset + 4);
    imuMsg->angular_velocity.z = bigEndianToFloat(message, offset + 8);

    // Linear acceleration (assuming this follows immediately after angular velocity)
    offset += 12;
    imuMsg->linear_acceleration.x = bigEndianToFloat(message, offset);
    imuMsg->linear_acceleration.y = bigEndianToFloat(message, offset + 4);
    imuMsg->linear_acceleration.z = bigEndianToFloat(message, offset + 8);

    // Publish the message
    if(rclcpp::ok()) {
        Imu_publisher_->publish(*imuMsg);
    }
}
