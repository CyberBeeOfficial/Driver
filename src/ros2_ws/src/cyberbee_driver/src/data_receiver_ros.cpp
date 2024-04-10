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
    std::vector<std::uint8_t> message;
    while (!stopThreads)
    {
        // This part needs to check the message and only than parse it in a different function
        std::vector<std::vector<uint8_t>> messages = serialPort_->ExtractMessageBinary();  // Assuming this method now just 

        for(auto msg = messages.begin(); msg != messages.end(); msg++)
        {
            message.assign((*msg).begin() + 1, (*msg).end() - 3); // without start byte and end byte

            uint16_t messageType = (static_cast<uint16_t>(message[0]) << 8) | message[1];

            if (!message.empty())
            {
                switch(messageType)
                {
                    case ODOMETRY:
                        std::cout << "Odometry message" << std::endl;
                        ParseOdometryMsg(message);
                        break;
                    case POSITION:
                        std::cout << "PoseStamped message" << std::endl;
                        ParsePoseStampedMsg(message);
                        break;
                    case IMU:
                        std::cout << "Imu message" << std::endl;
                        ParseImuMsg(message);
                        break;
                    case STATUS:
                        std::cout << "Status message" << std::endl;
                        ParseStatusMsg(message);
                        break;
                    case ERROR:
                        std::cout << "Error message" << std::endl;
                        ParseErrorMsg(message);
                        break;
                    default:
                        break;
                }
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
    if (message.size() < 50) // Adjusted for the new message structure
    { 
        throw std::runtime_error("Incomplete message received.");
    }
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

void DataReceiverRos::ParsePoseStampedMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() < 38) // Adjust based on your message structure
    { 
        throw std::runtime_error("Incomplete PoseStamped message received.");
    }
    
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

void DataReceiverRos::ParseImuMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() < 50) // Adjust based on the expected length
    { 
        throw std::runtime_error("Incomplete IMU message received.");
    }
    
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

void DataReceiverRos::ParseStatusMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() != 18) { // Fixed size: 2 bytes type, 16 bytes data, 2 bytes checksum
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
}

void DataReceiverRos::ParseErrorMsg(const std::vector<uint8_t>& message) 
{
    if (message.size() != 18) { // Fixed size: 2 bytes type, 16 bytes data, 2 bytes checksum
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
}

// ------------------------------- Helper functions ------------------------------------

// Helper function to convert from big endian to host endianess for floating-point numbers
float DataReceiverRos::bigEndianToFloat(const std::vector<uint8_t>& data, size_t offset) 
{
    uint32_t temp = 0;
    std::memcpy(&temp, &data[offset], sizeof(temp));
    temp = ntohl(temp); // Assumes your system has ntohl. Otherwise, implement a similar function
    float result;
    std::memcpy(&result, &temp, sizeof(result));
    return result;
}

// Helper function to convert from big endian to host endianess for 64-bit integers
uint64_t DataReceiverRos::bigEndianToUint64(const std::vector<uint8_t>& data, size_t offset) 
{
    uint64_t result = 0;
    for (int i = 0; i < 8; ++i) 
    {
        result = (result << 8) | data[offset + i];
    }
    return result;
}



