#ifndef POSE_MESSAGE_HPP
#define POSE_MESSAGE_HPP
#include <string>
#include <iostream>
#include <sstream> // For std::ostringstream
#include <tuple>

class PoseMessage{
public:
    std::string message_type;
    std::string timestamp; 
    std::tuple<float, float, float> position;
    std::tuple<float, float, float, float> quaternion;
    std::tuple<float, float, float> velocity;

    PoseMessage(const std::string& mt,const std::string& ts, std::tuple<float, float, float> pos, std::tuple<float, float, float, float> quat, std::tuple<float, float, float> vel); 
      : message_type(mt), 
        timestamp(ts), 
        position(pos),
        quaternion(quat),
        velocity(vel)
    {}

    //TODO: maybe const required inside constructor 
    

    std::string ToString() const {
        std::ostringstream oss;
        oss << "Type: " << message_type << ", Time: " << timestamp
            << ", Position: (" << std::get<0>(position) << ", " << std::get<1>(position) << ", " << std::get<2>(position) << ")"
            << ", Quaternion: (" << std::get<0>(quaternion) << ", " << std::get<1>(quaternion) << ", " << std::get<2>(quaternion) << ", " << std::get<3>(quaternion) << ")"
            << ", Velocity: (" << std::get<0>(velocity) << ", " << std::get<1>(velocity) << ", " << std::get<2>(velocity) << ")";
        return oss.str();
    }
    ~PoseMessage(){}
};



#endif //POSE_MESSAGE_H