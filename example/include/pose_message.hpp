#ifndef POSE_MESSAGE_HPP
#define POSE_MESSAGE_HPP
#include <iostream>
#include <sstream>  // For std::ostringstream
#include <string>
#include <tuple>
#include <vector>

class PoseMessage
{
   public:
    std::string message_type;
    std::string timestamp;
    std::vector<float> position;
    std::vector<float> quaternion;
    std::vector<float> velocity;

    PoseMessage(const std::string& mt, const std::string& ts,
                const std::vector<float>& pos, const std::vector<float>& quat,
                const std::vector<float>& vel)
        : message_type(mt),
          timestamp(ts),
          position(pos),
          quaternion(quat),
          velocity(vel)
    {
    }

    // TODO: maybe const required inside constructor

    std::string ToString() const
    {
        std::ostringstream oss;
        oss << "Type: " << message_type << ", Time: " << timestamp;

        oss << ", Position: (";
        for (size_t i = 0; i < position.size(); ++i)
        {
            oss << position[i];
            if (i < position.size() - 1)
                oss << ", ";
        }
        oss << ")";

        oss << ", Quaternion: (";
        for (size_t i = 0; i < quaternion.size(); ++i)
        {
            oss << quaternion[i];
            if (i < quaternion.size() - 1)
                oss << ", ";
        }
        oss << ")";

        oss << ", Velocity: (";
        for (size_t i = 0; i < velocity.size(); ++i)
        {
            oss << velocity[i];
            if (i < velocity.size() - 1)
                oss << ", ";
        }
        oss << ")";

        return oss.str();
    }
    ~PoseMessage() {}
};

#endif  // POSE_MESSAGE_H