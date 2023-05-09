#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
using namespace std::chrono_literals;


class TF_publisher: public rclcpp::Node
{   

    public:
    TF_publisher(int argc, char** argv)
    : Node("frame_publisher")
    {
        // Initialize the transform broadcaster
        tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(this);
        this->num_of_lidar = (argc - 1) / 7;
        parse_args(argc, argv);
        for(int i = 0; i < num_of_lidar; i++)
        {
            static_transforms.push_back(geometry_msgs::msg::TransformStamped());
            generate_lidar_tf(static_transforms.back(), i);
        }
        timer_ = this->create_wall_timer(
        10ms, std::bind(&TF_publisher::publish_tf, this));
    }

    private:
    void publish_tf()
    {
        for(int i = 0; i < num_of_lidar; i++)
        {
            static_transforms[i].header.stamp = this->get_clock()->now();
            tf_broadcaster_->sendTransform(static_transforms[i]);
        }
    }

    void generate_lidar_tf(geometry_msgs::msg::TransformStamped &static_transform, int idx)
    {
        static_transform.header.frame_id = "world";
        static_transform.child_frame_id = lidar_names[idx];
        static_transform.transform.translation.x = pos_arr[idx][0];
        static_transform.transform.translation.y = pos_arr[idx][1];
        static_transform.transform.translation.z = pos_arr[idx][2];
        tf2::Quaternion quat;
        quat.setRPY(rot_arr[idx][0], rot_arr[idx][1], rot_arr[idx][2]); // Set rotation in Roll-Pitch-Yaw
        static_transform.transform.rotation.x = quat.x();
        static_transform.transform.rotation.y = quat.y();
        static_transform.transform.rotation.z = quat.z();
        static_transform.transform.rotation.w = quat.w();
    }

    void parse_args(int argc, char **argv)
    {
        for(int i = 0; i < this->num_of_lidar; i++)
        {
            this->lidar_names.push_back(argv[7*i + 1]);
            std::vector<float> pos1 = {std::stof(argv[7*i + 2]), std::stof(argv[7*i + 3]), std::stof(argv[7*i + 4])};
            std::vector<float> rot1 = {std::stof(argv[7*i + 5]), std::stof(argv[7*i + 6]), std::stof(argv[7*i + 7])};
            this->pos_arr.push_back(pos1);
            this->rot_arr.push_back(rot1);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    std::vector<float> pos_world = {0, 0, 0};
    std::vector<float> rot_world = {0, 0, 0};
    int num_of_lidar;
    std::vector<std::string> lidar_names;
    std::vector<std::vector<float>> pos_arr;
    std::vector<std::vector<float>> rot_arr;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TF_publisher>(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
