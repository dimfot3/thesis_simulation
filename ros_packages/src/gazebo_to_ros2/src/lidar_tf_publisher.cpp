#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <chrono>

void generate_lidar_tf(rclcpp::Node::SharedPtr node, geometry_msgs::msg::TransformStamped &static_transform, std::string lidar_name, std::vector<float> &pos, std::vector<float> &rot)
{
    static_transform.header.stamp = node->now();
    static_transform.header.frame_id = "world";
    static_transform.child_frame_id = lidar_name;
    static_transform.transform.translation.x = pos[0];
    static_transform.transform.translation.y = pos[1];
    static_transform.transform.translation.z = pos[2];
    tf2::Quaternion quat;
    quat.setRPY(rot[0], rot[1], rot[2]); // Set rotation in Roll-Pitch-Yaw
    static_transform.transform.rotation.x = quat.x();
    static_transform.transform.rotation.y = quat.y();
    static_transform.transform.rotation.z = quat.z();
    static_transform.transform.rotation.w = quat.w();
}

void parse_args(int argc, char **argv, std::vector<std::string> &lidar_names, std::vector<std::vector<float>> &pos, std::vector<std::vector<float>> &rot)
{
    int num_of_lidar = (argc - 1) / 7;
    for(int i = 0; i < num_of_lidar; i++)
    {
        lidar_names.push_back(argv[7*i + 1]);
        std::vector<float> pos1 = {std::stof(argv[7*i + 2]), std::stof(argv[7*i + 3]), std::stof(argv[7*i + 4])};
        std::vector<float> rot1 = {std::stof(argv[7*i + 5]), std::stof(argv[7*i + 6]), std::stof(argv[7*i + 7])};
        pos.push_back(pos1);
        rot.push_back(rot1);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("static_tf2_broadcaster");
    tf2_ros::StaticTransformBroadcaster static_broadcaster(node);
    int num_of_lidar = (argc - 1) / 7;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    std::vector<std::string> lidar_names;
    std::vector<std::vector<float>> pos;
    std::vector<std::vector<float>> rot;
    parse_args(argc, argv, lidar_names, pos, rot);
    std::vector<float> pos_world = {0, 0, 0};
    std::vector<float> rot_world = {0, 0, 0};
    // geometry_msgs::msg::TransformStamped static_transform;
    // std::string world_frame = "world";
    // generate_lidar_tf(node, static_transform, world_frame, pos_world, rot_world);
    // static_broadcaster.sendTransform(static_transform);
    for(int i = 0; i < num_of_lidar; i++)
    {
        geometry_msgs::msg::TransformStamped static_transform;
        transforms.push_back(static_transform);
        generate_lidar_tf(node, transforms[i], lidar_names[i], pos[i], rot[i]);
        static_broadcaster.sendTransform(transforms[i]);
    }
    rclcpp::spin(node);
    return 0;
}
