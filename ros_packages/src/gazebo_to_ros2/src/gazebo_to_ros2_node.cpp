#include "rclcpp/rclcpp.hpp"
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <string>


sensor_msgs::msg::PointCloud2 convertPointCloudPackedToPointCloud2(const gz::msgs::PointCloudPacked& msg)
{
    sensor_msgs::msg::PointCloud2 pc2;
    rclcpp::Time now = rclcpp::Clock().now();
    pc2.header.stamp = now;
    pc2.header.frame_id = "lidar_frame";

    pc2.height = 1;
    pc2.width = msg.data().size() / msg.point_step();
    pc2.fields.resize(3);

    // Populate the fields
    pc2.fields[0].name = "x";
    pc2.fields[0].offset = 0;
    pc2.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2.fields[0].count = 1;
    pc2.fields[1].name = "y";
    pc2.fields[1].offset = 4;
    pc2.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2.fields[1].count = 1;
    pc2.fields[2].name = "z";
    pc2.fields[2].offset = 8;
    pc2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2.fields[2].count = 1;

    pc2.is_bigendian = false;
    pc2.point_step = msg.point_step();
    pc2.row_step = pc2.point_step * pc2.width;

    pc2.data.resize(msg.data().size());
    std::memcpy(pc2.data.data(), msg.data().data(), msg.data().size());

    pc2.is_dense = true;

    return pc2;
}

void lidar_cb(const gz::msgs::PointCloudPacked &msg, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher)
{
    sensor_msgs::msg::PointCloud2 pcl2_msg = convertPointCloudPackedToPointCloud2(msg);
    lidar_publisher->publish(pcl2_msg);
}

std::vector<std::string> get_lidar_topics(int argc, char **argv)
{
    std::vector<std::string> lidar_list;
    for(int i = 1; i < argc; i++)
    {
        lidar_list.push_back(argv[i]);
    }
    return lidar_list;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lidar_publisher_node");
    std::vector<std::string> lidar_topics = get_lidar_topics(argc, argv);
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_publisher;
    gz::transport::Node nodegz;
    for(int i = 0; i < lidar_topics.size(); i++)
    {
        lidar_publisher.push_back(node->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topics[i], 10));
        std::string topic_sub = "/points";
        topic_sub = lidar_topics[i] + topic_sub;
        std::function<void(const gz::msgs::PointCloudPacked &)> bound_lidar_callback = std::bind(lidar_cb, std::placeholders::_1, lidar_publisher[i]);
        nodegz.Subscribe(topic_sub, bound_lidar_callback);
    }
    rclcpp::spin(node);
    return 0;
}
