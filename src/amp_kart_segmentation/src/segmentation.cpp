#include <iostream>
#include <fstream>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <stdint.h>

class Segmentation : public rclcpp::Node
{

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr cloud_pub;

public:
    explicit Segmentation()
	    : Node("segmentation")
    {
        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>("~/input", 10,
                std::bind(&Segmentation::cloudcb, this, std::placeholders::_1));
        cloud_pub = create_publisher<nav2_msgs::msg::Costmap>("~/output", 1);

        declare_parameter("resolution", 0.05);
        declare_parameter("minx", -10.);
        declare_parameter("maxx", 10.);
        declare_parameter("miny", -10.);
        declare_parameter("maxy", 10.);
    }

private:
    inline int32_t findChannelIndex(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const std::string & channel)
    {
    for (size_t i = 0; i < cloud->fields.size(); ++i) {
        if (cloud->fields[i].name == channel) {
        return static_cast<uint32_t>(i);
        }
    }

    return -1;
    }

    Offsets PointCloud2Display::determineOffsets(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const
    {
        Offsets offsets{
            cloud->fields[findChannelIndex(cloud, "x")].offset,
            cloud->fields[findChannelIndex(cloud, "y")].offset,
            cloud->fields[findChannelIndex(cloud, "z")].offset
        };
        return offsets;
    }

    inline bool validateFloats(float val)
    {
    return !(std::isnan(val) || std::isinf(val));
    }

    inline bool validateFloats(double val)
    {
    return !(std::isnan(val) || std::isinf(val));
    }

    bool PointCloud2Display::validateFloatsAtPosition(
    sensor_msgs::msg::PointCloud2::_data_type::const_iterator position,
    const Offsets offsets) const
    {
        float x = *reinterpret_cast<const float *>(&*(position + offsets.x));
        float y = *reinterpret_cast<const float *>(&*(position + offsets.y));
        float z = *reinterpret_cast<const float *>(&*(position + offsets.z));

        return rviz_common::validateFloats(x) &&
                rviz_common::validateFloats(y) &&
                rviz_common::validateFloats(z);
    }

    // this function gets called every time new pcl data comes in
    void cloudcb(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud
    {
        // Get Parameters
        double resolution = get_parameter("resolution").as_double();
        double minx = get_parameter("minx").as_double();
        double maxx = get_parameter("maxx").as_double();
        double miny = get_parameter("miny").as_double();
        double maxy = get_parameter("maxy").as_double();

        sensor_msgs::msg::PointCloud2::_data_type filteredData;
        filteredData.reserve(cloud->data.size());

        Offsets offsets = determineOffsets(cloud);
        size_t points_to_copy = 0;
        sensor_msgs::msg::PointCloud2::_data_type::const_iterator copy_start_pos;
        for (auto it = cloud->data.begin(); it < cloud->data.end(); it += cloud->point_step) {
            if (validateFloatsAtPosition(it, offsets)) {
            if (points_to_copy == 0) {
                copy_start_pos = it;
            }
            ++points_to_copy;
            } else if (points_to_copy > 0) {
            filteredData.insert(
                filteredData.end(),
                copy_start_pos,
                copy_start_pos + points_to_copy * cloud->point_step);
            points_to_copy = 0;
            }
        }
        // Don't forget to flush what needs to be copied
        if (points_to_copy > 0) {
            filteredData.insert(
            filteredData.end(),
            copy_start_pos,
            copy_start_pos + points_to_copy * cloud->point_step);
        }

        return filteredData;




        nav2_msgs::msg::Costmap costmap;
        costmap.metadata.resolution = resolution;
        costmap.metadata.size_x = (maxx - minx + 1) / resolution;
        costmap.metadata.size_y = (maxy - miny + 1) / resolution;
        costmap.header = input->header;
        costmap.data.resize(costmap.metadata.size_y * costmap.metadata.size_x);

        // PointCloud2 to PCL cloud
	    pcl::fromROSMsg(*input, *cloud);
        RCLCPP_INFO(get_logger(), "Pointcloud received of size %ld \n", cloud->size());
        RCLCPP_INFO(get_logger(), "map received of size %ld %ld \n", costmap.metadata.size_x, costmap.metadata.size_y);

        for(long i = 0; i < (long) cloud->points.size(); i++) {
            RCLCPP_INFO(get_logger(), "coord %d %d %d \n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            if (cloud->points[i].y < maxy && cloud->points[i].z < maxx && cloud->points[i].y > miny && cloud->points[i].z > minx) {
                double h = (cloud->points[i].y - miny) / resolution;
                double w = (cloud->points[i].z - minx) / resolution;
                if(costmap.data[(long)(w * costmap.metadata.size_x + h)] < 255)
                    costmap.data[(long)(h * costmap.metadata.size_y + w)] += 1;
            }
        }

        // Publish
        cloud_pub->publish(costmap);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Segmentation>());
    rclcpp::shutdown();
    return 0;
}
