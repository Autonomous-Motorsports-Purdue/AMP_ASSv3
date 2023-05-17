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

struct Offsets
    {
    uint32_t x, y, z;
    };

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
Offsets determineOffsets(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) 
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
    return !(std::isnan(val));
    }

    inline bool validateFloats(double val)
    {
    return !(std::isnan(val));
    }

    bool validateFloatsAtPosition(
    sensor_msgs::msg::PointCloud2::_data_type::const_iterator position,
    const Offsets offsets) 
    {
        float x = *reinterpret_cast<const float *>(&*(position + offsets.x));
        float y = *reinterpret_cast<const float *>(&*(position + offsets.y));
        float z = *reinterpret_cast<const float *>(&*(position + offsets.z));

        return validateFloats(x) &&
                validateFloats(y) &&
                validateFloats(z);
    }

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


    // this function gets called every time new pcl data comes in
    void cloudcb(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
    {
        // Get Parameters
        double resolution = get_parameter("resolution").as_double();
        double minx = get_parameter("minx").as_double();
        double maxx = get_parameter("maxx").as_double();
        double miny = get_parameter("miny").as_double();
        double maxy = get_parameter("maxy").as_double();

        nav2_msgs::msg::Costmap costmap;
        costmap.metadata.resolution = resolution;
        costmap.metadata.size_x = (maxx - minx + 1) / resolution;
        costmap.metadata.size_y = (maxy - miny + 1) / resolution;
        costmap.header = cloud->header;
        costmap.data.resize(costmap.metadata.size_y * costmap.metadata.size_x);

        Offsets offsets = determineOffsets(cloud);
        size_t points_to_copy = 0;
        for (auto it = cloud->data.begin(); it < cloud->data.end(); it += cloud->point_step) {
            if (validateFloatsAtPosition(it, offsets)) {
                float x = *reinterpret_cast<const float *>(&*(it + offsets.x));
                float y = *reinterpret_cast<const float *>(&*(it + offsets.y));
                float z = *reinterpret_cast<const float *>(&*(it + offsets.z));

                if (points_to_copy % 20 == 0)
                        RCLCPP_INFO(get_logger(), "coord %f %f %f \n", x,y,z);

                if (y < maxy && x < maxx && y > miny && x > minx) {
                    double h = (y - miny) / resolution;
                    double w = (x - minx) / resolution;
                    if(costmap.data[(long)(w * costmap.metadata.size_x + h)] < 255)
                        costmap.data[(long)(w * costmap.metadata.size_x + h)] += 1;

                    RCLCPP_INFO(get_logger(), "anal %f %f %f %f \n", x, minx, resolution, w);
                    RCLCPP_INFO(get_logger(), "data %d", costmap.data[(long)(w * costmap.metadata.size_x + h)]);

                    // RCLCPP_INFO(get_logger(), "offsets %d %d %d \n", offsets.x,offsets.y,offsets.z);
                    
                }
                points_to_copy++;
            } 
        }
    
        RCLCPP_INFO(get_logger(), "Pointcloud received of size %ld \n", points_to_copy);
        RCLCPP_INFO(get_logger(), "map received of size %ld %ld \n", costmap.metadata.size_x, costmap.metadata.size_y);

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
