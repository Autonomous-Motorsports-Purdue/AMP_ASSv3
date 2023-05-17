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
    pcl::PointCloud<pointT> cloud;

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
    void cloudcb(const sensor_msgs::msg::PointCloud2::UniquePtr input)
    {
        // Get Parameters
        double resolution = get_parameter("resolution").as_double();
        double minx = get_parameter("minx").as_double();
        double maxx = get_parameter("maxx").as_double();
        double miny = get_parameter("miny").as_double();
        double maxy = get_parameter("maxy").as_double();

        using pointT = pcl::PointXYZ;
        using cloudT = pcl::PointCloud<pointT>;

        nav2_msgs::msg::Costmap::UniquePtr costmap(new nav2_msgs::msg::Costmap);
        costmap->metadata.resolution = resolution;
        costmap->metadata.size_x = (maxx - minx + 1) / resolution;
        costmap->metadata.size_y = (maxy - miny + 1) / resolution;
        costmap->header = input.header;
        costmap->data.resize(costmap->metadata.size_y * costmap->metadata.size_x);


        // PointCloud2 to PCL cloud
	    pcl::fromROSMsg(*input, &cloud);
        RCLCPP_INFO(get_logger(), "Pointcloud received of size %ld \n", cloud.size());

        for(size_t i = 0; i < cloud.size(); i++) {
            if (cloud[i].y < maxy && cloud[i].x < maxx && cloud[i].y > miny && cloud[i].x < minx) {
                size_t h = (cloud[i].y - miny) / resolution;
                size_t w = (cloud[i].x - minx) / resolution;
                costmap->data[h * costmap->metadata.size_y + w] += 1;
            }
        }


        // Publish
        cloud_pub->publish(std::move(costmap));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Segmentation>());
    rclcpp::shutdown();
    return 0;
}
