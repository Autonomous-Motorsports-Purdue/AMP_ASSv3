#include <iostream>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class Segmentation : public rclcpp::Node
{

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;

public:
    explicit Segmentation()
	    : Node("segmentation")
    {
        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>("~/input", 10,
                std::bind(&Segmentation::cloudcb, this, std::placeholders::_1));
        cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("~/output", 1);

        declare_parameter("threshold", 0.01);
        declare_parameter("negative", true);
    }

private:
    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::msg::PointCloud2::UniquePtr input)
    {
        // Get Parameters
        double threshold;
        bool negative;

        threshold = get_parameter("threshold").as_double();
        negative = get_parameter("negative").as_bool();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inlier_indexes(new pcl::PointIndices);
        pcl::PointCloud<pcl::PointXYZ>  inliers;

        // PointCloud2 to PCL cloud
	    pcl::fromROSMsg(*input, *cloud);
        RCLCPP_DEBUG(get_logger(), "Pointcloud received of size %ld \n", cloud->size());

        // Configure SAC
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(threshold);

        // Perform Segmentation
        seg.setInputCloud(cloud);
        seg.segment(*inlier_indexes, *coefficients);

        if (inlier_indexes->indices.size () == 0)
        {
            RCLCPP_ERROR(get_logger(), "Could not estimate a planar model for the given dataset\n");
            return;
        }

        // Indexes to points
        pcl::ExtractIndices<pcl::PointXYZ> extract_indicies;
        extract_indicies.setInputCloud(cloud);
        extract_indicies.setIndices(inlier_indexes);
        extract_indicies.setNegative(negative);
        extract_indicies.filter(inliers);

        // PCL cloud to PointCloud2
        sensor_msgs::msg::PointCloud2::UniquePtr publish_cloud(new sensor_msgs::msg::PointCloud2());
        pcl::toROSMsg(inliers, *publish_cloud);

        publish_cloud->header.stamp = input->header.stamp;
        publish_cloud->header.frame_id = input->header.frame_id;

        // Publish
        cloud_pub->publish(std::move(publish_cloud));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Segmentation>());
    rclcpp::shutdown();
    return 0;
}
