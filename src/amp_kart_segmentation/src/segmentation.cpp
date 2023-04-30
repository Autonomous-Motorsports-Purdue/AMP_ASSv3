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
        declare_parameter("voxel_size", 0.1);
        declare_parameter("minx", -10.);
        declare_parameter("maxx", 10.);
        declare_parameter("miny", -10.);
        declare_parameter("maxy", 10.);
        declare_parameter("minz", -3.);
        declare_parameter("maxz", 3.);
        declare_parameter("ring_gap", 10.);
    }

private:
    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::msg::PointCloud2::UniquePtr input)
    {
        // Get Parameters
        double threshold = get_parameter("threshold").as_double();
        bool negative = get_parameter("negative").as_bool();
        float voxel_size = (float) get_parameter("voxel_size").as_double();
        double minx = get_parameter("minx").as_double();
        double maxx = get_parameter("maxx").as_double();
        double miny = get_parameter("miny").as_double();
        double maxy = get_parameter("maxy").as_double();
        double minz = get_parameter("minz").as_double();
        double maxz = get_parameter("maxz").as_double();
        double ring_gap = get_parameter("ring_gap").as_double();

        using pointT = pcl::PointXYZ;
        using cloudT = pcl::PointCloud<pointT>;

        cloudT cloud;
        cloudT cropped;
        cloudT voxelized;
        cloudT inliers;

        // PointCloud2 to PCL cloud
	    pcl::fromROSMsg(*input, cloud);
        RCLCPP_DEBUG(get_logger(), "Pointcloud received of size %ld \n", cloud.size());

        // Crop
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        const cloudT::Ptr cloud_const(new cloudT(cloud));
        boxFilter.setMin(Eigen::Vector4f(minx, miny, minz, 1.0));
        boxFilter.setMax(Eigen::Vector4f(maxx, maxy, maxz, 1.0));
        boxFilter.setInputCloud(cloud_const);
        boxFilter.filter(cropped);

        // Voxels for stability
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        const cloudT::Ptr cropped_const(new cloudT(cropped));
        vox.setInputCloud(cropped_const);
        vox.setLeafSize(voxel_size, voxel_size, voxel_size);
        vox.filter(voxelized);

        // Configure SAC
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(threshold);

        // Configure ExtractIndices
        pcl::ExtractIndices<pointT> extract_indicies;

        // Precompute radiuses
        std::vector<double> radii;
        radii.resize(voxelized.size());
        double max_radius = 0;
        for(size_t i = 0; i < voxelized.size(); i++) {
            const pointT &point = voxelized[i];
            radii[i] = std::hypot(point.x, point.y);
            if (radii[i] > max_radius)
                max_radius = radii[i];
        }

        // Precompute thetas
        std::vector<double> theta;
        theta.resize(voxelized.size());
        for (size_t i = 0; i < voxelized.size(); i++) {
            const pointT &point = voxelized[i];
            theta[i] = std::atan2(point.x, point.y) + 3.141592;
        }

        RCLCPP_DEBUG(get_logger(), "Voxelized pointcloud size: %ld \n", voxelized.size());

        // Sort into sections
        std::vector<std::vector<cloudT::Ptr>> sections;
        for (size_t i = 0; i < voxelized.size(); i++) {
            size_t ring = (int) radii[i] / ring_gap;
            double ring_circ = 3.141592 * 2 * (ring * ring_gap);
            int sectors_in_ring = ring_circ / ring_gap;
            double sector_degs = 2 * 3.141592 / sectors_in_ring;
            size_t sector = theta[i] / sector_degs;

            if (sections.size() < ring + 1) sections.resize(ring + 1);
            if (sections[ring].size() < sector + 1) sections[ring].resize(sector + 1);

            if (!sections[ring][sector]) sections[ring][sector].reset(new cloudT);
            sections[ring][sector]->push_back(voxelized[i]);
        }

        for (const std::vector<cloudT::Ptr>& ring : sections) {
            for (const cloudT::Ptr section : ring) {
                if (!section)
                    continue;

                // Perform Segmentation
                pcl::PointIndices::Ptr section_inlier_indexes(new pcl::PointIndices);
                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                seg.setInputCloud(section);
                seg.segment(*section_inlier_indexes, *coefficients);

                if (section_inlier_indexes->indices.size() == 0) {
                    RCLCPP_WARN(get_logger(), "Could not estimate a planar model for the given section\n");
                    continue;
                }

                // Indexes to points
                cloudT section_inliers;
                extract_indicies.setInputCloud(section);
                extract_indicies.setIndices(section_inlier_indexes);
                extract_indicies.setNegative(negative);
                extract_indicies.filter(section_inliers);

                inliers += section_inliers;
            }
        }

        // PCL cloud to PointCloud2
        RCLCPP_DEBUG(get_logger(), "Publish pointcloud of size %ld \n", inliers.size());

        sensor_msgs::msg::PointCloud2::UniquePtr publish_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(inliers, *publish_cloud);
        RCLCPP_INFO(get_logger(), "inliers now has %ld points\n", inliers.size());

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
