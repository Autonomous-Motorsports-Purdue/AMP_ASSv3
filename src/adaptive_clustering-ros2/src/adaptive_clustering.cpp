// Copyright (C) 2018  Zhi Yan and Li Sun
// Copyright (C) 2023  Andres Hoyos and Haoguang Yang

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

// ROS 2
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "adaptive_clustering/msg/cluster_array.hpp"

// PCL
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

// for portability -- not every project is shipped with autoware.auto
#if __has_include(<autoware_auto_perception_msgs/msg/bounding_box.hpp>)
#include <autoware_auto_perception_msgs/msg/bounding_box.hpp>
#include <autoware_auto_perception_msgs/msg/bounding_box_array.hpp>

using autoware_auto_perception_msgs::msg::BoundingBox;
using autoware_auto_perception_msgs::msg::BoundingBoxArray;
#endif

using namespace std::chrono_literals;
using adaptive_clustering::msg::ClusterArray;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using sensor_msgs::msg::PointCloud2;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// #define LOG

class AdaptiveClustering : public rclcpp::Node {
 public:
  AdaptiveClustering() : Node("adaptive_clustering") {
    print_fps_ = this->declare_parameter<bool>("print_fps", false);
    cluster_size_min_ =
        static_cast<unsigned int>(this->declare_parameter<int>("cluster_size_min", 3));
    cluster_size_max_ =
        static_cast<unsigned int>(this->declare_parameter<int>("cluster_size_max", 5000));
    // pre-processing options (ground removal)
    do_ground_ceiling_rm_ = this->declare_parameter<bool>("perform_ground_ceiling_removal", true);
    z_axis_min_ = this->declare_parameter<float>("z_axis_min", 0.1);
    z_axis_max_ = this->declare_parameter<float>("z_axis_max", 10.0);
    // Divide the point cloud into nested circular regions centred at the sensor for region-wise
    // clustering For more details, see our IROS-17 paper "Online learning for human classification
    // in 3D LiDAR-based tracking"
    sensor_model_ = this->declare_parameter<std::string>("sensor_model", "");
    if (sensor_model_.compare("VLP-16") == 0) {
      regions_ = std::vector<long int>{2, 3, 3, 3, 3, 3, 3, 2, 3, 3, 3, 3, 3, 3};
    } else if (sensor_model_.compare("HDL-32E") == 0) {
      regions_ = std::vector<long int>{4, 5, 4, 5, 4, 5, 5, 4, 5, 4, 5, 5, 5, 5};
    } else if (sensor_model_.compare("HDL-64E") == 0) {
      regions_ = std::vector<long int>{14, 14, 14, 15, 14};
    } else {
      regions_ = this->declare_parameter<std::vector<long int>>(
          "circular_region_width", std::vector<long int>{30});
    }
    // post-processing options
    leaf_ = this->declare_parameter<int>("leaf", 3);
    k_merging_threshold_ = this->declare_parameter<float>("k_merging_threshold", 0.1);
    z_merging_threshold_ = this->declare_parameter<float>("z_merging_threshold", 0.0);
    radius_min_ = this->declare_parameter<float>("radius_min", 0.4);
    radius_max_ = this->declare_parameter<float>("radius_max", 120.0);
    generate_bounding_boxes_ = this->declare_parameter<bool>("generate_bounding_boxes", true);

    /*** Subscribers ***/
    point_cloud_sub_ = this->create_subscription<PointCloud2>(
        "nonground", rclcpp::SensorDataQoS(),
        std::bind(&AdaptiveClustering::pointCloudCallback, this, std::placeholders::_1));

    /*** Publishers ***/
    if (do_ground_ceiling_rm_) {
      cloud_filtered_pub_ =
          this->create_publisher<PointCloud2>("cloud_filtered", rclcpp::SystemDefaultsQoS());
    }
    cluster_array_pub_ =
        this->create_publisher<ClusterArray>("clusters", rclcpp::SystemDefaultsQoS());
    pose_array_pub_ = this->create_publisher<PoseArray>("poses", rclcpp::SystemDefaultsQoS());
    marker_array_pub_ =
        this->create_publisher<MarkerArray>("clustering_markers", rclcpp::SystemDefaultsQoS());

#ifdef AUTOWARE_AUTO_PERCEPTION_MSGS__MSG__BOUNDING_BOX_HPP_
    bounding_boxes_pub_ = this->create_publisher<BoundingBoxArray>("/perception/lidar_clusters",
                                                                   rclcpp::SystemDefaultsQoS());
    wall_boxes_pub_ = this->create_publisher<BoundingBoxArray>("/perception/lidar_clusters_wall",
                                                               rclcpp::SystemDefaultsQoS());
    obstacle_boxes_pub_ = this->create_publisher<BoundingBoxArray>(
        "/perception/lidar_clusters_obstacle", rclcpp::SystemDefaultsQoS());
#endif
    obstacle_marker_array_pub_ = this->create_publisher<MarkerArray>(
        "/perception/lidar_clusters_obstacle_marker", rclcpp::SystemDefaultsQoS());

    // fps initialization
    reset = true;
    frames = 0;
    start_time = this->now();
  }

 private:
  void pointCloudCallback(PointCloud2::UniquePtr ros_pc2_in);

  bool print_fps_;
  // If a pre-processed point cloud is used, we do't need the internal heuristics-based ground
  // removal stage.
  bool do_ground_ceiling_rm_;
  std::string sensor_model_;
  int leaf_;
  float z_axis_min_;
  float z_axis_max_;
  unsigned int cluster_size_min_;
  unsigned int cluster_size_max_;

  const int region_max_ = 10;  // Change this value to match how far you want to detect.
  std::vector<long int> regions_;

  // post-processing of clustering results
  float k_merging_threshold_;
  float z_merging_threshold_;
  float radius_min_;
  float radius_max_;
  bool generate_bounding_boxes_;

  rclcpp::Publisher<ClusterArray>::SharedPtr cluster_array_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr cloud_filtered_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_array_pub_, obstacle_marker_array_pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr point_cloud_sub_;

#ifdef AUTOWARE_AUTO_PERCEPTION_MSGS__MSG__BOUNDING_BOX_HPP_
  rclcpp::Publisher<BoundingBoxArray>::SharedPtr bounding_boxes_pub_, wall_boxes_pub_,
                                                 obstacle_boxes_pub_;
#endif

  // fps calculation
  int frames;
  rclcpp::Time start_time;
  bool reset = true;
};

void AdaptiveClustering::pointCloudCallback(PointCloud2::UniquePtr ros_pc2_in) {
  if (print_fps_ && reset) {
    frames = 0;
    start_time = this->now();
    reset = false;
  }  // fps

  /*** Convert ROS message to PCL ***/
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);

  /*** Remove ground and ceiling ***/
  pcl::IndicesPtr pc_indices(new std::vector<int>);
  if (do_ground_ceiling_rm_) {
    for (size_t i = 0; i < pcl_pc_in->size(); ++i) {
      if (i % leaf_) continue;
      if (pcl_pc_in->points[i].z < z_axis_min_ || pcl_pc_in->points[i].z > z_axis_max_) continue;
      pc_indices->push_back(i);
    }
  } else {
    pc_indices->resize(pcl_pc_in->size());
    // sequentially increment from 1 to size of cloud
    std::iota(pc_indices->begin(), pc_indices->end(), 1);
  }

  /*** Divide the point cloud into nested circular regions ***/
  std::vector<std::vector<int>> indices_array{};
  indices_array.resize(region_max_);
  for (size_t i = 0; i < pc_indices->size(); i++) {
    float range = 0.0;
    for (int j = 0; j < region_max_; j++) {
      float d2 = pcl_pc_in->points[(*pc_indices)[i]].x * pcl_pc_in->points[(*pc_indices)[i]].x +
                 pcl_pc_in->points[(*pc_indices)[i]].y * pcl_pc_in->points[(*pc_indices)[i]].y +
                 pcl_pc_in->points[(*pc_indices)[i]].z * pcl_pc_in->points[(*pc_indices)[i]].z;
      if (d2 > radius_min_ * radius_min_ && d2 < radius_max_ * radius_max_ && d2 > range * range &&
          d2 <= (range + regions_[j]) * (range + regions_[j])) {
        indices_array[j].push_back((*pc_indices)[i]);
        break;
      }
      range += regions_[j];
    }
  }

  /*** Euclidean clustering ***/
  float tolerance = 0.0;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr,
              Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>>
      clusters;
  int last_clusters_begin = 0;
  int last_clusters_end = 0;

  for (int i = 0; i < region_max_; i++) {
    tolerance += 0.2;
    if (indices_array[i].size() > cluster_size_min_) {
      pcl::IndicesPtr indices_array_ptr(new std::vector<int>(indices_array[i]));
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud(pcl_pc_in, indices_array_ptr);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min_);
      ec.setMaxClusterSize(cluster_size_max_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(pcl_pc_in);
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);

      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
           it != cluster_indices.end(); it++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end();
             ++pit) {
          cluster->points.push_back(pcl_pc_in->points[*pit]);
        }

        /*** Merge clusters separated by nested regions ***/
        for (int j = last_clusters_begin; j < last_clusters_end; j++) {
          pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
          int K = 1;  // the number of neighbors to search for
          std::vector<int> k_indices(K);
          std::vector<float> k_sqr_distances(K);
          kdtree.setInputCloud(cluster);
          if (clusters[j]->points.size() >= 1) {
            if (kdtree.nearestKSearch(*clusters[j], clusters[j]->points.size() - 1, K, k_indices,
                                      k_sqr_distances) > 0) {
              if (k_sqr_distances[0] < k_merging_threshold_) {
                *cluster += *clusters[j].get();
                clusters.erase(clusters.begin() + j);
                last_clusters_end--;
                // std::cerr << "k-merging: clusters " << j << " is merged" << std::endl;
              }
            }
          }
        }
        /**************************************************/

        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
      }

      /*** Merge z-axis clusters ***/
      for (size_t j = last_clusters_end; j < clusters.size(); j++) {
        Eigen::Vector4f j_min, j_max;
        pcl::getMinMax3D(*clusters[j], j_min, j_max);
        for (size_t k = j + 1; k < clusters.size(); k++) {
          Eigen::Vector4f k_min, k_max;
          pcl::getMinMax3D(*clusters[k], k_min, k_max);
          if (std::max(std::min((double)j_max[0], (double)k_max[0]) -
                           std::max((double)j_min[0], (double)k_min[0]),
                       0.0) *
                  std::max(std::min((double)j_max[1], (double)k_max[1]) -
                               std::max((double)j_min[1], (double)k_min[1]),
                           0.0) >
              z_merging_threshold_) {
            *clusters[j] += *clusters[k];
            clusters.erase(clusters.begin() + k);
            // std::cerr << "z-merging: clusters " << k << " is merged into " << j << std::endl;
          }
        }
      }
      last_clusters_begin = last_clusters_end;
      last_clusters_end = clusters.size();
      /*****************************/
    }
  }

  /* Post-process the clusters -- distinguish between obstacles and walls */
  std::vector<bool> isWall(clusters.size());
  std::vector<std::vector<float>> min_max_coords(clusters.size());
  std::vector<std::vector<float>> centroid_coords(clusters.size());
  std::vector<std::vector<float>> box_sizes(clusters.size());
  // TODO: use std::transform instead.
  for (size_t i = 0; i < clusters.size(); i++) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*clusters[i], centroid);
    // FIXME: Add a method to compute the orientation of the bounding box
    // FIXME: the quaternion (orientation of the box) should replace [0., 0., 0., 1.].
    centroid_coords[i] = {centroid[0], centroid[1], centroid[2], 0., 0., 0., 1.};

    Eigen::Vector4f min_coords, max_coords;
    pcl::getMinMax3D(*clusters[i], min_coords, max_coords);
    min_max_coords[i] = {min_coords[0], min_coords[1], min_coords[2],
                         max_coords[0], max_coords[1], max_coords[2]};
    box_sizes[i] = {(max_coords[0] - min_coords[0]), (max_coords[1] - min_coords[1]),
                    (max_coords[2] - min_coords[2])};
#ifdef LOG
    std::cerr << ros_pc2_in->header.seq << " " << ros_pc2_in->header.stamp << " " << min_coords[0]
              << " " << min_coords[1] << " " << min_coords[2] << " " << max_coords[0] << " "
              << max_coords[1] << " " << max_coords[2] << " " << std::endl;
#endif
    float max_box_dim = box_sizes[i][0];
    float min_box_dim = box_sizes[i][0];
    for (size_t j = 0; j < 3; j++) {
      max_box_dim = std::fmax(box_sizes[i][j], max_box_dim);
      min_box_dim = std::fmin(box_sizes[i][j], min_box_dim);
    }
    isWall[i] = (box_sizes[i][0] * box_sizes[i][1] * box_sizes[i][2] >= 30.0) ||
                (max_box_dim > 10.0 * min_box_dim);
  }

  /*** Output: use lazy publishers ***/
  if (do_ground_ceiling_rm_) {
    if (cloud_filtered_pub_->get_subscription_count()) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
      std::unique_ptr<PointCloud2> ros_pc2_out(new PointCloud2);
      pcl::copyPointCloud(*pcl_pc_in, *pc_indices, *pcl_pc_out);
      pcl::toROSMsg(*pcl_pc_out, *ros_pc2_out.get());
      cloud_filtered_pub_->publish(std::move(ros_pc2_out));
    }
  }

  if (cluster_array_pub_->get_subscription_count()) {
    std::unique_ptr<ClusterArray> cluster_array(new ClusterArray);
    for (size_t i = 0; i < clusters.size(); i++) {
      PointCloud2 ros_pc2_out;
      pcl::toROSMsg(*clusters[i], ros_pc2_out);
      cluster_array->clusters.push_back(ros_pc2_out);
    }
    if (cluster_array->clusters.size()) {
      cluster_array->header = ros_pc2_in->header;
      cluster_array_pub_->publish(std::move(cluster_array));
    }
  }

  if (pose_array_pub_->get_subscription_count() > 0) {
    std::unique_ptr<PoseArray> pose_array(new PoseArray);
    pose_array->poses.resize(clusters.size());
    std::transform(centroid_coords.begin(), centroid_coords.end(), pose_array->poses.begin(),
                   [](const std::vector<float> &c) {
                     geometry_msgs::msg::Pose pose;
                     pose.position.x = c[0];
                     pose.position.y = c[1];
                     pose.position.z = c[2];
                     pose.orientation.x = c[3];
                     pose.orientation.y = c[4];
                     pose.orientation.z = c[5];
                     pose.orientation.w = c[6];
                     return pose;
                   });
    pose_array->header = ros_pc2_in->header;
    pose_array_pub_->publish(std::move(pose_array));
  }

  std::unique_ptr<MarkerArray> marker_array = std::make_unique<MarkerArray>();
  marker_array->markers.reserve(clusters.size());
  std::unique_ptr<MarkerArray> obstacle_marker_array = std::make_unique<MarkerArray>();
  Marker marker;
  marker.header = ros_pc2_in->header;
  marker.ns = "adaptive_clustering";
  marker.type = Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.a = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = rclcpp::Duration(0.5s);
  for (size_t i = 0; i < clusters.size(); i++) {
    marker.id = static_cast<int>(i);
    marker.pose.position.x = (min_max_coords[i][0] + min_max_coords[i][3]) * 0.5;
    marker.pose.position.y = (min_max_coords[i][1] + min_max_coords[i][4]) * 0.5;
    marker.pose.position.z = (min_max_coords[i][2] + min_max_coords[i][5]) * 0.5;
    marker.pose.orientation.x = centroid_coords[i][3];
    marker.pose.orientation.y = centroid_coords[i][4];
    marker.pose.orientation.z = centroid_coords[i][5];
    marker.pose.orientation.w = centroid_coords[i][6];
    marker.scale.x = box_sizes[i][0];
    marker.scale.y = box_sizes[i][1];
    marker.scale.z = box_sizes[i][2];
    marker_array->markers.push_back(marker);
    if (!isWall[i]) {
      obstacle_marker_array->markers.push_back(marker);
      // marker color tweaks
      obstacle_marker_array->markers.back().color.r = 1.0;
      obstacle_marker_array->markers.back().color.g = 0.0;
      obstacle_marker_array->markers.back().color.b = 0.0;
      obstacle_marker_array->markers.back().color.a = 0.75;
    }
  }

#ifdef AUTOWARE_AUTO_PERCEPTION_MSGS__MSG__BOUNDING_BOX_HPP_
  std::unique_ptr<BoundingBoxArray> bounding_boxes(new BoundingBoxArray);
  std::unique_ptr<BoundingBoxArray> wall_bounding_boxes(new BoundingBoxArray);
  std::unique_ptr<BoundingBoxArray> obstacle_bounding_boxes(new BoundingBoxArray);
  bounding_boxes->header = ros_pc2_in->header;
  wall_bounding_boxes->header = ros_pc2_in->header;
  obstacle_bounding_boxes->header = ros_pc2_in->header;
  autoware_auto_perception_msgs::msg::BoundingBox box;
  for (size_t i = 0; i < clusters.size(); i++) {
    box.centroid.x = marker_array->markers[i].pose.position.x;
    box.centroid.y = marker_array->markers[i].pose.position.y;
    box.centroid.z = marker_array->markers[i].pose.position.z;
    box.orientation.x = marker_array->markers[i].pose.orientation.x;
    box.orientation.y = marker_array->markers[i].pose.orientation.y;
    box.orientation.z = marker_array->markers[i].pose.orientation.z;
    box.orientation.w = marker_array->markers[i].pose.orientation.w;
    box.size.x = marker_array->markers[i].scale.x;
    box.size.y = marker_array->markers[i].scale.y;
    box.size.z = marker_array->markers[i].scale.z;
    bounding_boxes->boxes.push_back(box);
    // figure out geometrically if it is a wall
    if (isWall[i]) {
      wall_bounding_boxes->boxes.push_back(box);
    } else {
      obstacle_bounding_boxes->boxes.push_back(box);
    }
  }
  bounding_boxes_pub_->publish(std::move(bounding_boxes));
  wall_boxes_pub_->publish(std::move(wall_bounding_boxes));
  obstacle_boxes_pub_->publish(std::move(obstacle_bounding_boxes));
#endif

  marker_array_pub_->publish(std::move(marker_array));
  obstacle_marker_array_pub_->publish(std::move(obstacle_marker_array));

  if (print_fps_) {
    if (++frames > 10) {
      std::cerr << "[adaptive_clustering] fps = "
                << float(frames) / (this->now() - start_time).seconds()
                << ", timestamp = " << this->now().seconds() << std::endl;
      reset = true;
    }  // fps
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdaptiveClustering>());
  rclcpp::shutdown();

  return 0;
}