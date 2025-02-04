#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class PlaneSegmentationNode : public rclcpp::Node
{
public:
    PlaneSegmentationNode() : Node("my_pcl_node")
    {
        // Subscribe to the input point cloud topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10, std::bind(&PlaneSegmentationNode::pointCloudCallback, this, std::placeholders::_1));

        // Advertise output topics for the plane and non-plane points
        plane_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("plane_points", 10);
        non_plane_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_plane_points", 10);

        RCLCPP_INFO(this->get_logger(), "Plane Segmentation Node Initialized.");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS message to PCL format
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud. Skipping processing.");
            return;
        }

        // Create a container for the model coefficients
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);       // Detect planes
        seg.setMethodType(pcl::SAC_RANSAC);          // Use RANSAC
        seg.setDistanceThreshold(0.01);              // Distance threshold (adjust as needed)

        // Segment the largest planar component from the point cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No plane detected in the point cloud.");
            return;
        }

        // Extract the plane points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false); // Extract the plane
        extract.filter(*cloud_plane);

        // Extract the non-plane points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_non_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setNegative(true); // Extract everything except the plane
        extract.filter(*cloud_non_plane);

        // Convert the plane and non-plane point clouds back to ROS messages
        sensor_msgs::msg::PointCloud2 plane_msg, non_plane_msg;
        pcl::toROSMsg(*cloud_plane, plane_msg);
        pcl::toROSMsg(*cloud_non_plane, non_plane_msg);

        // Set headers for the output messages
        plane_msg.header = msg->header;
        non_plane_msg.header = msg->header;

        // Publish the results
        plane_publisher_->publish(plane_msg);
        non_plane_publisher_->publish(non_plane_msg);

        //RCLCPP_INFO(this->get_logger(), "Published plane with %lu points and non-plane with %lu points.",
                    //cloud_plane->size(), cloud_non_plane->size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_plane_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlaneSegmentationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}