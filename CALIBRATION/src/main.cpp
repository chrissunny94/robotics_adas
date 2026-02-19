#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace message_filters;

class SensorFusionNode : public rclcpp::Node
{
public:
    SensorFusionNode()
        : Node("sensor_fusion_node")
    {
        // --- LiDAR1 + LiDAR2 Subscribers ---
        lidar1_sub_.subscribe(this, "/lidar1/scan");
        lidar2_sub_.subscribe(this, "/lidar2/scan");

        lidar_sync_ = std::make_shared<SyncLidarLidar>(SyncLidarLidar(10), lidar1_sub_, lidar2_sub_);
        lidar_sync_->registerCallback(std::bind(&SensorFusionNode::lidarCallback, this, std::placeholders::_1, std::placeholders::_2));

        // --- LiDAR + Camera Subscribers ---
        lidar_sub_.subscribe(this, "/lidar1/scan");
        camera_sub_.subscribe(this, "/camera/image_raw");

        lidar_camera_sync_ = std::make_shared<SyncLidarCamera>(SyncLidarCamera(10), lidar_sub_, camera_sub_);
        lidar_camera_sync_->registerCallback(std::bind(&SensorFusionNode::lidarCameraCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node Initialized");
    }

private:
    // Type aliases for sync policies
    using SyncPolicyLidarLidar = sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;
    using SyncLidarLidar = Synchronizer<SyncPolicyLidarLidar>;

    using SyncPolicyLidarCamera = sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::Image>;
    using SyncLidarCamera = Synchronizer<SyncPolicyLidarCamera>;

    // Subscribers
    Subscriber<sensor_msgs::msg::LaserScan> lidar1_sub_;
    Subscriber<sensor_msgs::msg::LaserScan> lidar2_sub_;

    Subscriber<sensor_msgs::msg::LaserScan> lidar_sub_;
    Subscriber<sensor_msgs::msg::Image> camera_sub_;

    // Synchronizers
    std::shared_ptr<SyncLidarLidar> lidar_sync_;
    std::shared_ptr<SyncLidarCamera> lidar_camera_sync_;

    // Callbacks
    void lidarCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr lidar1,
                       const sensor_msgs::msg::LaserScan::ConstSharedPtr lidar2)
    {
        RCLCPP_INFO(this->get_logger(), "Synchronized LiDARs received. Lidar1 points: %ld, Lidar2 points: %ld",
                    lidar1->ranges.size(), lidar2->ranges.size());
        // Example processing: fuse scans, filter, etc.
    }

    void lidarCameraCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr lidar,
                             const sensor_msgs::msg::Image::ConstSharedPtr image)
    {
        RCLCPP_INFO(this->get_logger(), "Synchronized LiDAR + Camera received. Lidar points: %ld, Image resolution: %dx%d",
                    lidar->ranges.size(), image->width, image->height);
        // Example processing: project lidar to camera, visualize, calibration, etc.
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
