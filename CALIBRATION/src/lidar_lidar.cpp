#include "lidar_lidar.hpp"

namespace calibration {

LiDARLiDARNode::LiDARLiDARNode()
    : Node("lidar_lidar_node")
{
    sub1_.subscribe(this, "/lidar1/points");
    sub2_.subscribe(this, "/lidar2/points");

    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSync>>(ApproxSync(10), sub1_, sub2_);
    sync_->registerCallback(std::bind(&LiDARLiDARNode::cloudCallback, this,
                                     std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "LiDAR-LiDAR ICP/NDT node initialized");
}

void LiDARLiDARNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg1,
                                   const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg2)
{
    CloudT::Ptr cloud1(new CloudT);
    CloudT::Ptr cloud2(new CloudT);
    pcl::fromROSMsg(*msg1, *cloud1);
    pcl::fromROSMsg(*msg2, *cloud2);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud2);
    icp.setInputTarget(cloud1);
    CloudT::Ptr aligned(new CloudT);
    icp.align(*aligned);

    if (icp.hasConverged())
    {
        RCLCPP_INFO(this->get_logger(), "ICP converged! Score: %f", icp.getFitnessScore());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
    }
}

} // namespace calibration
