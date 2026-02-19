#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace calibration {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class LiDARLiDARNode : public rclcpp::Node
{
public:
    LiDARLiDARNode();

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg1,
                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg2);

    // Synchronizer typedef
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                            sensor_msgs::msg::PointCloud2> ApproxSync;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub1_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub2_;
    std::shared_ptr<message_filters::Synchronizer<ApproxSync>> sync_;
};

} // namespace calibration
