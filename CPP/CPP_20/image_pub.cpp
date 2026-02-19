#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class WebcamPublisherNode : public rclcpp::Node
{
public:
    WebcamPublisherNode()
        : Node("webcam_publisher_node")
    {
        // Create publisher using image_transport
        image_pub_ = image_transport::create_publisher(this, "camera/image_raw");

        // Open webcam (default /dev/video0)
        cap_.open(0);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open webcam!");
            rclcpp::shutdown();
        }

        // Timer to capture frames at 30Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&WebcamPublisherNode::publishFrame, this)
        );

        RCLCPP_INFO(this->get_logger(), "Webcam publisher initialized.");
    }

private:
    void publishFrame()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame from webcam.");
            return;
        }

        // Convert to ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // Publish
        image_pub_.publish(msg);

        // Optional: show locally
        cv::imshow("Webcam", frame);
        cv::waitKey(1);
    }

    image_transport::Publisher image_pub_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebcamPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
