#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using list = std::vector<std::string>;

class DummyCameraNode : public rclcpp::Node {
public:
    DummyCameraNode()
        : rclcpp::Node("dummy_camera")
    {
        declare_parameter<list>("files", list());
        declare_parameter<int64_t>("fps", 30);

        int64_t fps = get_parameter("fps").as_int();
        int64_t frequency = (int64_t)((1000.0) / (double)fps);

        auto files = get_parameter("files").as_string_array();
        for (auto& file : files) {
            cv::Mat image = cv::imread(file);

            if (image.empty()) {
                RCLCPP_WARN(get_logger(), "unable to load file: %s", file.c_str());
            } else {
                m_images.emplace_back(image);
                RCLCPP_INFO(get_logger(), "file loaded: %s", file.c_str());
            }
        }

        if (m_images.size() == 0) {
            RCLCPP_FATAL(get_logger(), "no files to show");
            exit(1);
        }

        m_publisher = create_publisher<sensor_msgs::msg::Image>("/image", 1);
        m_timer = create_wall_timer(std::chrono::milliseconds(frequency),
            std::bind(&DummyCameraNode::publish_image, this));

        RCLCPP_INFO(get_logger(), "started");
    }

private:
    void publish_image()
    {
        auto image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", m_images[m_counter]).toImageMsg();
        image->header.frame_id = "camera";
        image->header.stamp = now();
        m_publisher->publish(*image);

        ++m_counter;
        if (m_counter >= m_images.size()) {
            m_counter = 0;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::vector<cv::Mat> m_images;
    uint32_t m_counter { 0 };
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DummyCameraNode>());

    rclcpp::shutdown();
    return 0;
}