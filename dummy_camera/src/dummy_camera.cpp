#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

class DummyCameraNode: public rclcpp::Node 
{
public:
    DummyCameraNode() : rclcpp::Node("dummy_camera"){
        declare_parameter<std::vector<std::string>>("files", {});
        declare_parameter<int64_t>("fps", 30);

        int64_t fps = get_parameter("fps").as_int();
        int64_t frequency = (int64_t)((1000.0) / (double)fps);

        auto files = get_parameter("files").as_string_array();
        for(auto & file: files){
            cv::Mat image = cv::imread(file);

            if(image.empty()){
                RCLCPP_WARN(get_logger(), "unable to load file: %s", file.c_str());
            } else {
                m_images.emplace_back(image);
                RCLCPP_INFO(get_logger(), "file loaded: %s", file.c_str());
            }
        }

        if(m_images.size() == 0){
            RCLCPP_ERROR(get_logger(), "no files to show");
            m_error = true;
            return;
        }

        m_publisher = create_publisher<sensor_msgs::msg::Image>("dummy_camera/image", 1);
        m_timer = create_wall_timer(std::chrono::milliseconds(frequency), 
                                    std::bind(&DummyCameraNode::publish_image, this));

        RCLCPP_INFO(get_logger(), "started");
    }

    bool has_error() const { return m_error; };
private:
    void publish_image(){
        auto image = cv_bridge::CvImage(std_msgs::msg::Header(), 
                     "bgr8", m_images[m_counter]).toImageMsg();
        image->header.frame_id = "camera";
        image->header.stamp = rclcpp::Clock().now();
        m_publisher->publish(*image);
        m_counter = m_counter + 1 < m_images.size() ? m_counter + 1 : 0;
    }

    bool m_error { false };
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::vector<cv::Mat> m_images;
    uint32_t m_counter { 0 };
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DummyCameraNode>();

    if(node->has_error() == false){
        rclcpp::spin(node);
    }

    rclcpp::shutdown();
    return 0;
}