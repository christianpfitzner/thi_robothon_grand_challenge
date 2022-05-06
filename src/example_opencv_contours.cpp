#include <algorithm>
#include <bits/c++config.h>
#include <fstream>
#include <memory>
#include <rclcpp/parameter_value.hpp>
#include <stdlib.h>
#include <functional>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"


class IMAGE_CONTOURS : public rclcpp::Node
{
    public:
        IMAGE_CONTOURS() : Node("example_contours")
        {
            // Ceclare a few start parameters 
            this->declare_parameter("sub_rgb_topic", "image");
            this->declare_parameter("pub_rgb_topic", "image_processed");
            this->declare_parameter("view_debug_windows", false);

            rclcpp::Parameter _param_sub_topic  = this->get_parameter("sub_rgb_topic");
            rclcpp::Parameter _param_pub_topic  = this->get_parameter("pub_rgb_topic");
            rclcpp::Parameter _param_view_debug = this->get_parameter("view_debug_windows");

            // Write the parameters to variables
            _sub_topic  = _param_sub_topic.as_string();
            _pub_topic  = _param_pub_topic.as_string();
            _view_debug = _param_view_debug.as_bool();
            
            // Convert a bool variable to string, to print it in the info messages on the beginning
            std::string _view_debug_str = _view_debug ? "true" : "false";

            // Info message at the beginng
            RCLCPP_INFO(this->get_logger(), "Start image processing...");
            RCLCPP_INFO(this->get_logger(), "Subscripted RGB image on topic:    %s", _sub_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "Publish result RGB image on topic: %s", _pub_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "Showing the dubug image windows:   %s", _view_debug_str.c_str());

            // Subscriber to the camera RGB image
            _sub_image = this->create_subscription<sensor_msgs::msg::Image>
                (_sub_topic, rclcpp::QoS(1).best_effort(), std::bind(&IMAGE_CONTOURS::cb_image, this, std::placeholders::_1));

            // Publisher the processed RGB image
            _pub_image = this->create_publisher<sensor_msgs::msg::Image>
                (_pub_topic, rclcpp::QoS(1).reliable());
        }

        ~IMAGE_CONTOURS()
        {
            cv::destroyAllWindows();
        }

    private:
        // Every time a picture comes from the camera node, the callback is triggerd and the image is processd
        void cb_image(const sensor_msgs::msg::Image::SharedPtr msg)
        {            
            // To work with the image, we need to put it in the OpenCV universe and check if it is correct
            cv_bridge::CvImagePtr cv_ptr;

            try
            {
                // Input is the ROS image message and an optional cv_encoding
                cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            }
            catch (cv_bridge::Exception &exp) 
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge eception: %s", exp.what());
            }

            // OpenCV matrix type variables to store the OpenCV images
            cv::Mat mat_input, mat_gray, mat_blur, mat_thresh, mat_thresh_con, mat_canny, mat_canny_con, mat_contour;

            // OpenCV normaly wants to get BGR images. If the colors of the image seems not correct, you have to switch colors to RGB
            cv::cvtColor(cv_ptr->image, mat_input, cv::COLOR_RGB2BGR);

            // Convert the image to grayscale to prepare it for for the contour detection algorithm
            cv::cvtColor(mat_input, mat_gray, cv::COLOR_RGB2GRAY);
            if(_view_debug == true)
            {
                cv::imshow("grayscale", mat_gray);
            }

            // Blur the grayscale image to reduce noise on the corners
            cv::blur(mat_gray, mat_blur, cv::Size(3, 3));
            if(_view_debug == true)
            {
                cv::imshow("blur", mat_blur);
            }

            // First apply binary thresholding or canny edge detection on the grayscale image.
            // Any pixel with a value greater than 130 will be set to 255 (white), all remaining will 
            // be set to 0 (black).
            cv::threshold(mat_blur, mat_thresh, 140, 255, cv::THRESH_BINARY);
            if(_view_debug == true)
            {
                cv::imshow("threshold", mat_thresh);
            }

            // Canny edge detection
            cv::Canny(mat_blur, mat_canny, 150, 150 * 2);
            if(_view_debug == true)
            {
                cv::imshow("canny edge", mat_canny);
            }

            // Detect the contours on the threshold binary image using CHAIN_APPROX_SIMPLE
            std::vector<std::vector<cv::Point>> contours_thresh;
            std::vector<cv::Vec4i> hierarchy_thresh;
            cv::findContours(mat_thresh, contours_thresh, hierarchy_thresh, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if(_view_debug == true)
            {
                cv::Mat mat_thresh_contours = mat_thresh.clone();
                cv::drawContours(mat_thresh_contours, contours_thresh, -1, cv::Scalar_(0, 255, 0), 2);
                cv::imshow("canny edge", mat_thresh_contours);
            }

            // Detect the contours on the threshold canny edge using CHAIN_APPROX_SIMPLE
            std::vector<std::vector<cv::Point>> contours_canny;
            std::vector<cv::Vec4i> hierarchy_canny;
            cv::findContours(mat_canny, contours_canny, hierarchy_canny, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if(_view_debug == true)
            {
                cv::Mat mat_canny_contours = mat_canny.clone();
                cv::drawContours(mat_canny_contours, contours_canny, -1, cv::Scalar_(0, 255, 0), 2);
                cv::imshow("canny edge", mat_canny);
            }

            // Draw threshold contour in a set color on the original image and show the result
            cv::Mat mat_final_thresh = mat_input.clone();
            cv::drawContours(mat_final_thresh, contours_thresh, -1, cv::Scalar_(0, 255, 0), 2);
            cv::imshow("final with treshold",   mat_final_thresh);

            // Draw canny edge contour in a set color on the original image and show the result
            cv::Mat mat_final_canny = mat_input.clone();
            cv::drawContours(mat_final_canny, contours_canny, -1, cv::Scalar_(0, 255, 0), 2);
            cv::imshow("final with canny edge",   mat_final_canny);

            // Wait for a key event
            cv::waitKey(3);

            // Convert the OpenCV image back to ROS image message and publish it
            sensor_msgs::msg::Image msg_out2;
            msg_out2.header   = msg->header;    // Same timestamp and tf frame as input image
            msg_out2.encoding = msg->encoding;  // Same encoding as input image
            msg_out2.height   = msg->height;    // Same diamension as input image
            msg_out2.width    = msg->width;
            msg_out2.data.assign(mat_input.datastart,
                                 mat_input.dataend); // Image mat to publish

            _pub_image->publish(std::move(msg_out2));
        }

    private:
        std::string _sub_topic;
        std::string _pub_topic;
        bool _view_debug;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_image;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_image;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMAGE_CONTOURS>());
    rclcpp::shutdown();
    return 0;
}