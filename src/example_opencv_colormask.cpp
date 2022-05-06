#include <algorithm>
#include <array>
#include <bits/c++config.h>
#include <fstream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
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


class IMAGE_COLORMASK: public rclcpp::Node
{
    public:
        IMAGE_COLORMASK() : Node("example_colormask")
        {
            // Ceclare a few start parameters 
            this->declare_parameter("sub_rgb_topic", "image");
            this->declare_parameter("pub_rgb_topic", "image_processed");
            this->declare_parameter("view_debug_windows", false);
            this->declare_parameter("view_debug_filter_1", false);

            rclcpp::Parameter _param_sub_topic      = this->get_parameter("sub_rgb_topic");
            rclcpp::Parameter _param_pub_topic      = this->get_parameter("pub_rgb_topic");
            rclcpp::Parameter _param_view_debug     = this->get_parameter("view_debug_windows");
            rclcpp::Parameter _param_view_filters_1 = this->get_parameter("view_debug_filter_1");

            // Write the parameters to variables
            _sub_topic            = _param_sub_topic.as_string();
            _pub_topic            = _param_pub_topic.as_string();
            _view_debug           = _param_view_debug.as_bool();
            _view_debug_filters_1 = _param_view_filters_1.as_bool();
            
            // Convert a bool variable to string, to print it in the info messages on the beginning
            std::string _view_debug_str = _view_debug ? "true" : "false";
            std::string _view_debug_filters_1_str = _view_debug_filters_1 ? "true" : "false";

            // Info message at the beginng
            RCLCPP_INFO(this->get_logger(), "Start image processing...");
            RCLCPP_INFO(this->get_logger(), "Subscripted RGB image on topic:     %s", _sub_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "Publish result RGB image on topic:  %s", _pub_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "Showing the dubug image windows:    %s", _view_debug_str.c_str());
            RCLCPP_INFO(this->get_logger(), "Showing all filter 1 image windows: %s", _view_debug_filters_1_str.c_str());

            // Subscriber to the camera RGB image
            _sub_image = this->create_subscription<sensor_msgs::msg::Image>
                (_sub_topic, rclcpp::QoS(1).best_effort(), std::bind(&IMAGE_COLORMASK::cb_image, this, std::placeholders::_1));

            // Publisher the processed RGB image
            _pub_image = this->create_publisher<sensor_msgs::msg::Image>
                (_pub_topic, rclcpp::QoS(1).reliable());
        }

        ~IMAGE_COLORMASK()
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


            // OpenCV normaly wants to get BGR images. If the colors of the image seems not correct, you have to switch colors to RGB
            cv::Mat mat_input;
            cv::cvtColor(cv_ptr->image, mat_input, cv::COLOR_BGR2RGB);
            if(_view_debug == true)
            {
                cv::imshow("Input Image", mat_input);
            }


            // First convert the image to HSV values to use the OpenCV color filters
            cv::Mat mat_hsv;
            cv::cvtColor(cv_ptr->image, mat_hsv, cv::COLOR_BGR2HSV);


            // Define the first window to show the trackbars and the color mask output for the first color filter
            cv::namedWindow(_window_mask_1);

            cv::createTrackbar("Low Hue", _window_mask_1, &_low_hue_1, _max_value);
            cv::createTrackbar("High Hue", _window_mask_1, &_high_hue_1, _max_value); 

            cv::createTrackbar("Low Sat", _window_mask_1, &_low_sat_1, _max_value);
            cv::createTrackbar("High Sat", _window_mask_1, &_high_sat_1, _max_value);

            cv::createTrackbar("Low Val", _window_mask_1, &_low_val_1, _max_value);
            cv::createTrackbar("High Val", _window_mask_1, &_high_val_1, _max_value);          


            // Filter for a a color in the image
            cv::Mat mat_color_mask_1;
            cv::inRange(mat_hsv, cv::Scalar(_low_hue_1, _low_sat_1, _low_val_1), cv::Scalar(_high_hue_1, _high_sat_1, _high_val_1), mat_color_mask_1);
            if(_view_debug == true)
            {
                cv::imshow(_window_mask_1, mat_color_mask_1);
            }


            // Settings for the folowing filters
            int _morph_size = 2;
            cv::Mat element = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(2 * _morph_size + 1,
                                         2 * _morph_size + 1),
                cv::Point(_morph_size, _morph_size)
            );

            // Erosion
            cv::Mat mat_mask_1_erosion;
            cv::erode(mat_color_mask_1, mat_mask_1_erosion, element, cv::Point(-1, -1), 1);

            // Dilation
            cv::Mat mat_mask_1_dilation;
            cv::dilate(mat_color_mask_1, mat_mask_1_dilation, element, cv::Point(-1, -1), 1);

            // Opening filter to remove the false positiv pixel
            cv::Mat mat_mask_1_opening;
            cv::morphologyEx(mat_mask_1_erosion, mat_mask_1_opening, cv::MORPH_OPEN, element);

            // Closing filter to remove the false negativ pixel
            cv::Mat mat_mask_1_closing;
            cv::morphologyEx(mat_color_mask_1, mat_mask_1_closing, cv::MORPH_OPEN, element);

            if(_view_debug_filters_1 == true)
            {
                cv::imshow("Mask 1 Erosion", mat_mask_1_erosion);
                cv::imshow("Mask 1 Dilation", mat_mask_1_dilation);
                cv::imshow("Mask 1 Opening", mat_mask_1_opening);
                cv::imshow("Mask 1 Closing", mat_mask_1_closing);
            }

            // Canny edge detection of the choosen filter
            cv::Mat mat_canny_input = mat_mask_1_closing.clone();
            cv::Mat mat_canny_output;
            cv::Canny(mat_canny_input, mat_canny_output, 150, 150 * 2);
            if(_view_debug == true)
            {
                cv::imshow("canny edge", mat_canny_output);
            }

            // Detect the contours on the threshold binary image using CHAIN_APPROX_SIMPLE
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mat_canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            // if(_view_debug == true)
            // {
            //     cv::Mat mat_contours = mat_canny_output.clone();
            //     cv::drawContours(mat_contours, mat_canny_output, -1, cv::Scalar_(0, 255, 0), 2);
            //     cv::imshow("canny edge", mat_contours);
            // }

            // Draw threshold contour in a set color on the original image and show the result
            cv::Mat mat_final_color_1 = mat_input.clone();
            cv::drawContours(mat_final_color_1, contours, -1, cv::Scalar_(0, 255, 0), 2);
            cv::imshow("Finale Color 1",   mat_final_color_1);


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
        bool _view_debug_filters_1;

        // Create variables for the trackbars to play with the color mask values
        std::string _window_mask_1 = "Mask 1";
        std::string _window_mask_2 = "Mask 2";

        int _max_value = 255;
        int _low_hue_1 = 95, _low_sat_1 = 25, _low_val_1 = 160;
        int _high_hue_1 = 150, _high_sat_1 = 255, _high_val_1 = 255;
        // int _low_hue_2 = _min_value, _low_sat_2 = _min_value, _low_val_2 = _min_value;
        // int _high_hue_2 = _max_value, _high_sat_2 = _max_value, _high_val_2 = _max_value;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_image;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_image;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMAGE_COLORMASK>());
    rclcpp::shutdown();
    return 0;
}