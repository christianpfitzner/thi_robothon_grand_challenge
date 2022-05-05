#include <algorithm>
#include <memory>
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


class IMAGE_CONVERTER : public rclcpp::Node
{
    public:
        IMAGE_CONVERTER() : Node("image_converter")
        {
            // Info message at the beginng
            RCLCPP_INFO(this->get_logger(), "Start image processing");

            // If set, show the stage windows
            if(_view_debug_windos == true)
            {
                cv::namedWindow(_window_input);
                cv::namedWindow(_window_stage_1);
                cv::namedWindow(_window_stage_2);
            }

            // Show the final image
            cv::namedWindow(_window_final);

            // Subscriber to the camera RGB image
            _sub_image = this->create_subscription<sensor_msgs::msg::Image>
                ("image", rclcpp::QoS(1).best_effort(), std::bind(&IMAGE_CONVERTER::cb_image, this, std::placeholders::_1));

            // Publisher
            _pub_image = this->create_publisher<sensor_msgs::msg::Image>
                ("image_processed", rclcpp::QoS(1).reliable());
        }

        ~IMAGE_CONVERTER()
        {
            cv::destroyAllWindows();
        }

    private:
        // Every time a picture comes from the camera node, the callback is triggerd
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
            cv::Mat mat_input = cv_ptr->image;
            cv::Mat mat_gray, mat_blur, mat_thresh, mat_canny, mat_contour, mat_final;


            // --- Image processing stage 1 ---
            // Convert the image to grayscale to prepare it for for the contour detection algorithm
            cv::cvtColor(mat_input, mat_gray, cv::COLOR_BGR2GRAY);

            // Blur the grayscale image to reduce noise on the corners
            cv::blur(mat_gray, mat_blur, cv::Size(3, 3));

            // First apply binary thresholding or canny edge detection on the grayscale image.
            
            // Any pixel with a value greater than 130 will be set to 255 (white), all remaining will 
            // be set to 0 (black).
            cv::threshold(mat_blur, mat_thresh, 130, 255, cv::THRESH_BINARY);
            
            // Canny edge detection
            cv::Canny(mat_blur, mat_canny, 150, 200);


            // --- Image processing stage 2 ---
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            // Detect the contours on the binary image or canny edge using CHAIN_APPROX_SIMPLE
            // cv::findContours(mat_thresh, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            cv::findContours(mat_canny, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // Draw contour in the given color on the original image
            mat_final = mat_input.clone();
            cv::drawContours(mat_final, contours, -1, cv::Scalar_(0, 255, 0), 2);

            // Show the images in seperate windows
            cv::imshow(_window_input,   mat_input);   // The first window is just showing the input image to verify, we subscried the right camera topic
            cv::imshow(_window_stage_1, mat_thresh);
            // cv::imshow(_window_stage_1, mat_canny);
            // cv::imshow(_window_stage_2, mat_contour);
            cv::imshow(_window_final,   mat_final);

            // Wait for a key event
            cv::waitKey(3);

            // Convert the OpenCV image back to ROS image message and publish
            sensor_msgs::msg::Image msg_out2;
            msg_out2.header   = msg->header;    // Same timestamp and tf frame as input image
            msg_out2.encoding = msg->encoding;  // Same encoding as input image
            msg_out2.height   = msg->height;    // Same diamension as input image
            msg_out2.width    = msg->width;
            msg_out2.data.assign(mat_input.datastart,
                                 mat_input.dataend); // Image mat to publish

            _pub_image->publish(std::move(msg_out2));
        }

        // When the stage windows not longer needed, set to false
        bool _view_debug_windos = true;
        
        std::string _window_input   = "window_input";
        std::string _window_final   = "window_final";
        std::string _window_stage_1 = "window_stage_1";
        std::string _window_stage_2 = "window_stage_2";

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_image;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_image;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMAGE_CONVERTER>());
    rclcpp::shutdown();
    return 0;
}