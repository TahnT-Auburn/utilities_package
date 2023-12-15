//STD
#include <iostream>

//OpenCV
#include <opencv2/opencv.hpp>

//ROS2
#include "rclcpp/rclcpp.hpp"

class VideoWriterClass : public rclcpp::Node
{
public:
    VideoWriterClass()
    : Node("video_writer_class")
    {
        parseParameters();
        writeVideo();
    }
private:
    
    //Initialize ROS2 parameters
    std::string input_path_,
        extension_,
        frame_name_,
        output_path_;
    double fps_;

    cv::Matx33d camera_matrix_;

    void parseParameters();
    void writeVideo();

};

int main(int argc, char* argv[])
{   
    //Call ROS2 subcriber node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoWriterClass>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}