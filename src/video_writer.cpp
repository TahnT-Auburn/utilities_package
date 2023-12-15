//STD
#include <iostream>
#include <filesystem>
#include <cstdio>

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

//Header
#include "video_writer.h"

void VideoWriterClass::parseParameters()
{
    //Declare ROS2 parameters
    this->declare_parameter("input_path");
    this->declare_parameter("frame_name");
    this->declare_parameter("extension");
    this->declare_parameter("output_path");
    this->declare_parameter("fps");

    //Get ROS2 parameters
    this->get_parameter("input_path", input_path_);
    this->get_parameter("frame_name", frame_name_);
    this->get_parameter("extension", extension_);
    this->get_parameter("output_path", output_path_);
    this->get_parameter("fps", fps_);

}

void VideoWriterClass::writeVideo()
{   
    
    //Read in first image to generate video specs
    cv::Mat cv_image_init;
    std::string init_path = input_path_ + frame_name_ + "1" + extension_;
    cv_image_init = cv::imread(init_path, cv::IMREAD_COLOR);
    int width_ = cv_image_init.cols;
    int height_ = cv_image_init.rows;
    cv::Size frame_size(width_, height_);
    RCLCPP_INFO(this->get_logger(), "width: %d, height: %d", width_, height_);

    //Initialize video writer object
    cv::VideoWriter output(output_path_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_, frame_size);
           
    //Use filesystem to access saved frames
    auto dirIter = std::filesystem::directory_iterator(input_path_);

    //Read each frame and save to video writer
    int file_count_ = 0;
    for (auto& entry : dirIter)
    {
        if (entry.is_regular_file())
        {   
            //Access current frame
            ++file_count_;
            std::string file_count_str_ = std::to_string(file_count_);
            std::string frame_file_ = input_path_ + frame_name_ + file_count_str_ + extension_;

            //Read as OpenCV image
            cv::Mat cv_image;
            cv_image = cv::imread(frame_file_, cv::IMREAD_COLOR);
            if (cv_image.empty())   //Shutdown if frame is empty
            {
                RCLCPP_ERROR(this->get_logger(), "Frame is empty, shutting down");
                cv::destroyAllWindows();
                rclcpp::shutdown();
            }
            
            //Display image
            cv::imshow("CV Image", cv_image);
            char key = (char) cv::waitKey(1);
            if (key == 27)
            {
                RCLCPP_INFO(this->get_logger(), "'esc' key pressed, shutting down");
                cv::destroyAllWindows();
                rclcpp::shutdown();
            }

            //Write video file
            output.write(cv_image);
            RCLCPP_INFO_ONCE(this->get_logger(), "Writing video ...");
            RCLCPP_INFO(this->get_logger(), "%d frame(s) written", file_count_);
            
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid frame detected after frame %d", file_count_);
        }
        
    }
    //RCLCPP_INFO(this->get_logger(), "%d files detected", file_count_);
    cv::destroyAllWindows();
    output.release();
    RCLCPP_INFO(this->get_logger(), "Write Successful");
    RCLCPP_INFO(this->get_logger(), "Video file written to: %s", output_path_.c_str());
    
}

/*
void VideoWriterClass::deleteFrames()
{               
    RCLCPP_INFO(this->get_logger(), "Deleting frames ...");
    //Access frame files
    auto dirIter_del = std::filesystem::directory_iterator(input_path_);
    int file_count_del = 0;
    for (auto& entry : dirIter_del)
    {
        if (entry.is_regular_file())
        {   
            //Access current frame
            ++file_count_del;
            std::string file_count_str_del_= std::to_string(file_count_del);
            std::string frame_file_del_ = input_path_ + frame_name_ + file_count_str_del_ + extension_;

            //Delete frames to convserve memory
            int remove_check = std::remove(frame_file_del_.c_str());
            if (remove_check == 0)
            {
                RCLCPP_INFO(this->get_logger(), "frame %d deleted", file_count_del);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "frame %d could not be deleted", file_count_del);
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "Delete successful");
}
*/