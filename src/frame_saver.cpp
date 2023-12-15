//STD
#include <memory>
#include <iostream>
#include <functional>
#include <chrono>
#include <filesystem>

//OpenCV and ROS2
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try 
  { 
    //Initialize save directory
    std::string save_frames_dir = "/home/tahnt/T3_Repos/post_process_packages/ros2_ws/src/utilities_package/frames";
    std::string save_frames_name = "/frame";
    std::string save_frames_ext = ".jpg";

    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_INFO_ONCE(logger,"Subcribing to image topic ...");

    //Convert from ROS image msg to OpenCV image
    cv::Mat cv_image;
    cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;

    //Display image
    cv::imshow("OpenCV Playback", cv_image);
    char key = (char) cv::waitKey(1);

    //Write frames
    if (key < 0)
    { 
      //Check how many frames have been saved
      auto dirIter = std::filesystem::directory_iterator(save_frames_dir);
      int fileCount = 0;
      for (auto& entry : dirIter)
      {
        if (entry.is_regular_file())
        {
            ++fileCount;
        }
      }

      //Generate iterating save file based on file count
      int count;
      if (fileCount == 0)
      {
        count = 1;
      }
      else if (fileCount > 0)
      {
        count = fileCount + 1;
      }
      std::string save_frames_count = std::to_string(count);
      std::string save_frames_file = save_frames_dir + save_frames_name + save_frames_count + save_frames_ext;

      //Save frame with imwrite
      cv::imwrite(save_frames_file, cv_image);
      RCLCPP_INFO(logger, "%d frame(s) saved", count);
    }
    else if (key == 27)
    {
      RCLCPP_INFO_ONCE(logger, "'esc' key pressed, shutting down");
      cv::destroyAllWindows();
      rclcpp::shutdown();
    }
  } 
  catch (const cv_bridge::Exception & e) 
  {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
  cv::namedWindow("OpenCV Playback");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("tri028s_cc/stream", 1, imageCallback);
  rclcpp::spin(node);
  cv::destroyWindow("OpenCV Playback");

  return 0;
}