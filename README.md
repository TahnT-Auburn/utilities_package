# utilities_package

## Description:
ROS2 package to handle image data in bag files. Used to save frames from a bag and/or write video files.

The main purpose of this package is to process raw bag files. There are two nodes included:
* **Frame Saver Node** - subscribes to published image messages from bag files and writes each frame to a folder
* **Video Writer Node** - writes a video file from the saved frames

Using these two nodes, you can siphon image data and video data from your recorded bag files for further processing!

**For more information on this package, check the Joplin documentation under the _Utilities Package_ page.**

## Requirements:
* Ubuntu Focal Fossa
* ROS2 Foxy Fitzroy
* Arena SDK for Linux
* C++17 or higher

## To Use:
**_Before Use:_** 
* **Make sure ALL PATHS ARE SET CORRECTLY in `frame_saver.cpp`. For more information, check the Joplin documentation**
* **Make sure ALL PATHS ARE SET CORRECTLY in the launch and config files before use!**
* **These steps assume you have already created a workspace folder and a `/src` directory within it!**

**_Save Stream Images Steps:_**
1. Navigate into the `/src` directory of your workspace and clone the repo using `git clone`
2. Navigate back into the workspace directory and source `$ source /opt/ros/foxy/setup.bash`
3. Build package `$ colcon build` or `$ colcon build --packages-select <package_name>`
4. Open a new terminal and source it `$ . install/setup.bash`
5. Run executable `$ ros2 run <package_name> <executable_name>` in this case it is `$ ros2 run utilities_package frame_saver`
6. In a new terminal navigate to the directory of your desired bag file and source it using `$ source /opt/ros/foxy/setup.bash`
7. Play bag file `$ ros2 bag play <bag_name>`

**Note: The topic name used to record the bag file MUST MATCH the topic name specified in `frame_saver.cpp`. For more information, check the Joplin documentation**

**_Write Video File Steps:_**
1. Open a new terminal in your working directory and source using `$ . install/setup.bash`
2. Run launch file `$ ros2 launch <package_name> <launch_file_name>` in this case it is `$ros2 launch utilities_package video_writer_launch.py`

## Additional Notes:

The remote repository will NOT have a `/frames` directory, thus you must create one in your local when freshly cloning! This is because frames are deleted after selections are copied to other directories or after they are written to a video file. This is to ensure there isn't a folder housing thousands of image frames for obvious reasons.

Also, currently I clear this folder manually. I had a way for frames to delete themselves automatically after saving but this caused issues as it would not save all frames from a bag file.