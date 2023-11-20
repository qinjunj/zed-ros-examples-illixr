///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**
 * This tutorial demonstrates how to receive the Left and Right rectified images
 * from the ZED node
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <fstream>
#include <boost/filesystem.hpp>

const boost::filesystem::path imu_path = "/home/qinjun/Research/dataset/imu0";
std::ofstream imu_wt_file;
const boost::filesystem::path cam0_path = "/home/qinjun/Research/dataset/cam0";
const boost::filesystem::path cam1_path = "/home/qinjun/Research/dataset/cam1";
const boost::filesystem::path cam0_data_path = "/home/qinjun/Research/dataset/cam0/data";
const boost::filesystem::path cam1_data_path = "/home/qinjun/Research/dataset/cam1/data";
std::ofstream cam0_wt_file;
std::ofstream cam1_wt_file;

/**
 * Subscriber callbacks. The argument of the callback is a constant pointer to the received message
 */
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_INFO("Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, msg->angular_velocity.x,
            msg->angular_velocity.y, msg->angular_velocity.z, msg->orientation.x, msg->orientation.y, msg->orientation.z,
            msg->orientation.w);
    
    ros::Time timestamp = msg->header.stamp;
    int64_t timestamp_ns = timestamp.toNSec();

    imu_wt_file << timestamp_ns << "," << msg->angular_velocity.x << ","
                                       << msg->angular_velocity.y << ","
                                       << msg->angular_velocity.z << ","
                                       << msg->linear_acceleration.x << ","
                                       << msg->linear_acceleration.y << ","
                                       << msg->linear_acceleration.z << std::endl;
}

void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ros::Time timestamp = msg->header.stamp;
    int64_t timestamp_ns = timestamp.toNSec();

    try
    {
        // Convert the ROS image message to an OpenCV image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

        // Save the rectified grayscale image to a file
        std::string filename = cam0_data_path.string() + "/" + std::to_string(timestamp_ns) + ".png";
        cv::imwrite(filename, cv_ptr->image);

        cam0_wt_file << timestamp_ns << "," << std::to_string(timestamp_ns) + ".png" << std::endl;

        ROS_INFO("Rectified grayscale image saved at time %ld", timestamp_ns);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("CV_Bridge Exception: %s", e.what());
    }

}

void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ros::Time timestamp = msg->header.stamp;
    int64_t timestamp_ns = timestamp.toNSec();

    try
    {
        // Convert the ROS image message to an OpenCV image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

        // Save the rectified grayscale image to a file
        std::string filename = cam1_data_path.string() + "/" + std::to_string(timestamp_ns) + ".png";
        cv::imwrite(filename, cv_ptr->image);

        cam1_wt_file << timestamp_ns << "," << std::to_string(timestamp_ns) + ".png" << std::endl;

        ROS_INFO("Rectified grayscale image saved at time %ld", timestamp_ns);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("CV_Bridge Exception: %s", e.what());
    }
    
}

/**
 * Node main function
 */
int main(int argc, char** argv) {

    // Node initialization
    ros::init(argc, argv, "zed_video_subscriber");
    ros::NodeHandle n;

    // Subscribers
    ros::Subscriber subRightRectified = n.subscribe("/zedm/zed_node/right/image_rect_gray", 10,
                                                    imageRightRectifiedCallback);
    ros::Subscriber subLeftRectified  = n.subscribe("/zedm/zed_node/left/image_rect_gray", 10,
                                                    imageLeftRectifiedCallback);

    ros::Subscriber subImu = n.subscribe("/zedm/zed_node/imu/data", 100,
                                                    imuCallback);

    boost::filesystem::create_directories(imu_path);
    std::string imu_file = imu_path.string() + "/data.csv";
    imu_wt_file.open(imu_file, std::ofstream::out);
    imu_wt_file << "#timestamp [ns],w_x [rad s^-1],w_y [rad s^-1],w_z [rad s^-1],a_x [m s^-2],a_y [m s^-2],a_z [m s^-2]"
                    << std::endl;
    boost::filesystem::create_directories(cam0_path);
    boost::filesystem::create_directories(cam1_path);
    boost::filesystem::create_directories(cam0_data_path);
    boost::filesystem::create_directories(cam1_data_path);
    std::string cam0_file = cam0_path.string() + "/data.csv";
    std::string cam1_file = cam1_path.string() + "/data.csv";
    cam0_wt_file.open(cam0_file, std::ofstream::out);
    cam1_wt_file.open(cam1_file, std::ofstream::out);
    cam0_wt_file << "#timestamp [ns],filename" << std::endl;
    cam1_wt_file << "#timestamp [ns],filename" << std::endl;

    // Node execution
    ros::spin();

    return 0;
}
