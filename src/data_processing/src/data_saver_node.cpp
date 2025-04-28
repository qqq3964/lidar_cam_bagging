#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <Eigen/Dense>

// message_filters for synchronization
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// Save directories for images and point clouds
std::string image_save_dir = "/data/velodyne/Image";
std::string pcd_save_dir   = "/data/velodyne/PCD";
int save_index = 0; 

// Callback for synchronized image and LiDAR messages
void syncedCallback(const sensor_msgs::ImageConstPtr& img_msg,
                    const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
{
    try {
        // Convert compressed image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;

        // Generate image filename based on timestamp
        std::stringstream img_ss;
        img_ss << image_save_dir << "/"
               << std::setw(6) << std::setfill('0') << save_index << ".png";
        cv::imwrite(img_ss.str(), image);

        // Convert PointCloud2 to PCL format and save
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pcl_msg, *cloud);

        // Generate PCD filename based on timestamp
        std::stringstream pcd_ss;
        pcd_ss << pcd_save_dir << "/"
               << std::setw(6) << std::setfill('0') << save_index << ".pcd";
        pcl::io::savePCDFileBinary(pcd_ss.str(), *cloud);
        save_index++; 
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_lidar_sync_saver");
    ros::NodeHandle nh;

    // Create output directories if they don't exist
    std::filesystem::create_directories(image_save_dir);
    std::filesystem::create_directories(pcd_save_dir);

    // Subscribe to image and point cloud topics using message_filters
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/usb_cam/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/velodyne_points", 10);

    // ApproximateTime synchronizer setup
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image,
        sensor_msgs::PointCloud2> SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, lidar_sub);
    sync.registerCallback(boost::bind(&syncedCallback, _1, _2));

    ROS_INFO("Synchronized image and LiDAR saving node started.");
    ros::spin();
    return 0;
}
