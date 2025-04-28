#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>
#include <sstream>
#include <iomanip>

// message_filters for synchronization
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// Save directories
std::string image_save_dir = "/data/velodyne/Image";
std::string pcd_save_dir = "/data/velodyne/PCD";
int save_index = 0;

// Latest synchronized messages
sensor_msgs::ImageConstPtr latest_img_msg;
sensor_msgs::PointCloud2ConstPtr latest_pcl_msg;

// Latest converted data
cv::Mat latest_image;
pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud(new pcl::PointCloud<pcl::PointXYZI>);

// Convert PointXYZI to PointXYZRGB (intensity to brightness)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertIntensityToRGB(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& p : cloud_in->points) {
        pcl::PointXYZRGB prgb;
        prgb.x = p.x;
        prgb.y = p.y;
        prgb.z = p.z;

        uint8_t intensity = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, p.intensity * 255.0f)));
        prgb.r = intensity;
        prgb.g = intensity;
        prgb.b = intensity;
        cloud_out->points.push_back(prgb);
    }
    cloud_out->width = static_cast<uint32_t>(cloud_out->points.size());
    cloud_out->height = 1;
    cloud_out->is_dense = true;
    return cloud_out;
}

// Synchronization callback
void syncedCallback(const sensor_msgs::ImageConstPtr& img_msg,
                    const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
{
    try {
        latest_img_msg = img_msg;
        latest_pcl_msg = pcl_msg;

        // Convert image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        latest_image = cv_ptr->image;

        // Convert point cloud
        pcl::fromROSMsg(*pcl_msg, *latest_cloud);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// Save captured data
void saveData() {
    if (latest_image.empty() || latest_cloud->empty()) {
        ROS_WARN("No valid synchronized data to save!");
        return;
    }

    // Save image
    std::stringstream img_ss;
    img_ss << image_save_dir << "/"
           << std::setw(6) << std::setfill('0') << save_index << ".png";
    cv::imwrite(img_ss.str(), latest_image);

    // Save point cloud
    std::stringstream pcd_ss;
    pcd_ss << pcd_save_dir << "/"
           << std::setw(6) << std::setfill('0') << save_index << ".pcd";
    pcl::io::savePCDFileBinary(pcd_ss.str(), *latest_cloud);

    ROS_INFO_STREAM("Saved Image and PCD [" << save_index << "]");
    save_index++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_capture_save_gui_node");
    ros::NodeHandle nh;

    // Create output directories
    std::filesystem::create_directories(image_save_dir);
    std::filesystem::create_directories(pcd_save_dir);

    // Subscribers with message_filters
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/usb_cam/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/velodyne_points", 10);

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image,
        sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, lidar_sub);
    sync.registerCallback(boost::bind(&syncedCallback, _1, _2));

    // PCLVisualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Intensity Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    ROS_INFO("Synchronized capture/save GUI node started.");

    while (ros::ok()) {
        ros::spinOnce(); // Process incoming messages

        if (!latest_image.empty() && !latest_cloud->empty()) {
            // Show image
            cv::imshow("Captured Image", latest_image);

            // Update PCL Viewer
            if (viewer->contains("cloud")) {
                viewer->removePointCloud("cloud");
            }

            auto cloud_rgb = convertIntensityToRGB(latest_cloud);
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, "cloud");

            viewer->spinOnce(10);

            // Wait indefinitely for key input
            int key = cv::waitKey(1);

            if (key == 'q' || key == 'Q') {
                ROS_WARN("Exit requested by user (q pressed).");
                ros::shutdown();
                break;
            }
            
            if (key == 's' || key == 'S'){
                saveData();
            };
        }

        if (viewer->wasStopped()) {
            ROS_WARN("PCL Viewer window closed. Exiting...");
            ros::shutdown();
            break;
        }
    }

    return 0;
}
