#!/usr/bin/env python3
import rospy
import os
import cv2
from sensor_msgs.msg import CompressedImage, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

IMAGE_SAVE_DIR = '/data/Tcar_hs/Image'
PCD_SAVE_DIR = '/data/Tcar_hs/PCD'

os.makedirs(IMAGE_SAVE_DIR, exist_ok=True)
os.makedirs(PCD_SAVE_DIR, exist_ok=True)

class DataSaver:
    def __init__(self):
        rospy.init_node("image_lidar_saver", anonymous=True)

        self.bridge = CvBridge()

        image_sub = Subscriber("/camera_2/compressed", CompressedImage)
        lidar_sub = Subscriber("/fusion_pc_cur_cloud_rviz", PointCloud2)
        
        sync = ApproximateTimeSynchronizer([image_sub, lidar_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self.synced_callback)
        
        rospy.loginfo("Image and LiDAR subscribers initialized.")
        rospy.spin()

    def synced_callback(self, image_msg, lidar_msg):
        try:
            # Use ROS timestamp as a base filename
            stamp = image_msg.header.stamp
            filename_base = f"{stamp.secs}_{stamp.nsecs}"

            # === Save image ===
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            img_path = os.path.join(IMAGE_SAVE_DIR, f"{filename_base}.png")
            cv2.imwrite(img_path, cv_image)
            rospy.loginfo(f"[✓] Saved image: {img_path}")

            # === Convert and save point cloud ===
            points = np.array([
                [x, y, z]
                for x, y, z in pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)
            ], dtype=np.float32)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd_path = os.path.join(PCD_SAVE_DIR, f"{filename_base}.pcd")
            o3d.io.write_point_cloud(pcd_path, pcd)
            rospy.loginfo(f"[✓] Saved PCD: {pcd_path}")

        except Exception as e:
            rospy.logerr(f"[X] Error saving synced data: {e}")

if __name__ == "__main__":
    try:
        DataSaver()
    except rospy.ROSInterruptException:
        pass
