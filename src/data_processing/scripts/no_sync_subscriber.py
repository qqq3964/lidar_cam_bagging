#!/usr/bin/env python3
import rospy
import os
import cv2
from sensor_msgs.msg import CompressedImage, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np

IMAGE_SAVE_DIR = '/data/Tcar_hs/Image_no'
PCD_SAVE_DIR = '/data/Tcar_hs/PCD_no'

os.makedirs(IMAGE_SAVE_DIR, exist_ok=True)
os.makedirs(PCD_SAVE_DIR, exist_ok=True)

class DataSaver:
    def __init__(self):
        rospy.init_node("image_lidar_saver", anonymous=True)

        self.bridge = CvBridge()

        rospy.Subscriber("/camera_2/compressed", CompressedImage, self.image_callback, queue_size=10000, tcp_nodelay=True)
        rospy.Subscriber("/fusion_pc_final", PointCloud2, self.lidar_callback, queue_size=10000, tcp_nodelay=True)

        rospy.loginfo("Image and LiDAR subscribers initialized (no sync).")
        rospy.spin()

    def image_callback(self, image_msg):
        try:
            stamp = image_msg.header.stamp
            filename_base = f"{stamp.secs}_{stamp.nsecs}"

            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
            img_path = os.path.join(IMAGE_SAVE_DIR, f"{filename_base}.png")
            cv2.imwrite(img_path, cv_image)
            # rospy.loginfo(f"[✓] Saved image: {img_path}")
        except Exception as e:
            rospy.logerr(f"[X] Error saving image: {e}")

    def lidar_callback(self, lidar_msg):
        try:
            stamp = lidar_msg.header.stamp
            filename_base = f"{stamp.secs}_{stamp.nsecs}"

            points = np.array([
                [x, y, z]
                for x, y, z in pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)
            ], dtype=np.float32)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd_path = os.path.join(PCD_SAVE_DIR, f"{filename_base}.pcd")
            o3d.io.write_point_cloud(pcd_path, pcd)
            # rospy.loginfo(f"[✓] Saved PCD: {pcd_path}")
        except Exception as e:
            rospy.logerr(f"[X] Error saving PCD: {e}")

if __name__ == "__main__":
    DataSaver()

