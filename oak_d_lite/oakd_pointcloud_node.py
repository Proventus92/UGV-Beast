#!/usr/bin/env python3
# -*- coding: Windows-1252 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import depthai as dai
import numpy as np
import sys

class OakDNode(Node):
    def __init__(self):
        super().__init__('oakd_publisher')
        
        # --- CORRECTION : Définition des FPS ---
        self.declare_parameter('fps', 10.0)
        self.fps = self.get_parameter('fps').value
        # ---------------------------------------
        
        # Pipeline DepthAI
        self.pipeline = dai.Pipeline()
        
        # --- CAMERAS ---
        if hasattr(dai.node, 'MonoCamera'):
            monoLeft = self.pipeline.create(dai.node.MonoCamera)
            monoRight = self.pipeline.create(dai.node.MonoCamera)
            monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        else:
            monoLeft = self.pipeline.create(dai.node.Camera)
            monoRight = self.pipeline.create(dai.node.Camera)
            monoLeft.setSize(640, 400)
            monoRight.setSize(640, 400)
        
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        # --- STEREO DEPTH (REGLAGE SENSIBILITE MAX) ---
        stereo = self.pipeline.create(dai.node.StereoDepth)
        
        # 1. HIGH_DENSITY : Obligatoire pour les objets fins
        if hasattr(dai.node.StereoDepth.PresetMode, 'HIGH_DENSITY'):
             stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        else:
             stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)

        # 2. CONFIANCE ELEVEE : On accepte plus de points (0-255). 
        # 200 est le défaut. 245 remplit les trous (pieds de chaises).
        stereo.initialConfig.setConfidenceThreshold(245)
        
        # 3. Rectification et Subpixel
        stereo.setLeftRightCheck(True) 
        stereo.setSubpixel(True)
        stereo.setExtendedDisparity(False) 

        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
        stereo.setRectification(True)
        
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        pointcloud = self.pipeline.create(dai.node.PointCloud)
        stereo.depth.link(pointcloud.inputDepth)
        
        xout = self.pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("pcl")
        pointcloud.outputPointCloud.link(xout.input)

        try:
            self.device = dai.Device(self.pipeline)
            self.q_pcl = self.device.getOutputQueue(name="pcl", maxSize=4, blocking=False)
            self.get_logger().info("OAK-D : Mode Haute Sensibilité (Pieds de chaise)")
        except Exception as e:
            self.get_logger().error(f"Failed to start OAK-D: {e}")
            return

        self.pcl_pub = self.create_publisher(PointCloud2, '/oak/points', 10)
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def timer_callback(self):
        if not hasattr(self, 'q_pcl'): return

        in_pcl = self.q_pcl.tryGet()
        
        if in_pcl is not None:
            points = in_pcl.getPoints() 
            points = np.array(points).reshape(-1, 3)
            points = points / 1000.0 
            
            # --- FILTRAGE LEGER ---
            
            # 1. Z (Distance) : On garde tout ce qui est proche
            mask_z = (points[:, 2] > 0.15) & (points[:, 2] < 3.0)
            
            # 2. X (Largeur) : On nettoie juste les bords extremes
            mask_x = (points[:, 0] < 0.45) & (points[:, 0] > -0.45)

            # 3. Y (Hauteur) - ON RELACHE LA CONTRAINTE
            # Caméra à 11cm (0.11). 
            # On accepte tout ce qui est plus haut que 13cm sous la caméra (Y < 0.13).
            # Cela veut dire qu'on VA VOIR un peu de sol si le sol n'est pas plat.
            # C'est nécessaire pour être sûr de voir le bas du pied de chaise.
            mask_y = (points[:, 1] < 0.13) & (points[:, 1] > -0.10)
            
            mask = mask_z & mask_x & mask_y
            filtered_points = points[mask]

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "oak_rgb_camera_optical_frame" 
            
            if len(filtered_points) > 0:
                pc2_msg = pc2.create_cloud_xyz32(header, filtered_points)
                self.pcl_pub.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OakDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except: pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()