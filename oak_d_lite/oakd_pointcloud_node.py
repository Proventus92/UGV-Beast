#!/usr/bin/env python3
# -*- coding: Windows-1252 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import depthai as dai
import numpy as np
import math

class OakDNode(Node):
    def __init__(self):
        super().__init__('oakd_publisher')
        
        self.declare_parameter('fps', 30.0)
        self.fps = self.get_parameter('fps').value
        
        # --- PARAMETRES DE FILTRAGE (COMMUNS) ---
        # On définit ici la "boîte" stricte que tu veux garder
        self.z_min = 0.15   # Distance min (m)
        self.z_max = 3.0    # Distance max (m)
        self.x_limit = 0.45 # Largeur (m) du couloir (+/- 0.45m autour du centre)
        self.y_min = -0.10  # Hauteur min (par rapport cam) - coupe sol
        self.y_max = 0.13   # Hauteur max (par rapport cam) - coupe plafond
        
        self.pipeline = dai.Pipeline()
        
        # --- INIT CAMERA ---
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

        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.initialConfig.setConfidenceThreshold(150)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
        
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
            self.get_logger().info("OAK-D : Mode Filtrage Unifié (Scan = Nuage)")
        except Exception as e:
            self.get_logger().error(f"Failed: {e}")
            return

        self.scan_pub = self.create_publisher(LaserScan, '/oak/scan', 10)
        self.pcl_pub = self.create_publisher(PointCloud2, '/oak/points', 10)
        
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def timer_callback(self):
        if not hasattr(self, 'q_pcl'): return
        in_pcl = self.q_pcl.tryGet()
        
        if in_pcl is not None:
            points = in_pcl.getPoints() 
            points = np.array(points).reshape(-1, 3)
            points = points / 1000.0 
            
            if len(points) == 0: return

            # ==========================================================
            # ETAPE 1 : FILTRAGE UNIQUE (Le "Découpage")
            # ==========================================================
            # On applique les filtres stricts immédiatement.
            # Tout ce qui ne passe pas ce filtre n'existera ni pour Rviz, ni pour Nav2.

            # 1. Z (Profondeur)
            mask_z = (points[:, 2] > self.z_min) & (points[:, 2] < self.z_max)
            # 2. X (Largeur - Couloir)
            mask_x = (points[:, 0] < self.x_limit) & (points[:, 0] > -self.x_limit)
            # 3. Y (Hauteur - Coupe sol/plafond)
            mask_y = (points[:, 1] < self.y_max) & (points[:, 1] > self.y_min)
            
            # Combinaison des masques
            final_mask = mask_z & mask_x & mask_y
            filtered_points = points[final_mask]

            # Préparation du Header commun
            header_opt = Header()
            header_opt.stamp = self.get_clock().now().to_msg()
            header_opt.frame_id = "oak_rgb_camera_optical_frame"

            # ==========================================================
            # ETAPE 2 : PUBLICATION POINTCLOUD (Visuel)
            # ==========================================================
            if len(filtered_points) > 0:
                pc2_msg = pc2.create_cloud_xyz32(header_opt, filtered_points)
                self.pcl_pub.publish(pc2_msg)

            # ==========================================================
            # ETAPE 3 : PUBLICATION LASERSCAN (Nav2)
            # ==========================================================
            # On utilise EXCLUSIVEMENT 'filtered_points' calculé au-dessus
            
            scan = LaserScan()
            scan.header.stamp = header_opt.stamp
            scan.header.frame_id = "base_link" 
            
            # Configuration du scan
            scan.angle_min = -0.65  # ~ -37 degrés
            scan.angle_max = 0.65   # ~ +37 degrés
            scan.angle_increment = 0.01 
            scan.range_min = self.z_min
            scan.range_max = self.z_max
            scan.time_increment = 0.0
            
            num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
            ranges = [float('inf')] * num_readings
            
            if len(filtered_points) > 0:
                # TRANSFORMATION : Camera Optical -> Robot Base
                # X_robot = Z_camera
                # Y_robot = -X_camera
                
                # Note : On prend filtered_points ici !
                robot_x = filtered_points[:, 2]      
                robot_y = -filtered_points[:, 0]     
                
                angles = np.arctan2(robot_y, robot_x)
                distances = np.sqrt(robot_x**2 + robot_y**2)
                
                # Remplissage optimal avec NumPy (plus rapide qu'une boucle for simple)
                # On calcule les indices correspondants aux angles
                indices = ((angles - scan.angle_min) / scan.angle_increment).astype(int)
                
                # Filtrage des indices valides (dans le champ de vision du scan)
                valid_indices_mask = (indices >= 0) & (indices < num_readings)
                
                # On ne traite que les points valides
                valid_indices = indices[valid_indices_mask]
                valid_distances = distances[valid_indices_mask]
                
                # Mise à jour des ranges : on garde la distance minimale pour chaque index
                # (Approche itérative simple pour être sûr de prendre le min)
                for idx, dist in zip(valid_indices, valid_distances):
                    if dist < ranges[idx]:
                        ranges[idx] = float(dist)
            
            scan.ranges = ranges
            self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = OakDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        try: node.destroy_node()
        except: pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
