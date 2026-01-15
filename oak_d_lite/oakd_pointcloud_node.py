#!/usr/bin/env python3
# -*- coding: Windows-1252 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import depthai as dai
import numpy as np

class OakDNode(Node):
    def __init__(self):
        super().__init__('oakd_publisher')
        
        self.declare_parameter('fps', 30.0)
        self.fps = self.get_parameter('fps').value
        
        # --- PARAMETRES DE FILTRAGE (CRITIQUES) ---
        # 1. Distance MIN : 50cm. 
        # Le robot est "aveugle" en dessous. Nav2 DOIT l'empêcher d'approcher autant.
        self.z_min = 0.50   
        self.z_max = 4.0    # On voit jusqu'à 4m
        
        # 2. Largeur de vue (Couloir)
        self.x_limit = 1.0  # On regarde large (1m de chaque côté) pour bien contourner
        
        # 3. Hauteur (Sol/Plafond)
        self.y_min = -0.10  # On ignore le sol
        self.y_max = 0.50   # On regarde les obstacles jusqu'à 50cm de haut (ou plus selon tes besoins)
        
        self.pipeline = dai.Pipeline()
        
        # --- INIT CAMERA (Mono ou RGB selon dispo) ---
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
        stereo.initialConfig.setConfidenceThreshold(200)
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
            self.get_logger().info(f"OAK-D : Filtre Z_MIN={self.z_min}m (Zone Morte)")
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

            # --- FILTRAGE STRICT ---
            # On enlève tout ce qui est < 50cm (Bruit/Sol proche)
            mask_z = (points[:, 2] > self.z_min) & (points[:, 2] < self.z_max)
            mask_x = (points[:, 0] < self.x_limit) & (points[:, 0] > -self.x_limit)
            mask_y = (points[:, 1] < self.y_max) & (points[:, 1] > self.y_min)
            
            filtered_points = points[mask_z & mask_x & mask_y]

            header_opt = Header()
            header_opt.stamp = self.get_clock().now().to_msg()
            header_opt.frame_id = "oak_rgb_camera_optical_frame"

            # 1. Publish PointCloud (Visuel)
            if len(filtered_points) > 0:
                pc2_msg = pc2.create_cloud_xyz32(header_opt, filtered_points)
                self.pcl_pub.publish(pc2_msg)

            # 2. Publish LaserScan (Pour Nav2 - Obstacles)
            scan = LaserScan()
            scan.header.stamp = header_opt.stamp
            scan.header.frame_id = "base_link" # Scan horizontal robot
            
            # Champ de vision ~70°
            scan.angle_min = -0.65 
            scan.angle_max = 0.65 
            scan.angle_increment = 0.01 
            # RANGE MIN DOIT CORRESPONDRE AU FILTRE Z
            scan.range_min = self.z_min 
            scan.range_max = self.z_max
            scan.time_increment = 0.0
            
            num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
            ranges = [float('inf')] * num_readings
            
            if len(filtered_points) > 0:
                # Rotation repère: Z_cam -> X_robot, -X_cam -> Y_robot
                robot_x = filtered_points[:, 2]      
                robot_y = -filtered_points[:, 0]     
                
                angles = np.arctan2(robot_y, robot_x)
                distances = np.sqrt(robot_x**2 + robot_y**2)
                
                indices = ((angles - scan.angle_min) / scan.angle_increment).astype(int)
                valid_mask = (indices >= 0) & (indices < num_readings)
                
                valid_indices = indices[valid_mask]
                valid_distances = distances[valid_mask]
                
                # On garde la distance min pour chaque angle
                for idx, dist in zip(valid_indices, valid_distances):
                    if dist < ranges[idx]:
                        ranges[idx] = float(dist)
            
            scan.ranges = ranges
            self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = OakDNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        try: node.destroy_node()
        except: pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()

