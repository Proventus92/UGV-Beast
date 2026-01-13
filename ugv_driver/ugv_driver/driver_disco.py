#!/usr/bin/env python3
# -*- coding: Windows-1252 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String # Nécessaire pour les commandes Disco (JSON)
from tf2_ros import TransformBroadcaster
import serial
import json
import math
import time

class UgvDriver(Node):
    def __init__(self):
        super().__init__('ugv_driver')
        
        # --- PARAMETRES ---
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 115200)
        # Par défaut True, mais votre launch file le passera à False pour Cartographer
        self.declare_parameter('publish_tf', True) 
        
        # --- VARIABLES ODOMETRIE ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.ANGULAR_FACTOR = 0.6 
        
        # --- VARIABLES IMU & CALIBRATION ---
        self.last_gz = 0.0
        self.calibrating = True
        self.calibration_samples = 0
        self.calibration_sum_ax = 0.0
        self.calibration_sum_ay = 0.0
        self.calibration_sum_gz = 0.0
        self.offset_ax = 0.0
        self.offset_ay = 0.0
        self.offset_gz = 0.0
        self.TARGET_CALIBRATION_SAMPLES = 100 # 1 à 2 secondes d'immmobilité

        # --- INITIALISATION SERIE ---
        try:
            self.serial = serial.Serial(
                self.get_parameter('port').value, 
                self.get_parameter('baud').value, 
                timeout=1.0
            )
            self.init_robot()
        except Exception as e:
            self.get_logger().error(f"SERIAL ERROR: {e}")

        # --- PUBLISHERS ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # --- SUBSCRIBERS ---
        # 1. Commandes de déplacement (Nav2 / Teleop)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # 2. Commandes spéciales (Disco : LEDs, Servos, etc.)
        self.json_sub = self.create_subscription(String, 'json_cmd', self.json_callback, 10)
        
        # --- TIMERS ---
        self.timer = self.create_timer(0.05, self.timer_callback)       # Boucle Odom (20Hz)
        self.read_timer = self.create_timer(0.01, self.read_serial_data) # Boucle Lecture (100Hz)

    def init_robot(self):
        # Initialisation du robot pour recevoir les données IMU brutes
        cmd_type = json.dumps({"T": 900, "main": 3, "module": 0}) + "\n"
        cmd_feed = json.dumps({"T": 605, "cmd": 1}) + "\n"
        if hasattr(self, 'serial') and self.serial.is_open: 
            for _ in range(3):
                self.serial.write(cmd_type.encode('utf-8'))
                time.sleep(0.1)
                self.serial.write(cmd_feed.encode('utf-8'))
                time.sleep(0.1)
        self.get_logger().info("Robot Initialized. Starting CALIBRATION (Do not move)...")

    # --- NOUVELLE FONCTION : Gestion des commandes brutes (Disco) ---
    def json_callback(self, msg):
        try:
            command_str = msg.data
            # On s'assure qu'il y a un saut de ligne à la fin
            if not command_str.endswith('\n'):
                command_str += '\n'
            
            if hasattr(self, 'serial') and self.serial.is_open:
                self.serial.write(command_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error sending JSON command: {e}")

    def send_motor_command(self, linear, angular):
        left = max(min(linear - angular, 0.5), -0.5)
        right = max(min(linear + angular, 0.5), -0.5)
        
        cmd = {"T": 1, "L": round(left, 3), "R": round(right, 3)}
        data = json.dumps(cmd) + "\n"
        try:
            if hasattr(self, 'serial') and self.serial.is_open: 
                self.serial.write(data.encode('utf-8'))
        except: pass

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.linear_cmd = msg.linear.x
        self.angular_cmd = msg.angular.z * 0.6 
        self.send_motor_command(self.linear_cmd, self.angular_cmd)

    def read_serial_data(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            try:
                while self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue
                    if line.startswith('{') and line.endswith('}'):
                        data = json.loads(line)
                        
                        # On cherche les données IMU (gyro gz et accel ax)
                        if "gz" in data and "ax" in data:
                            raw_ax = float(data["ax"])
                            raw_ay = float(data["ay"])
                            raw_az = float(data["az"])
                            raw_gz = float(data["gz"])

                            # --- PHASE DE CALIBRATION ---
                            if self.calibrating:
                                self.calibration_sum_ax += raw_ax
                                self.calibration_sum_ay += raw_ay
                                self.calibration_sum_gz += raw_gz
                                self.calibration_samples += 1
                                
                                if self.calibration_samples >= self.TARGET_CALIBRATION_SAMPLES:
                                    self.offset_ax = self.calibration_sum_ax / self.TARGET_CALIBRATION_SAMPLES
                                    self.offset_ay = self.calibration_sum_ay / self.TARGET_CALIBRATION_SAMPLES
                                    self.offset_gz = self.calibration_sum_gz / self.TARGET_CALIBRATION_SAMPLES
                                    self.calibrating = False
                                    self.get_logger().info(f"CALIBRATION DONE. Offsets -> ax:{self.offset_ax:.2f}, gz:{self.offset_gz:.2f}")
                                return 

                            # --- PUBLICATION NORMALE ---
                            imu_msg = Imu()
                            imu_msg.header.stamp = self.get_clock().now().to_msg()
                            imu_msg.header.frame_id = "imu_link"
                            deg_to_rad = math.pi / 180.0
                            
                            # 1. GYRO (Avec retrait de l'offset)
                            clean_gz = raw_gz - self.offset_gz
                            alpha = 0.7
                            filtered_gz = (alpha * self.last_gz) + ((1.0 - alpha) * clean_gz)
                            self.last_gz = filtered_gz
                            
                            imu_msg.angular_velocity.z = filtered_gz * deg_to_rad
                            if "gx" in data: imu_msg.angular_velocity.x = float(data["gx"]) * deg_to_rad
                            if "gy" in data: imu_msg.angular_velocity.y = float(data["gy"]) * deg_to_rad

                            # 2. ACCEL (Retrait offset + Correction echelle G -> m/s^2)
                            acc_x_g = raw_ax - self.offset_ax
                            acc_y_g = raw_ay - self.offset_ay
                            acc_z_g = raw_az 
                            
                            imu_msg.linear_acceleration.x = acc_x_g * 9.81
                            imu_msg.linear_acceleration.y = acc_y_g * 9.81
                            imu_msg.linear_acceleration.z = acc_z_g * 9.81
                            
                            # Covariances pour Cartographer
                            imu_msg.orientation_covariance[0] = -1
                            imu_msg.angular_velocity_covariance[0] = 0.01
                            imu_msg.linear_acceleration_covariance[0] = 0.1
                            
                            self.imu_pub.publish(imu_msg)

            except Exception as e:
                pass

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Timeout commande (Sécurité)
        dt_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if dt_cmd > 1.0:
            self.linear_cmd = 0.0
            self.angular_cmd = 0.0
            self.send_motor_command(0.0, 0.0)

        # Calcul Odométrie (Dead Reckoning basé sur commande)
        delta_x = (self.linear_cmd * math.cos(self.th)) * dt
        delta_y = (self.linear_cmd * math.sin(self.th)) * dt
        delta_th = (self.angular_cmd * self.ANGULAR_FACTOR) * dt 

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        q = self.euler_to_quaternion(0, 0, self.th)

        # Gestion TF (Conditionnelle)
        if self.get_parameter('publish_tf').value == True:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        # Publication Odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.linear_cmd
        odom.twist.twist.angular.z = self.angular_cmd * self.ANGULAR_FACTOR
        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = UgvDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()