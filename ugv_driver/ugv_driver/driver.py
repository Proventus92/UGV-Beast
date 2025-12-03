#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import json
import math
import time

class UgvDriver(Node):
    def __init__(self):
        super().__init__('ugv_driver')
        
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 115200)
        
        # --- NOUVEAU PARAMÈTRE : L'INTERRUPTEUR ---
        # Par défaut à True (pour la navigation), mais on pourra le mettre à False
        self.declare_parameter('publish_tf', True) 
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.ANGULAR_FACTOR = 0.6 

        try:
            self.serial = serial.Serial(
                self.get_parameter('port').value, 
                self.get_parameter('baud').value, 
                timeout=1.0
            )
            self.init_robot()
        except Exception as e:
            self.get_logger().error(f"ERREUR SERIE: {e}")

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def init_robot(self):
        cmd = json.dumps({"T": 900, "main": 3, "module": 0}) + "\n"
        for _ in range(5):
            if hasattr(self, 'serial') and self.serial.is_open: 
                self.serial.write(cmd.encode('utf-8'))
                time.sleep(0.2)
        self.get_logger().info("Robot Initialise")

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

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Watchdog
        dt_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if dt_cmd > 1.0 and (abs(self.linear_cmd) > 0.0 or abs(self.angular_cmd) > 0.0):
            self.linear_cmd = 0.0
            self.angular_cmd = 0.0
            self.send_motor_command(0.0, 0.0)

        # Odométrie
        delta_x = (self.linear_cmd * math.cos(self.th)) * dt
        delta_y = (self.linear_cmd * math.sin(self.th)) * dt
        delta_th = (self.angular_cmd * self.ANGULAR_FACTOR) * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        q = self.euler_to_quaternion(0, 0, self.th)

        # --- GESTION INTELLIGENTE DE LA TF ---
        # On vérifie l'interrupteur avant d'envoyer !
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
        # -------------------------------------

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
