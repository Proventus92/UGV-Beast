#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

import serial
import json
import math
import time
import subprocess
import os
import signal
from pymavlink import mavutil

class MavrosDriver(Node):
    def __init__(self):
        super().__init__('mavros_driver')
        
        self.declare_parameter('port', '/dev/ttyACM0') 
        self.declare_parameter('baud', 115200)
        # REMIS A TRUE COMME DANS VOTRE CODE D'ORIGINE
        self.declare_parameter('publish_tf', True) 
        
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.last_time = self.get_clock().now()
        self.start_time_ns = self.get_clock().now().nanoseconds
        self.last_cmd_time = self.get_clock().now()         
        self.last_manual_cmd_time = self.get_clock().now()  
        
        self.MANUAL_TIMEOUT = 2.0 
        self.linear_cmd = 0.0; self.angular_cmd = 0.0; self.last_gz = 0.0
        self.ANGULAR_FACTOR = 0.6
        
        self.is_armed = False
        self.battery_voltage = 12.6
        self.battery_percent = 100
        
        self.custom_mode = 65536 
        self.autonomous_process = None

        self.fake_params = {
            'CAL_GYRO0_ID': 1.0, 'COM_RC_IN_MODE': 1.0, 'RC_MAP_ROLL': 1.0,
            'SYS_AUTOCONFIG': 0.0, 'MAV_SYS_ID': 1.0, 'CAL_ACC0_ID': 1.0,
            'CAL_MAG2_ID': 1.0, 'CAL_MAG1_ID': 1.0, 'CAL_MAG0_ID': 1.0,
            'RC_MAP_AUX2': 0.0, 'RC_MAP_AUX1': 0.0, 'RC_MAP_FLAPS': 0.0,
            'RC_MAP_THROTTLE': 3.0, 'RC_MAP_YAW': 4.0, 'RC_MAP_PITCH': 2.0,
            'COM_FLTMODE1': 0.0, 'COM_FLTMODE2': 0.0, 'COM_FLTMODE3': 0.0,
            'COM_FLTMODE4': 0.0, 'COM_FLTMODE5': 0.0, 'COM_FLTMODE6': 0.0,
            'BAT1_SOURCE': 0.0, 'RTL_LAND_DELAY': 0.0, 'RTL_DESCEND_ALT': 0.0,
            'RTL_RETURN_ALT': 0.0, 'NAV_DLL_ACT': 0.0, 'COM_RC_LOSS_T': 0.0,
            'NAV_RCL_ACT': 0.0, 'COM_LOW_BAT_ACT': 0.0,
            'SYS_AUTOSTART': 4001.0, 'COM_ARM_WO_GPS': 1.0
        }

        try:
            self.serial = serial.Serial(self.get_parameter('port').value, self.get_parameter('baud').value, timeout=1.0)
            self.init_robot()
        except Exception as e:
            self.get_logger().error(f"SERIAL ERROR: {e}")

        try:
            self.mav_conn = mavutil.mavlink_connection('udpout:127.0.0.1:14551', source_system=1, source_component=1)
        except Exception as e:
            self.get_logger().error(f"MAVLINK ERROR: {e}")

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.read_timer = self.create_timer(0.02, self.read_serial_data)
        self.mav_timer = self.create_timer(0.2, self.mavlink_periodic_send)
        self.mav_read_timer = self.create_timer(0.02, self.read_mavlink_data)

    def read_mavlink_data(self):
        if not hasattr(self, 'mav_conn'): return
        try:
            while True:
                msg = self.mav_conn.recv_match(blocking=False)
                if not msg: break
                
                msg_type = msg.get_type()
                
                if msg_type == 'MANUAL_CONTROL':
                    x_norm = msg.x / 1000.0
                    rot_norm = msg.r / 1000.0 if msg.r != 0 else msg.z / 1000.0
                    
                    if abs(x_norm) > 0.05 or abs(rot_norm) > 0.05:
                        self.last_manual_cmd_time = self.get_clock().now()
                        self.last_cmd_time = self.get_clock().now()
                        
                        if self.is_armed:
                            self.linear_cmd = x_norm * 0.5
                            self.angular_cmd = -rot_norm * 1.0 
                            self.send_motor_command(self.linear_cmd, self.angular_cmd)
                    else:
                        if self.custom_mode == 65536:
                            self.last_cmd_time = self.get_clock().now()
                            if self.is_armed:
                                self.send_motor_command(0.0, 0.0)

                elif msg_type == 'PARAM_REQUEST_LIST':
                    count = len(self.fake_params)
                    for i, (key, val) in enumerate(self.fake_params.items()):
                        self.mav_conn.mav.param_value_send(
                            key.encode('utf-8'), float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32, count, i
                        )
                
                elif msg_type == 'PARAM_REQUEST_READ':
                    req_id = ""
                    idx = -1
                    if hasattr(msg, 'param_index') and msg.param_index >= 0 and msg.param_index < len(self.fake_params):
                        req_id = list(self.fake_params.keys())[msg.param_index]
                        idx = msg.param_index
                    elif hasattr(msg, 'param_id'):
                        raw_id = msg.param_id
                        req_id = raw_id.decode('utf-8') if isinstance(raw_id, bytes) else str(raw_id)
                        req_id = req_id.strip('\x00')
                        if req_id in self.fake_params:
                            idx = list(self.fake_params.keys()).index(req_id)
                    
                    if req_id in self.fake_params:
                        self.mav_conn.mav.param_value_send(
                            req_id.encode('utf-8'), float(self.fake_params[req_id]), mavutil.mavlink.MAV_PARAM_TYPE_REAL32, len(self.fake_params), idx
                        )
                
                elif msg_type == 'PARAM_SET':
                    raw_id = msg.param_id
                    req_id = raw_id.decode('utf-8') if isinstance(raw_id, bytes) else str(raw_id)
                    req_id = req_id.strip('\x00')
                    self.fake_params[req_id] = float(msg.param_value)
                    idx = list(self.fake_params.keys()).index(req_id)
                    self.mav_conn.mav.param_value_send(
                        req_id.encode('utf-8'), float(msg.param_value), msg.param_type, len(self.fake_params), idx
                    )

                elif msg_type == 'SET_MODE':
                    self.custom_mode = msg.custom_mode
                    self._check_and_launch_mission()

                elif msg_type == 'COMMAND_LONG':
                    self.mav_conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        self.is_armed = (msg.param1 == 1.0)
                    elif msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                        self.custom_mode = int(msg.param2)
                        self._check_and_launch_mission()
                    elif msg.command == mavutil.mavlink.MAV_CMD_MISSION_START:
                        self._start_autonomous_mode()

                elif msg_type == 'MISSION_REQUEST_LIST':
                    target_sys = getattr(msg, 'target_system', 255)
                    target_comp = getattr(msg, 'target_component', 0)
                    self.mav_conn.mav.mission_count_send(target_sys, target_comp, 0)
                    
        except Exception: pass

    def _check_and_launch_mission(self):
        if self.custom_mode != 65536:
            self._start_autonomous_mode()
        else:
            self._stop_autonomous_mode()

    def _start_autonomous_mode(self):
        self.get_logger().info("=========================================")
        self.get_logger().info("=> MODE AUTONOME ACTIVE ! Lancement... <=")
        self.get_logger().info("=========================================")
        if self.autonomous_process is None:
            try:
                # NOUVEAU: preexec_fn=os.setsid permet de grouper tous les sous-processus ROS 2 !
                self.autonomous_process = subprocess.Popen(
                    ["ros2", "launch", "my_robot_cartographer", "mission.launch.py"],
                    preexec_fn=os.setsid
                )
            except Exception as e:
                self.get_logger().error(f"Erreur lancement: {e}")

    def _stop_autonomous_mode(self):
        self.get_logger().info("=> RETOUR EN MODE MANUEL. Arret de l'autonomie...")
        if self.autonomous_process:
            try:
                # NOUVEAU: On envoie le signal d'arret a TOUT LE GROUPE (Fini les zombies !)
                os.killpg(os.getpgid(self.autonomous_process.pid), signal.SIGINT)
            except Exception as e:
                pass
            self.autonomous_process = None

    def mavlink_periodic_send(self):
        if hasattr(self, 'mav_conn'):
            base_mode = mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            system_status = mavutil.mavlink.MAV_STATE_STANDBY
            if self.is_armed:
                base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                system_status = mavutil.mavlink.MAV_STATE_ACTIVE

            self.mav_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GROUND_ROVER, mavutil.mavlink.MAV_AUTOPILOT_PX4, 
                base_mode, self.custom_mode, system_status
            )
            
            sensors_ok = 32767 
            safe_voltage = max(0.0, min(65.5, self.battery_voltage))
            voltage_mv = int(safe_voltage * 1000)
            
            self.mav_conn.mav.sys_status_send(
                sensors_ok, sensors_ok, sensors_ok, 20, voltage_mv, -1, self.battery_percent, 0, 0, 0, 0, 0, 0
            )
            
            voltages = [voltage_mv] + [65535] * 9
            self.mav_conn.mav.battery_status_send(
                0, mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL, mavutil.mavlink.MAV_BATTERY_TYPE_LIPO, 
                32767, voltages, -1, -1, -1, self.battery_percent
            )
            
            now_ns = self.get_clock().now().nanoseconds
            time_boot_ms = int((now_ns - self.start_time_ns) / 1e6) % 4294967295
            self.mav_conn.mav.attitude_send(time_boot_ms, 0.0, 0.0, self.th, 0.0, 0.0, 0.0)

    def init_robot(self):
        cmd_type = json.dumps({"T": 900, "main": 3, "module": 0}, separators=(',', ':')) + "\n"
        cmd_feed = json.dumps({"T": 605, "cmd": 1}, separators=(',', ':')) + "\n"
        if hasattr(self, 'serial') and self.serial.is_open: 
            for _ in range(3):
                self.serial.write(cmd_type.encode('utf-8')); time.sleep(0.1)
                self.serial.write(cmd_feed.encode('utf-8')); time.sleep(0.1)

    def send_motor_command(self, linear, angular):
        left = max(min(linear - angular, 1.0), -1.0)
        right = max(min(linear + angular, 1.0), -1.0)
        cmd = {"T": 1, "L": round(left, 3), "R": round(right, 3)}
        data = json.dumps(cmd, separators=(',', ':')) + "\n"
        try:
            if hasattr(self, 'serial') and self.serial.is_open: 
                self.serial.write(data.encode('utf-8'))
        except: pass

    def cmd_vel_callback(self, msg):
        if not self.is_armed:
            return
            
        if (self.get_clock().now() - self.last_manual_cmd_time).nanoseconds / 1e9 < self.MANUAL_TIMEOUT: 
            return
            
        self.last_cmd_time = self.get_clock().now()
        self.linear_cmd = msg.linear.x
        self.angular_cmd = msg.angular.z * 0.6 
        self.send_motor_command(self.linear_cmd, self.angular_cmd)

    def read_serial_data(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            try:
                while self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if not line or not (line.startswith('{') and line.endswith('}')): continue
                    data = json.loads(line)
                    
                    if "V" in data:
                        self.battery_voltage = float(data["V"])
                    elif "v" in data:
                        self.battery_voltage = float(data["v"])
                        
                    v = self.battery_voltage
                    if v >= 12.3:
                        self.battery_percent = 100
                    elif v >= 12.0:
                        self.battery_percent = int(80 + ((v - 12.0) / 0.3) * 20)
                    elif v >= 11.4:
                        self.battery_percent = int(30 + ((v - 11.4) / 0.6) * 50)
                    elif v >= 9.0:
                        self.battery_percent = int(((v - 9.0) / 2.4) * 30)
                    else:
                        self.battery_percent = 0

                    if "gx" in data and "gz" in data:
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = "imu_link"
                        deg_to_rad = math.pi / 180.0
                        
                        raw_gz = float(data["gz"])
                        if abs(self.angular_cmd) < 0.01: 
                            filtered_gz = 0.0
                        else:
                            if abs(raw_gz) < 1.0: raw_gz = 0.0
                            alpha = 0.6
                            filtered_gz = (alpha * self.last_gz) + ((1.0 - alpha) * raw_gz)
                        
                        self.last_gz = filtered_gz
                        
                        # REMIS EXACTEMENT COMME DANS VOTRE driver.py Uploade
                        imu_msg.angular_velocity.z = filtered_gz * deg_to_rad
                        
                        imu_msg.linear_acceleration.x = 0.0
                        imu_msg.linear_acceleration.y = 0.0
                        imu_msg.linear_acceleration.z = 9.81
                        
                        imu_msg.orientation_covariance[0] = -1.0
                        imu_msg.angular_velocity_covariance[0] = 0.1
                        imu_msg.linear_acceleration_covariance[0] = 0.1
                        
                        self.imu_pub.publish(imu_msg)
            except Exception: pass

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if (current_time - self.last_cmd_time).nanoseconds / 1e9 > 1.0:
            self.linear_cmd = 0.0
            self.angular_cmd = 0.0
            self.send_motor_command(0.0, 0.0)
            
        delta_x = (self.linear_cmd * math.cos(self.th)) * dt
        delta_y = (self.linear_cmd * math.sin(self.th)) * dt
        delta_th = (self.angular_cmd * self.ANGULAR_FACTOR) * dt 

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        q = self.euler_to_quaternion(0, 0, self.th)
        
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

        if self.get_parameter('publish_tf').value:
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

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = MavrosDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.autonomous_process:
            try:
                os.killpg(os.getpgid(node.autonomous_process.pid), signal.SIGINT)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
