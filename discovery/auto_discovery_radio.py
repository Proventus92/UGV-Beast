#!/usr/bin/env python3
# -*- coding: Windows-1252 -*-

import subprocess
import signal
import sys
import time
import os
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal

# --- CHEMIN D'ACCÈS VERS LE MODULE RADIO ---
# Correction du tiret : "ugv-master" au lieu de "ugv_master"
UGV_DIR = '/home/jetson/ugv-master/ugv'

if UGV_DIR not in sys.path:
    sys.path.append(UGV_DIR)

try:
    import tw
    import trellis_pb2
    RADIO_MODULES_LOADED = True
except ImportError as e:
    print(f"ATTENTION: Impossible d'importer les modules radio : {e}")
    print(f"-> Le script a cherché dans : {UGV_DIR}")
    RADIO_MODULES_LOADED = False

# --- VARIABLES GLOBALES RADIO ---
LATEST_RADIO_SIGNAL = 100.0
MIN_SIGNAL_THRESHOLD = 30.0

def radio_callback(ev):
    global LATEST_RADIO_SIGNAL
    try:
        for p in ev.payload:
            if hasattr(p, 'src_link_quality_percent'):
                LATEST_RADIO_SIGNAL = float(p.src_link_quality_percent)
    except Exception:
        pass

def start_radio_monitor():
    if not RADIO_MODULES_LOADED:
        return
    try:
        a = tw.trellisAPI()
        a.addMqttProtobuff("twt/tsm/v1/sa/node/links/+", trellis_pb2.Links, radio_callback)
        t = threading.Thread(target=a.run, daemon=True)
        t.start()
        print(">>> Moniteur Radio Trellis/MQTT démarré en arrière-plan.")
    except Exception as e:
        print(f">>> Erreur radio: {e}")

# --- NŒUD DE SÉCURITÉ ROS 2 ---
class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('radio_safety_monitor')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Ce client permet d'annuler l'objectif Nav2 en cours (donné via 2D Goal Pose dans RViz)
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        
        self.timer = self.create_timer(0.5, self.check_safety)
        self.is_recovering = False

    def check_safety(self):
        global LATEST_RADIO_SIGNAL
        
        # Si le signal chute sous 30% et qu'on n'est pas déjà en train de reculer
        if LATEST_RADIO_SIGNAL < MIN_SIGNAL_THRESHOLD and not self.is_recovering:
            self.get_logger().warn(f"SIGNAL CRITIQUE ({LATEST_RADIO_SIGNAL}%) ! Arrêt de la navigation.")
            self.is_recovering = True
            
            # 1. Annuler l'objectif RViz actuel dans Nav2
            if self.cancel_client.wait_for_service(timeout_sec=1.0):
                req = CancelGoal.Request()
                self.cancel_client.call_async(req)
                self.get_logger().info("Objectif RViz annulé avec succès.")
            
            # 2. Forcer une marche arrière douce pour récupérer le signal
            self.get_logger().info("Marche arrière de sécurité en cours...")
            self.reverse_robot()
            
            self.get_logger().info("Marche arrière terminée. En attente d'un nouvel ordre sur RViz.")
            self.is_recovering = False

    def reverse_robot(self):
        # On publie un ordre de recul sur /cmd_vel
        cmd = Twist()
        cmd.linear.x = -0.2  # Vitesse de recul pour tes chenilles
        
        # Recule pendant 1.5 seconde
        for _ in range(15):
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)
            
        # On arrête les moteurs
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)

def ros2_thread_func():
    rclpy.init()
    node = SafetyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main():
    print("--- DÉMARRAGE DISCOVERY (MODE MANUEL RViz) + SÉCURITÉ RADIO ---")
    
    # ---> FIX : On se déplace dans le dossier contenant les certificats radio
    original_dir = os.getcwd()
    try:
        os.chdir(UGV_DIR)
    except Exception as e:
        print(f"Impossible de se déplacer dans {UGV_DIR} : {e}")

    start_radio_monitor()
    
    # Lancement du Safety Node ROS 2 en arrière-plan
    ros_thread = threading.Thread(target=ros2_thread_func, daemon=True)
    ros_thread.start()

    # Lancement du robot et de Nav2
    launch_cmd = ["ros2", "launch", "my_robot_cartographer", "discovery.launch.py"]
    print(f"Lancement de discovery.launch.py...")
    
    process = subprocess.Popen(launch_cmd, start_new_session=True)

    # Récupération du script de sauvegarde (en utilisant le chemin absolu du script d'origine)
    script_path = os.path.dirname(os.path.abspath(__file__))
    save_script = os.path.join(script_path, "save_my_map.py")

    try:
        # Attente infinie (le robot attend tes "2D Goal Pose" sur RViz)
        process.wait()
        
    except KeyboardInterrupt:
        print("\n\nSTOP DEMANDÉ (Ctrl+C) !")
        print("ATTENTION : Sauvegarde de la carte en cours...")
        
        # Sauvegarde la carte
        if os.path.exists(save_script):
            try:
                subprocess.run(["python3", save_script], check=True)
                print("Sauvegarde terminée.")
            except Exception as e:
                print(f"Erreur pendant la sauvegarde : {e}")
        
        print("Extinction du système ROS 2...")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=5)
        except:
            try: os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except: pass
        print("Terminé.")

if __name__ == "__main__":
    main()
