# -*- coding: Windows-1252 -*-
#!/usr/bin/env python3

import time
import os
import subprocess
import signal
import sys
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# --- IMPORT DE L'API RADIO (TRELLIS / MQTT) ---
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
# On suppose que le dossier "ugv" est au meme niveau ou juste au dessus
UGV_DIR = os.path.join(os.path.dirname(CURRENT_DIR), 'ugv') 
sys.path.append(UGV_DIR)

# Si ton dossier ugv est a un endroit tres specifique sur la Jetson, 
# tu peux forcer le chemin en decommentant et modifiant la ligne ci-dessous :
# sys.path.append('/home/jetson/ugv-master/ugv')

try:
    import tw
    import trellis_pb2
    RADIO_MODULES_LOADED = True
except ImportError as e:
    print(f"ATTENTION: Impossible d'importer les modules radio : {e}")
    RADIO_MODULES_LOADED = False

# --- CONFIGURATION UTILISATEUR ---
MIN_FRONTIER_SIZE = 4        
MAX_RETRIES = 5              
RETRY_DELAY = 1.0            
SAFETY_DISTANCE = 0.45       
BLACKLIST_RADIUS = 0.80      
INITIAL_WAIT = 5.0           
HOME_POSE = [0.0, 0.0, 0.0]
OBSTACLE_PATIENCE = 3.0      

# --- CONFIGURATION RADIO ---
MIN_SIGNAL_THRESHOLD = 30.0  # Seuil critique du signal en % (src_link_quality_percent)
RADIO_CHECK_INTERVAL = 10    

# Variables globales mises a jour par le thread MQTT
LATEST_RADIO_SIGNAL = 100.0  
LATEST_RADIO_DISTANCE = 0.0

# --- FONCTION DE RECEPTION MQTT (CALLBACK) ---
def radio_callback(ev):
    global LATEST_RADIO_SIGNAL, LATEST_RADIO_DISTANCE
    try:
        # ev contient "payload" qui est une liste (repeated en Protobuf)
        for p in ev.payload:
            # On recupere le pourcentage de qualite et la distance
            if hasattr(p, 'src_link_quality_percent'):
                LATEST_RADIO_SIGNAL = float(p.src_link_quality_percent)
            if hasattr(p, 'range'):
                LATEST_RADIO_DISTANCE = float(p.range)
                
            # Note: S'il y a plusieurs radios (plusieurs payload), 
            # ca gardera la valeur de la derniere liste.
    except Exception as e:
        # On ignore silencieusement les erreurs de parsing pour ne pas polluer le terminal
        pass

def start_radio_monitor():
    """ Lance le client MQTT dans un thread pour ne pas bloquer ROS 2 """
    if not RADIO_MODULES_LOADED:
        print(">>> Exploration SANS surveillance radio (modules introuvables).")
        return
    try:
        a = tw.trellisAPI()
        # On ecoute le meme topic que testtw.py
        a.addMqttProtobuff("twt/tsm/v1/sa/node/links/+", trellis_pb2.Links, radio_callback)
        
        t = threading.Thread(target=a.run, daemon=True)
        t.start()
        print(">>> Moniteur Radio Trellis/MQTT demarre en arriere-plan.")
    except Exception as e:
        print(f">>> Erreur au lancement du thread radio: {e}")

def get_radio_signal_quality():
    global LATEST_RADIO_SIGNAL
    return LATEST_RADIO_SIGNAL

def get_radio_distance():
    global LATEST_RADIO_DISTANCE
    return LATEST_RADIO_DISTANCE

# --- NODE CARTO / ROS 2 ---
class MapProcessor(Node):
    def __init__(self):
        super().__init__('map_processor_node')
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_profile)
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)

        self.map_data = None
        self.map_info = None
        self.latest_costmap = None

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def costmap_callback(self, msg):
        self.latest_costmap = msg

def get_frontiers(map_node):
    if map_node.map_data is None: return []

    grid = map_node.map_data
    res = map_node.map_info.resolution
    ox = map_node.map_info.origin.position.x
    oy = map_node.map_info.origin.position.y

    unknown = (grid == -1) | (grid == 255)
    free = (grid >= 0) & (grid < 50)

    if np.sum(free) == 0: return []

    is_frontier = np.zeros_like(free, dtype=bool)
    is_frontier |= free & np.roll(unknown, 1, axis=0)
    is_frontier |= free & np.roll(unknown, -1, axis=0)
    is_frontier |= free & np.roll(unknown, 1, axis=1)
    is_frontier |= free & np.roll(unknown, -1, axis=1)

    y_idxs, x_idxs = np.where(is_frontier)
    
    frontiers = []
    for y, x in zip(y_idxs, x_idxs):
        frontiers.append([x * res + ox, y * res + oy])
    return frontiers

def is_target_safe(map_node, tx, ty, blacklist=[]):
    if map_node.map_data is None: return False
    
    for bad_x, bad_y in blacklist:
        dist = np.hypot(tx - bad_x, ty - bad_y)
        if dist < BLACKLIST_RADIUS:
            return False 

    res = map_node.map_info.resolution
    ox = map_node.map_info.origin.position.x
    oy = map_node.map_info.origin.position.y
    
    px = int((tx - ox) / res)
    py = int((ty - oy) / res)
    radius_px = int(SAFETY_DISTANCE / res)
    
    h, w = map_node.map_data.shape
    x_min = max(0, px - radius_px)
    x_max = min(w, px + radius_px)
    y_min = max(0, py - radius_px)
    y_max = min(h, py + radius_px)
    
    sub_grid = map_node.map_data[y_min:y_max, x_min:x_max]
    
    if np.any(sub_grid > 65):
        return False
        
    return True

def is_goal_blocked_by_costmap(map_node, tx, ty):
    if map_node.latest_costmap is None: return False
    
    cm = map_node.latest_costmap
    res = cm.info.resolution
    ox = cm.info.origin.position.x
    oy = cm.info.origin.position.y
    w = cm.info.width
    h = cm.info.height
    
    gx = int((tx - ox) / res)
    gy = int((ty - oy) / res)
    
    if gx < 0 or gx >= w or gy < 0 or gy >= h:
        return False
        
    index = gy * w + gx
    cost = cm.data[index]
    
    if cost > 50:
        return True 
        
    return False

def cluster_frontiers(map_node, points, blacklist):
    if not points: return []
    clusters = []
    points_copy = points.copy()
    
    while points_copy:
        ref = points_copy.pop(0)
        cluster = [ref]
        to_remove = []
        for p in points_copy:
            if np.hypot(p[0]-ref[0], p[1]-ref[1]) < 0.5:
                cluster.append(p)
                to_remove.append(p)
        for p in to_remove: points_copy.remove(p)
        
        if len(cluster) >= MIN_FRONTIER_SIZE:
            center = np.mean(np.array(cluster), axis=0)
            if is_target_safe(map_node, center[0], center[1], blacklist):
                clusters.append(center)
                
    return clusters

def main():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    save_script = os.path.join(current_dir, "save_my_map.py")
    launch_cmd = ["ros2", "launch", "my_robot_cartographer", "discovery.launch.py"]

    print(f"--- EXPLORATION V15 (AVEC SECURITE SIGNAL RADIO) ---")
    
    # LANCEMENT DU THREAD RADIO AVANT DE DEMARRER LE ROBOT
    start_radio_monitor()
    
    process = subprocess.Popen(launch_cmd, start_new_session=True)
    rclpy.init()
    
    failed_goals = [] 
    
    try:
        nav = BasicNavigator()
        map_node = MapProcessor()

        print(f">>> Demarrage ({INITIAL_WAIT}s)...")
        time.sleep(INITIAL_WAIT)

        print(">>> Connexion Nav2...")
        nav.nav_to_pose_client.wait_for_server()
        print("   -> Connecte.")
        
        print(">>> Attente Carte...")
        while map_node.map_data is None:
            rclpy.spin_once(map_node, timeout_sec=1.0)
        print("   -> Carte OK.")
        time.sleep(2.0) 

        print("\n>>> DEBUT EXPLORATION INTELLIGENTE")
        exploration_active = True
        retry = 0
        has_moved = False

        while exploration_active:
            rclpy.spin_once(map_node, timeout_sec=0.1)
            
            raw = get_frontiers(map_node)
            targets = cluster_frontiers(map_node, raw, failed_goals)
            
            # Affichage optionnel de la qualite radio actuelle au moment du scan
            sig = get_radio_signal_quality()
            dist = get_radio_distance()
            print(f"   [Scan] Cibles: {len(targets)} | Signal: {sig}% | Dist: {dist}m")

            if not targets:
                retry += 1
                if retry <= MAX_RETRIES:
                    print(f"   [INFO] Pas de cible accessible. Attente... ({retry}/{MAX_RETRIES})")
                    time.sleep(RETRY_DELAY)
                    continue
                else:
                    print(">>> FIN : Plus aucune zone accessible a explorer.")
                    break
            
            retry = 0
            target = targets[0]
            print(f"-> Go: x={target[0]:.2f}, y={target[1]:.2f}")
            
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = nav.get_clock().now().to_msg()
            goal.pose.position.x = target[0]
            goal.pose.position.y = target[1]
            goal.pose.orientation.w = 1.0
            
            nav.goToPose(goal)
            has_moved = True
            
            i = 0
            aborted_by_obstacle = False
            aborted_by_radio = False
            blocked_start_time = None 

            # BOUCLE DE NAVIGATION SURVEILLEE
            while not nav.isTaskComplete():
                i += 1
                rclpy.spin_once(map_node, timeout_sec=0.1)
                
                # --- VERIFICATION DU SIGNAL RADIO ---
                if i % RADIO_CHECK_INTERVAL == 0:
                    current_signal = get_radio_signal_quality()
                    current_dist = get_radio_distance()
                    
                    if current_signal < MIN_SIGNAL_THRESHOLD:
                        print(f"   [ALERTE RADIO] Signal faible ({current_signal}% < {MIN_SIGNAL_THRESHOLD}%) a {current_dist}m ! Stop immediat.")
                        nav.cancelTask()
                        aborted_by_radio = True
                        break

                # --- VERIFICATION OBSTACLES ---
                is_blocked = is_goal_blocked_by_costmap(map_node, target[0], target[1])
                
                if is_blocked:
                    if blocked_start_time is None:
                        blocked_start_time = time.time()
                    else:
                        elapsed = time.time() - blocked_start_time
                        if elapsed > OBSTACLE_PATIENCE:
                            print(f"   [ALERTE] Obstacle CONFIRME (> {OBSTACLE_PATIENCE}s) ! Abandon.")
                            nav.cancelTask()
                            aborted_by_obstacle = True
                            break 
                else:
                    if blocked_start_time is not None:
                        blocked_start_time = None

                if i % 20 == 0:
                    fb = nav.getFeedback()
                    if fb: print(f"      Reste: {fb.distance_remaining:.2f}m")

            # GESTION DU RESULTAT
            if aborted_by_radio:
                print(f"   >>> Ajout a la BLACKLIST (Hors de portee Radio) : x={target[0]:.2f}, y={target[1]:.2f}")
                failed_goals.append(target)
                nav.clearAllCostmaps()
                
                # Le robot recule de 0.3 m/s pendant 1 seconde pour retrouver du signal
                try:
                    from geometry_msgs.msg import Twist
                    pub = nav.nav_to_pose_client._node.create_publisher(Twist, 'cmd_vel', 10)
                    cmd = Twist()
                    cmd.linear.x = -0.3
                    pub.publish(cmd)
                    time.sleep(1.0)
                    cmd.linear.x = 0.0
                    pub.publish(cmd)
                except:
                    pass
                
                continue 

            if aborted_by_obstacle:
                print(f"   >>> Ajout a la BLACKLIST (Obstacle Confirme) : x={target[0]:.2f}, y={target[1]:.2f}")
                failed_goals.append(target)
                nav.clearAllCostmaps()
                time.sleep(1.0)
                continue 

            result = nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print("   [SUCCES] Arrive. Scan...")
                time.sleep(1.0)
            else:
                print("   [ECHEC] Cible inaccessible ou dangereuse !")
                print(f"   >>> Ajout a la BLACKLIST : x={target[0]:.2f}, y={target[1]:.2f}")
                failed_goals.append(target)
                nav.clearAllCostmaps()
                time.sleep(1.5)

        if has_moved:
            print("\n>>> Retour base...")
            home = PoseStamped()
            home.header.frame_id = 'map'
            home.header.stamp = nav.get_clock().now().to_msg()
            home.pose.position.x = HOME_POSE[0]
            home.pose.position.y = HOME_POSE[1]
            home.pose.orientation.w = 1.0
            nav.goToPose(home)
            while not nav.isTaskComplete(): pass
            print("Arrive.")

    except KeyboardInterrupt:
        print("\nSTOP")
        nav.cancelTask()

    finally:
        print("\n--- EXTINCTION ---")
        if os.path.exists(save_script):
            try: subprocess.run(["python3", save_script], check=False)
            except: pass
        try: rclpy.shutdown()
        except: pass
        try: os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except: pass

if __name__ == '__main__':
    main()
