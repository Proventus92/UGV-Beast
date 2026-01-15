# -*- coding: Windows-1252 -*-
#!/usr/bin/env python3

import time
import os
import subprocess
import signal
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# --- CONFIGURATION UTILISATEUR ---
MIN_FRONTIER_SIZE = 4        # On ignore les trop petits trous
MAX_RETRIES = 5              # Nombre de tentatives avant fin
RETRY_DELAY = 1.0            # Temps entre deux analyses
SAFETY_DISTANCE = 0.45       # 45cm du mur minimum
BLACKLIST_RADIUS = 0.80      # Rayon d'exclusion autour d'un échec
INITIAL_WAIT = 5.0           # Attente démarrage
HOME_POSE = [0.0, 0.0, 0.0]

# NOUVEAU : Temps d'attente pour confirmer un obstacle
OBSTACLE_PATIENCE = 3.0      

class MapProcessor(Node):
    def __init__(self):
        super().__init__('map_processor_node')
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        # Abonnement Carte Statique (SLAM)
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_profile)
        
        # AJOUT : Abonnement Carte Dynamique (Obstacles temps réel)
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
    """ Trouve les frontières (Zone Libre <-> Zone Inconnue) """
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
    """ Vérifie si la cible est sûre sur la carte STATIQUE """
    if map_node.map_data is None: return False
    
    # 1. Vérification Blacklist
    for bad_x, bad_y in blacklist:
        dist = np.hypot(tx - bad_x, ty - bad_y)
        if dist < BLACKLIST_RADIUS:
            return False 

    # 2. Vérification Murs
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
    """ 
    Vérifie si la cible est bloquée par un obstacle DYNAMIQUE (Costmap).
    Retourne True si bloqué.
    """
    if map_node.latest_costmap is None: return False
    
    cm = map_node.latest_costmap
    res = cm.info.resolution
    ox = cm.info.origin.position.x
    oy = cm.info.origin.position.y
    w = cm.info.width
    h = cm.info.height
    
    # Conversion coords monde -> grille
    gx = int((tx - ox) / res)
    gy = int((ty - oy) / res)
    
    # Hors limite ?
    if gx < 0 or gx >= w or gy < 0 or gy >= h:
        return False
        
    # Vérification de la valeur dans la costmap (0-100)
    index = gy * w + gx
    cost = cm.data[index]
    
    # > 50 signifie généralement qu'on est très proche d'un obstacle ou dedans
    if cost > 50:
        return True 
        
    return False

def cluster_frontiers(map_node, points, blacklist):
    """ Regroupe les points et filtre les zones dangereuses """
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

    print(f"--- EXPLORATION V13 (VERIFICATION PATIENTE {OBSTACLE_PATIENCE}s) ---")
    process = subprocess.Popen(launch_cmd, start_new_session=True)
    rclpy.init()
    
    failed_goals = [] 
    
    try:
        nav = BasicNavigator()
        map_node = MapProcessor()

        print(f">>> Démarrage ({INITIAL_WAIT}s)...")
        time.sleep(INITIAL_WAIT)

        print(">>> Connexion Nav2...")
        nav.nav_to_pose_client.wait_for_server()
        print("   -> Connecté.")
        
        print(">>> Attente Carte...")
        while map_node.map_data is None:
            rclpy.spin_once(map_node, timeout_sec=1.0)
        print("   -> Carte OK.")
        time.sleep(2.0) 

        print("\n>>> DÉBUT EXPLORATION INTELLIGENTE")
        exploration_active = True
        retry = 0
        has_moved = False

        while exploration_active:
            rclpy.spin_once(map_node, timeout_sec=0.1)
            
            # 1. Analyse Carte
            raw = get_frontiers(map_node)
            # 2. Filtrage avec BLACKLIST
            targets = cluster_frontiers(map_node, raw, failed_goals)
            
            print(f"   [Scan] Frontières: {len(raw)} | Cibles Sûres: {len(targets)} | Blacklistés: {len(failed_goals)}")

            if not targets:
                retry += 1
                if retry <= MAX_RETRIES:
                    print(f"   [INFO] Pas de cible accessible. Attente... ({retry}/{MAX_RETRIES})")
                    time.sleep(RETRY_DELAY)
                    continue
                else:
                    print(">>> FIN : Plus aucune zone accessible à explorer.")
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
            blocked_start_time = None # Chronomètre pour la patience

            # BOUCLE DE NAVIGATION SURVEILLÉE
            while not nav.isTaskComplete():
                i += 1
                rclpy.spin_once(map_node, timeout_sec=0.1)
                
                # --- LOGIQUE DE CONFIRMATION D'OBSTACLE ---
                # On vérifie si le point cible est recouvert par un obstacle (costmap)
                is_blocked = is_goal_blocked_by_costmap(map_node, target[0], target[1])
                
                if is_blocked:
                    if blocked_start_time is None:
                        # Début du problème : on lance le chrono
                        blocked_start_time = time.time()
                        # print("   [INFO] Obstacle détecté... Analyse en cours...")
                    else:
                        # Si cela fait plus de 3 secondes qu'on est bloqué
                        elapsed = time.time() - blocked_start_time
                        if elapsed > OBSTACLE_PATIENCE:
                            print(f"   [ALERTE] 🛑 Obstacle CONFIRMÉ (> {OBSTACLE_PATIENCE}s) ! Abandon.")
                            nav.cancelTask()
                            aborted_by_obstacle = True
                            break 
                else:
                    # Si l'obstacle disparaît (faux positif), on reset le chrono
                    if blocked_start_time is not None:
                        blocked_start_time = None
                        # print("   [INFO] Le chemin s'est libéré. On continue.")

                if i % 20 == 0:
                    fb = nav.getFeedback()
                    if fb: print(f"      Reste: {fb.distance_remaining:.2f}m")

            # 3. GESTION DU RÉSULTAT
            if aborted_by_obstacle:
                print(f"   >>> Ajout à la BLACKLIST (Obstacle Confirmé) : x={target[0]:.2f}, y={target[1]:.2f}")
                failed_goals.append(target)
                nav.clearAllCostmaps()
                time.sleep(1.0)
                continue # On passe direct au prochain tour de boucle

            result = nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print("   [SUCCÈS] Arrivé. Scan...")
                time.sleep(1.0)
            else:
                print("   [ÉCHEC] Cible inaccessible ou dangereuse !")
                print(f"   >>> Ajout à la BLACKLIST : x={target[0]:.2f}, y={target[1]:.2f}")
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
            print("Arrivé.")

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
