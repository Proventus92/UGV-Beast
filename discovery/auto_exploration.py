# -*- coding: Windows-1252 -*-

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
MIN_FRONTIER_SIZE = 4        # On ignore les trop petits trous (bruit)
MAX_RETRIES = 5              # Nombre de tentatives avant fin
RETRY_DELAY = 1.0            # Temps entre deux analyses
SAFETY_DISTANCE = 0.45       # 45cm du mur minimum pour valider une cible
BLACKLIST_RADIUS = 0.80      # Rayon d'exclusion autour d'un échec (80cm)
INITIAL_WAIT = 5.0           # Attente démarrage
HOME_POSE = [0.0, 0.0, 0.0]

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
        self.map_data = None
        self.map_info = None

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

def get_frontiers(map_node):
    """ Trouve les frontières (Zone Libre <-> Zone Inconnue) """
    if map_node.map_data is None: return []

    grid = map_node.map_data
    res = map_node.map_info.resolution
    ox = map_node.map_info.origin.position.x
    oy = map_node.map_info.origin.position.y

    # --- LOGIQUE ROBUSTE ---
    # Inconnu = -1 ou 255 (selon cartographer)
    # Libre = 0 à 50 (tolérance pour le gris clair)
    unknown = (grid == -1) | (grid == 255)
    free = (grid >= 0) & (grid < 50)

    # Si pas de zone libre, pas de frontières
    if np.sum(free) == 0: return []

    # Pixel libre voisin d'un inconnu
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
    """ 
    Vérifie 2 choses :
    1. Pas trop proche d'un mur (SAFETY_DISTANCE)
    2. Pas dans une zone blacklistée (échec précédent)
    """
    if map_node.map_data is None: return False
    
    # 1. Vérification Blacklist
    for bad_x, bad_y in blacklist:
        dist = np.hypot(tx - bad_x, ty - bad_y)
        if dist < BLACKLIST_RADIUS:
            return False # Trop près d'un endroit maudit

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
    
    # Si on trouve un obstacle (>65) dans le rayon de sécurité, on rejette
    if np.any(sub_grid > 65):
        return False
        
    return True

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
            # On ne garde que si c'est SÛR et PAS dans la Blacklist
            if is_target_safe(map_node, center[0], center[1], blacklist):
                clusters.append(center)
                
    return clusters

def main():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    save_script = os.path.join(current_dir, "save_my_map.py")
    launch_cmd = ["ros2", "launch", "my_robot_cartographer", "discovery.launch.py"]

    print(f"--- EXPLORATION V12 (BLACKLIST + SECURITE) ---")
    process = subprocess.Popen(launch_cmd, start_new_session=True)
    rclpy.init()
    
    # Liste des endroits où le robot a échoué (x, y)
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
        time.sleep(2.0) # Petite stabilisation

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
            # On prend la première cible valide
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
            while not nav.isTaskComplete():
                i += 1
                rclpy.spin_once(map_node, timeout_sec=0.1)
                if i % 20 == 0:
                    fb = nav.getFeedback()
                    if fb: print(f"      Reste: {fb.distance_remaining:.2f}m")

            # 3. GESTION DU RÉSULTAT
            result = nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print("   [SUCCÈS] Arrivé. Scan...")
                time.sleep(1.0)
            else:
                print("   [ÉCHEC] Cible inaccessible ou dangereuse !")
                print(f"   >>> Ajout à la BLACKLIST : x={target[0]:.2f}, y={target[1]:.2f}")
                # On ajoute ce point à la liste noire pour ne plus y retourner
                failed_goals.append(target)
                
                # On nettoie les cartes de coûts pour repartir propre
                nav.clearAllCostmaps()
                time.sleep(1.5)
                # La boucle reprendra, mais `cluster_frontiers` ignorera désormais cette zone

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