# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import time
import os
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

MIN_FRONTIER_SIZE = 4        
MAX_RETRIES = 5              
RETRY_DELAY = 1.0            
SAFETY_DISTANCE = 0.45       
BLACKLIST_RADIUS = 0.80      
INITIAL_WAIT = 5.0           
HOME_POSE = [0.0, 0.0, 0.0] # <-- Le point d'origine
OBSTACLE_PATIENCE = 3.0      

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

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)

        self.map_data = None
        self.map_info = None
        self.latest_costmap = None
        self.is_armed = False 
        self.flight_mode = "" # <-- Mémorise le mode de vol (Mission, Return, Manual...)

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def costmap_callback(self, msg):
        self.latest_costmap = msg

    def state_callback(self, msg):
        self.is_armed = msg.armed
        self.flight_mode = msg.mode

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

    print(f"--- EXPLORATION MISSION QGC (RTL SECURISE) ---")
    
    rclpy.init()
    failed_goals = [] 
    
    try:
        nav = BasicNavigator()
        map_node = MapProcessor()

        print(f">>> Demarrage de l'IA ({INITIAL_WAIT}s)...")
        time.sleep(INITIAL_WAIT)

        print(">>> Connexion a Nav2...")
        nav.nav_to_pose_client.wait_for_server()
        print("   -> Connecte.")
        
        print(">>> Attente de la carte de Cartographer...")
        while map_node.map_data is None:
            rclpy.spin_once(map_node, timeout_sec=1.0)
        print("   -> Carte OK.")
        time.sleep(2.0) 

        if not map_node.is_armed:
            print("\n>>> [ATTENTE] Le robot est DESARME. Armez-le depuis QGC pour lancer l'exploration...")
            while not map_node.is_armed:
                rclpy.spin_once(map_node, timeout_sec=0.5)
            print("   -> Robot ARME ! Lancement imminent.")
            time.sleep(1.0)

        print("\n>>> DEBUT EXPLORATION INTELLIGENTE")
        exploration_active = True
        retry = 0
        has_moved = False

        while exploration_active:
            rclpy.spin_once(map_node, timeout_sec=0.1)

            # --- INTERCEPTION DU BOUTON "RETURN" DE QGC ---
            if "RTL" in map_node.flight_mode:
                print("\n   [RTL] Ordre de retour a la base (RTL) recu depuis QGC !")
                exploration_active = False
                break
            
            raw = get_frontiers(map_node)
            targets = cluster_frontiers(map_node, raw, failed_goals)
            
            print(f"   [Scan] Frontieres: {len(raw)} | Cibles Sures: {len(targets)} | Blacklistes: {len(failed_goals)}")

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
            aborted_by_disarm = False 
            blocked_start_time = None 

            while not nav.isTaskComplete():
                i += 1
                rclpy.spin_once(map_node, timeout_sec=0.1)

                # --- INTERCEPTION DU BOUTON "RETURN" PENDANT LE TRAJET ---
                if "RTL" in map_node.flight_mode:
                    print("\n   [RTL]  Ordre de retour a la base (RTL) recu en cours de route !")
                    nav.cancelTask()
                    exploration_active = False
                    break
                
                # PAUSE SI DESARMEMENT
                if not map_node.is_armed:
                    print("\n   [PAUSE]  Robot DESARME ! Annulation du trajet et mise en pause...")
                    nav.cancelTask()
                    
                    while not map_node.is_armed:
                        rclpy.spin_once(map_node, timeout_sec=0.5)
                        
                    print("   [REPRISE]  Robot RE-ARME ! Calcul d'une nouvelle trajectoire...")
                    aborted_by_disarm = True
                    break

                # GESTION DES OBSTACLES IMPREVUS
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

            if not exploration_active:
                break # On sort immediatement pour declencher le RTL en bas

            if aborted_by_disarm:
                time.sleep(1.0)
                continue 

            if aborted_by_obstacle:
                print(f"   >>> Ajout a la BLACKLIST (Obstacle) : x={target[0]:.2f}, y={target[1]:.2f}")
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

        # ========================================================
        # SEQUENCE DE RETOUR (FIN DE MISSION OU BOUTON RTL)
        # ========================================================
        if has_moved or "RTL" in map_node.flight_mode:
            print("\n==================================================")
            print(">>> LANCEMENT DE LA SEQUENCE DE RETOUR (RTL) <<<")
            print("==================================================")
            
            home = PoseStamped()
            home.header.frame_id = 'map'
            home.header.stamp = nav.get_clock().now().to_msg()
            home.pose.position.x = HOME_POSE[0]
            home.pose.position.y = HOME_POSE[1]
            home.pose.orientation.w = 1.0
            
            nav.goToPose(home)
            
            while not nav.isTaskComplete():
                rclpy.spin_once(map_node, timeout_sec=0.1)
                
                # Le retour a la base conserve la securite d'arret d'urgence !
                if not map_node.is_armed:
                    print("\n   [PAUSE RTL] Robot DESARME pendant le retour ! Mise en pause...")
                    nav.cancelTask()
                    
                    while not map_node.is_armed:
                        rclpy.spin_once(map_node, timeout_sec=0.5)
                        
                    print("   [REPRISE RTL] Robot RE-ARME ! Reprise du retour a la base...")
                    home.header.stamp = nav.get_clock().now().to_msg()
                    nav.goToPose(home)

            print(">>> ARRIVE A DESTINATION ! Le robot est a la base.")

    except KeyboardInterrupt:
        print("\nARRET DEMANDE (Mode Manuel active depuis QGC)")
        try:
            nav.cancelTask()
        except:
            pass

    finally:
        print("\n--- EXTINCTION PROPRE DU SCRIPT ---")
        if os.path.exists(save_script):
            try: 
                print("   -> Sauvegarde de la carte en cours...")
                subprocess.run(["python3", save_script], check=False)
            except: pass
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()
