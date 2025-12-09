#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
from cartographer_ros_msgs.msg import StatusResponse
import time

class CartographerBridge(Node):
    def __init__(self):
        super().__init__('carto_pose_bridge')
        
        # --- CONFIGURATION ---
        # Adapter le chemin si nécessaire
        self.declare_parameter('configuration_directory', '/home/jetson/ros2_ws/install/my_robot_cartographer/share/my_robot_cartographer/config')
        self.declare_parameter('configuration_basename', 'localization.lua')
        
        # --- ETAT ---
        self.current_traj_id = 1  # On assume que la 1 est lancée au démarrage
        self.is_working = False   # VERROU DE SECURITE
        
        # --- COMMUNICATIONS ---
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.callback, 10)
        self.finish_client = self.create_client(FinishTrajectory, 'finish_trajectory')
        self.start_client = self.create_client(StartTrajectory, 'start_trajectory')
        
        self.get_logger().info("--- PONT CARTO PRET (Mode Blinde) ---")

    def callback(self, msg):
        # 1. VERROUILLAGE : On ignore si on est déjà en train de reset
        if self.is_working:
            self.get_logger().warn("IGNORED: Reset deja en cours, doucement sur le clic !")
            return
        
        self.is_working = True
        self.get_logger().info(f"[RECU] Clic detecte. On redemarre tout...")
        self.pending_pose = msg.pose.pose
        
        # 2. SEQUENCE DE LANCEMENT
        self.stop_current_trajectory()

    def stop_current_trajectory(self):
        # On tente d'arrêter la trajectoire courante
        if self.current_traj_id is not None:
            self.get_logger().info(f"[ETAPE 1] Tentative arret trajectoire ID {self.current_traj_id}...")
            req = FinishTrajectory.Request()
            req.trajectory_id = self.current_traj_id
            future = self.finish_client.call_async(req)
            future.add_done_callback(self.on_stop_complete)
        else:
            # Si on ne connait pas l'ID, on passe direct à la suite
            self.get_logger().warn("Pas d'ID connu, on tente le demarrage direct...")
            self.start_new_trajectory()

    def on_stop_complete(self, future):
        try:
            # On se fiche un peu du resultat (même si ça fail car "doesn't exist", c'est bon signe, elle est arretee)
            future.result() 
            self.get_logger().info("[ETAPE 1] Ordre d'arret envoye.")
        except Exception as e:
            self.get_logger().warn(f"Erreur arret (c est peut-etre normal) : {e}")
        
        # 3. PAUSE DE SECURITE (Crucial pour Jetson)
        # On laisse 2 secondes à Cartographer pour liberer le scan
        self.get_logger().info("Pause de nettoyage (2s)...")
        time.sleep(2.0) 
        
        self.start_new_trajectory()

    def start_new_trajectory(self):
        self.get_logger().info("[ETAPE 2] Demarrage nouvelle trajectoire...")
        
        req = StartTrajectory.Request()
        req.configuration_directory = self.get_parameter('configuration_directory').value
        req.configuration_basename = self.get_parameter('configuration_basename').value
        req.use_initial_pose = True
        req.initial_pose = self.pending_pose
        req.relative_to_trajectory_id = 0 # Par rapport a la map (ID 0)
        
        future = self.start_client.call_async(req)
        future.add_done_callback(self.on_start_complete)

    def on_start_complete(self, future):
        try:
            res = future.result()
            # VERIFICATION DU SUCCES
            if res.status.code == 0: # Code 0 = OK
                self.current_traj_id = res.trajectory_id
                self.get_logger().info(f"[SUCCES] ROBOT RECALe ! Nouvelle ID = {self.current_traj_id}")
            else:
                self.get_logger().error(f"[ECHEC] Cartographer a refuse : {res.status.message}")
                # Si ça echoue, on perd la trace de l'ID, on devra peut-etre tuer le noeud
                
        except Exception as e:
            self.get_logger().error(f"[CRASH] Erreur appel service : {e}")
        
        # 4. DEVERROUILLAGE
        self.is_working = False
        self.get_logger().info("--- Pret pour le prochain clic ---")

def main():
    rclpy.init()
    node = CartographerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()