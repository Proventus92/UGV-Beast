#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory

class CartographerBridge(Node):
    def __init__(self):
        super().__init__('carto_pose_bridge')
        
        # Parametres
        self.declare_parameter('configuration_directory', '/home/jetson/ros2_ws/src/my_robot_cartographer/config')
        self.declare_parameter('configuration_basename', 'localization.lua')
        
        # ID de la trajectoire active (On commence a 1 car 0 est la carte chargee)
        self.current_traj_id = 1 
        
        # 1. Subscriber
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            self.callback, 
            10
        )
        
        # 2. Clients Services
        self.finish_client = self.create_client(FinishTrajectory, 'finish_trajectory')
        self.start_client = self.create_client(StartTrajectory, 'start_trajectory')
        
        self.get_logger().info("--- PONT CARTO INITIALISE ---")
        self.get_logger().info("En attente d'un '2D Pose Estimate' dans RViz...")
        
        # Verification des services au demarrage
        if not self.finish_client.wait_for_service(timeout_sec=1.0):
             self.get_logger().warn("ATTENTION: Service 'finish_trajectory' introuvable !")
        if not self.start_client.wait_for_service(timeout_sec=1.0):
             self.get_logger().warn("ATTENTION: Service 'start_trajectory' introuvable !")

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f"[RECU] Clic RViz detecte : x={x:.2f}, y={y:.2f}")
        self.restart_trajectory(msg.pose.pose)

    def restart_trajectory(self, pose):
        # Etape A : Finir la trajectoire actuelle
        if self.current_traj_id is not None:
            self.get_logger().info(f"[ETAPE A] Demande d'arret de la trajectoire ID {self.current_traj_id}...")
            req_fin = FinishTrajectory.Request()
            req_fin.trajectory_id = self.current_traj_id
            
            # Appel asynchrone pour ne pas bloquer
            future_fin = self.finish_client.call_async(req_fin)
            future_fin.add_done_callback(self.on_finish_complete)
        else:
            self.get_logger().warn("Pas de trajectoire active connue a arreter.")

        # Etape B : Demarrer la nouvelle (On le fait tout de suite)
        self.get_logger().info("[ETAPE B] Preparation de la nouvelle trajectoire...")
        req_start = StartTrajectory.Request()
        req_start.configuration_directory = self.get_parameter('configuration_directory').value
        req_start.configuration_basename = self.get_parameter('configuration_basename').value
        req_start.use_initial_pose = True
        req_start.initial_pose = pose
        req_start.relative_to_trajectory_id = 0 # On se recale par rapport a la carte (ID 0)
        
        future_start = self.start_client.call_async(req_start)
        future_start.add_done_callback(self.on_start_complete)

    def on_finish_complete(self, future):
        try:
            # On verifie juste que l'appel s'est bien passe
            # (Cartographer renvoie peu d'infos sur le finish)
            self.get_logger().info("[ETAPE A] Trajectoire precedente terminee.")
        except Exception as e:
            self.get_logger().error(f"[ERREUR A] Echec arret trajectoire : {e}")

    def on_start_complete(self, future):
        try:
            response = future.result()
            # On met a jour l'ID pour la prochaine fois
            self.current_traj_id = response.trajectory_id
            self.get_logger().info(f"[SUCCES] Nouvelle trajectoire demarree ! ID = {self.current_traj_id}")
            self.get_logger().info(f"Statut Cartographer : {response.status.message}")
        except Exception as e:
            self.get_logger().error(f"[ERREUR B] Echec demarrage trajectoire : {e}")

def main():
    rclpy.init()
    node = CartographerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
