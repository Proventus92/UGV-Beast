#!/usr/bin/env python3
import subprocess
import signal
import sys
import time
import os

def main():
    # Configuration automatique des chemins
    current_dir = os.path.dirname(os.path.abspath(__file__))
    save_script = os.path.join(current_dir, "save_my_map.py")
    
    launch_cmd = ["ros2", "launch", "my_robot_cartographer", "discovery.launch.py"]

    print(f"--- DEMARRAGE DECOUVERTE AUTO ---")
    print(f"Dossier de stockage : {current_dir}")
    print(f"Lancement du robot...")
    
    # start_new_session=True est crucial
    process = subprocess.Popen(launch_cmd, start_new_session=True)

    try:
        # Attente infinie
        process.wait()
        
    except KeyboardInterrupt:
        print("\n\nSTOP DEMANDE (Ctrl+C) !")
        print("ATTENTION : Ne touchez plus a rien, sauvegarde en cours...")

        if os.path.exists(save_script):
            try:
                print("Lancement de la sauvegarde automatique...")
                subprocess.run(["python3", save_script], check=True)
                print("Sauvegarde terminee avec succes.")
            except Exception as e:
                print(f"Erreur pendant la sauvegarde : {e}")
        else:
            print(f"Erreur : Script introuvable -> {save_script}")

        print("Extinction du systeme ROS 2...")
        
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10)
        except:
            print("Forcage de l'arret...")
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except:
                pass
            
        print("Termine.")

if __name__ == "__main__":
    main()