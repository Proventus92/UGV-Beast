#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import subprocess
import glob

def get_next_map_name(base_name="ma_carte"):
    # On liste TOUS les fichiers qui commencent par le nom de base
    files = glob.glob(f"{base_name}_*")
    max_num = 0
    for f in files:
        try:
            # On isole le nom de fichier sans le dossier
            fname = os.path.basename(f)
            # On enleve l'extension
            fname_no_ext = os.path.splitext(fname)[0]
            # On recupere ce qui est apres le nom de base
            num_part = fname_no_ext.replace(base_name + "_", "")
            
            # Si c'est un chiffre, on le prend en compte
            if num_part.isdigit():
                num = int(num_part)
                if num > max_num:
                    max_num = num
        except:
            pass
            
    return f"{base_name}_{max_num + 1}"

def clean_existing_files(map_name):
    # Supprime les fichiers potentiels qui pourraient bloquer l'ecriture
    extensions = [".yaml", ".png", ".pgm", ".pbstream"]
    for ext in extensions:
        filename = f"{map_name}{ext}"
        if os.path.exists(filename):
            try:
                os.remove(filename)
                print(f"   [Nettoyage] Fichier existant supprime : {filename}")
            except Exception as e:
                print(f"   [Attention] Impossible de supprimer {filename} : {e}")

def main():
    # Se place dans le dossier du script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    map_name = get_next_map_name()
    
    # Securite : on s'assure que la place est nette
    clean_existing_files(map_name)
    
    print(f"--- SAUVEGARDE : {map_name} ---")
    print(f"Dossier : {script_dir}")
    
    # 1. Sauvegarde Carte (PNG + YAML)
    print("1. Generation de la carte (PNG)...")
    try:
        # CORRECTION : Utilisation des parametres ROS standards pour le timeout (20s)
        # Suppression des arguments non reconnus
        cmd_nav2 = f"ros2 run nav2_map_server map_saver_cli -f {map_name} --fmt png --ros-args -p save_map_timeout:=20.0"
        subprocess.run(cmd_nav2, shell=True, check=True)
        print("   -> OK (Image PNG generee)")
    except Exception as e:
        print(f"   -> ERREUR Nav2 : {e}")

    # 2. Sauvegarde Etat (PBSTREAM) - Vital pour le robot
    print("2. Generation de la memoire robot (.pbstream)...")
    try:
        abs_path = os.path.join(script_dir, f"{map_name}.pbstream")
        cmd_carto = f"ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \"{{filename: '{abs_path}'}}\""
        subprocess.run(cmd_carto, shell=True, check=True)
        print("   -> OK")
    except Exception as e:
        print(f"   -> ERREUR Carto : {e}")

    print(f"--- TERMINE ! ---")

if __name__ == "__main__":
    main()