# 🤖 UGV Beast — Guide de prise en main ROS2

> Robot terrestre autonome à chenilles — **PROCOMM-MMC / Service R&D**
>
> Plateforme : **NVIDIA Jetson Orin Nano** · **ROS2 Humble** · **Ubuntu 22.04**
>
> **IP du robot sur le réseau R&D : `192.168.50.107`**

---

## 📋 Table des matières

1. [Architecture du système](#1-architecture-du-système)
2. [Connexion SSH via MobaXterm](#2-connexion-ssh-via-mobaxterm)
3. [Installation complète](#3-installation-complète)
4. [Structure du workspace](#4-structure-du-workspace)
5. [Scripts Python — Discovery & Exploration](#5-scripts-python--discovery--exploration)
6. [Visualisation avec RViz2](#6-visualisation-avec-rviz2)
7. [QGroundControl + MAVROS](#7-qgroundcontrol--mavros)
8. [WinTAK — Situation Awareness](#8-wintak--situation-awareness)
9. [Utilisation avec les radios TW Spirit 860](#9-utilisation-avec-les-radios-tw-spirit-860)

---

## 1. Architecture du système

Le robot repose sur une architecture à deux niveaux.

**Niveau supérieur — Jetson Orin Nano** : exécute tous les algorithmes (SLAM, navigation autonome, traitement capteurs, communication réseau). Tourne sous Ubuntu 22.04 avec ROS2 Humble.

**Niveau inférieur — ESP32** : gère uniquement le bas-niveau (moteurs de traction, servomoteur de la tête, LEDs). La communication entre la Jetson et l'ESP32 se fait via **USB** (port `/dev/ttyUSB0`).

```
┌─────────────────────────────────────────────────┐
│              Jetson Orin Nano                    │
│              Ubuntu 22.04 / ROS2 Humble          │
│              IP WiFi R&D : 192.168.50.107         │
│                                                  │
│  ┌──────────────┐  ┌─────────────┐  ┌─────────┐  │
│  │ LiDAR LD19   │  │ OAK-D Lite  │  │  MAVROS │  │
│  │ /dev/ttyUSB* │  │ USB 3       │  │  bridge │  │
│  └──────────────┘  └─────────────┘  └─────────┘  │
└──────────────────────┬──────────────────────────┘
                       │ USB (/dev/ttyUSB0)
              ┌────────▼────────┐
              │     ESP32       │
              │  Moteurs/Servos │
              └─────────────────┘
        ▲                       ▲
        │  UDP réseau local      │  UDP réseau local
   QGroundControl            WinTAK
   (MAVLink :14550)       (CoT :8087)
```

**Capteurs embarqués :**
- **LiDAR LDLidar LD19** : balayage 360° horizontal, publie sur `/scan`
- **IMU** : accélération et rotation, publie sur `/imu/data`
- **Caméra OAK-D Lite** : flux RGB + nuage de points 3D, publie sur `/oak/points` et `/oak/scan`

> ⚠️ L'odométrie par roue n'est **pas utilisée** : les chenilles glissent et la mesure n'est pas fiable. L'odométrie est calculée à partir du LiDAR par `rf2o_laser_odometry`.

<!-- 📸 IMAGE SUGGÉRÉE : Photo du robot avec légendes des composants -->

---

## 2. Connexion SSH via MobaXterm

> Le robot doit être **allumé** et **connecté au WiFi R&D** avant toute connexion.

### Prérequis

- Être connecté au **WiFi du service R&D** (même réseau que le robot)
- Avoir **MobaXterm** installé → [télécharger ici](https://mobaxterm.mobatek.net/download.html)

> 💡 MobaXterm gère nativement le SSH et le **forwarding X11**, ce qui permet d'ouvrir des interfaces graphiques distantes comme RViz2 directement depuis Windows.

### Étapes

**1. Créer une nouvelle session SSH**

Dans MobaXterm : `Session` → `SSH`

<!-- 📸 IMAGE SUGGÉRÉE : Screenshot MobaXterm — fenêtre Session SSH -->

**2. Renseigner les paramètres de connexion**

| Champ | Valeur |
|-------|--------|
| Remote host | `192.168.50.107` |
| Username | `jetson` |
| Port | `22` |

Cocher ✅ **"Specify username"** → saisir `jetson`.

Dans l'onglet `Advanced` : vérifier que ✅ **"X11-Forwarding"** est activé (nécessaire pour RViz2).

<!-- 📸 IMAGE SUGGÉRÉE : Screenshot du formulaire SSH MobaXterm rempli -->

**3. Se connecter**

```
Mot de passe : jetson
```

Une fois connecté, le terminal affiche :

```
jetson@ubuntu:~$
```

**4. Ouvrir plusieurs terminaux (si besoin)**

Clic droit sur l'onglet de session → `Duplicate tab`

> ✅ Toutes les commandes qui suivent sont à exécuter dans ce terminal SSH.

<!-- 📸 IMAGE SUGGÉRÉE : Screenshot MobaXterm avec la session active -->

---

## 3. Installation complète

> ⚠️ Cette section est à suivre **une seule fois** lors d'une installation from scratch sur une Jetson vierge.
> Si le workspace est déjà compilé sur le robot, passer directement à la **section 4**.

### 3.1 Installer ROS2 Humble

```bash
# Configurer les sources
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Installer ROS2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep

# Initialiser rosdep
sudo rosdep init
rosdep update

# Sourcer ROS2 au démarrage
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.2 Installer les dépendances ROS2

```bash
# Nav2 — stack de navigation autonome
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# Google Cartographer — SLAM 2D
sudo apt install -y ros-humble-cartographer ros-humble-cartographer-ros

# MAVROS — bridge MAVLink ↔ ROS2
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras

# Installer les datasets GeographicLib (obligatoire pour MAVROS)
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# pointcloud_to_laserscan — conversion nuage de points 3D → scan 2D
sudo apt install -y ros-humble-pointcloud-to-laserscan

# teleop_twist_keyboard — contrôle clavier du robot (utile pour cartographier)
sudo apt install -y ros-humble-teleop-twist-keyboard

# Outils de diagnostic TF
sudo apt install -y ros-humble-tf2-tools
```

### 3.3 Installer le driver OAK-D Lite (DepthAI)

```bash
# Dépendances Python DepthAI
sudo apt install -y python3-pip
pip3 install depthai --break-system-packages

# Driver ROS2 pour OAK-D Lite
sudo apt install -y ros-humble-depthai-ros
```

### 3.4 Cloner le dépôt et compiler

```bash
# Créer le workspace s'il n'existe pas
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Cloner le dépôt
git clone https://github.com/Proventus92/UGV-Beast.git .

# Installer les dépendances ROS2 manquantes automatiquement
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Compiler
colcon build --symlink-install

# Sourcer le workspace au démarrage
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.5 Configurer les permissions des ports série

```bash
# Donner les droits d'accès aux ports série (LiDAR et ESP32 via USB)
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1

# Rendre permanent (ajouter l'utilisateur au groupe dialout)
sudo usermod -aG dialout jetson
```

> 💡 Après `usermod`, se déconnecter et reconnecter pour que le changement prenne effet. Les ports `/dev/ttyUSB0` et `/dev/ttyUSB1` peuvent s'inverser selon l'ordre de branchement USB. Utiliser `ls /dev/ttyUSB*` pour identifier lequel correspond au LiDAR et lequel à l'ESP32.

### 3.6 Vérifier l'installation

```bash
# Vérifier que ROS2 voit bien les packages compilés
ros2 pkg list | grep -E "ugv|ldlidar|cartographer|rf2o|oak"

# Vérifier les topics actifs (robot allumé et scripts lancés)
ros2 topic list
```

La liste de topics attendue quand tout tourne :

```
/cmd_vel          ← commandes moteurs
/imu              ← données IMU
/map              ← carte Cartographer
/oak/points       ← nuage de points OAK-D Lite
/oak/scan         ← scan virtuel caméra (obstacles bas)
/odom             ← odométrie rf2o
/scan             ← données LiDAR
/tf               ← transformées spatiales
```

<!-- 📸 IMAGE SUGGÉRÉE : Terminal avec ros2 topic list montrant tous les topics actifs -->

---

## 4. Structure du workspace

```
~/ros2_ws/
├── src/
│   ├── ugv_driver/              # Driver : comm Jetson ↔ ESP32 via USB
│   ├── ugv/                     # Package principal (MAVROS, modes, launch files)
│   ├── ldlidar_stl_ros2/        # Driver LiDAR LD19
│   ├── my_robot_cartographer/   # Configuration SLAM Cartographer (fichier .lua)
│   ├── oak_d_lite/              # Driver + config caméra OAK-D Lite
│   └── rf2o_laser_odometry/     # Odométrie calculée depuis le LiDAR
│
└── discovery/                   # Scripts Python autonomes
    ├── auto_discovery.py            # Exploration autonome (WiFi)
    ├── auto_discovery_radio.py      # Exploration autonome (radio TrellisWare)
    ├── auto_exploration.py          # Exploration + cartographie (WiFi)
    ├── auto_exploration_radio.py    # Exploration + cartographie (radio TrellisWare)
    ├── mavros_exploration.py        # Exploration + MAVLink (QGC + WinTAK)
    ├── save_my_map.py               # Sauvegarde manuelle de la carte
    └── ma_carte_*.{pbstream,png,yaml}  # Cartes sauvegardées (numérotation auto)
```

---

## 5. Scripts Python — Discovery & Exploration

Tous les scripts se trouvent dans `~/ros2_ws/discovery/`. Chaque script **lance automatiquement tous les nœuds ROS2 nécessaires** (LiDAR, odométrie, Cartographer, Nav2…). Il n'est pas nécessaire de lancer des composants séparément dans d'autres terminaux.

```bash
cd ~/ros2_ws/discovery
python3 <nom_du_script>.py
```

**Arrêter un script :**
```bash
Ctrl + C
```

---

### Vue d'ensemble des modes

| Script | Connexion | Cartographie | Exploration | MAVLink/CoT |
|--------|-----------|-------------|-------------|-------------|
| `auto_discovery.py` | WiFi | Oui | Oui | Non |
| `auto_discovery_radio.py` | Radio TW860 | Non | ✅ Oui | Non |
| `auto_exploration.py` | WiFi | ✅ Oui | ✅ Oui | Non |
| `auto_exploration_radio.py` | Radio TW860 | ✅ Oui | ✅ Oui | Non |
| `mavros_exploration.py` | WiFi ou Radio | ✅ Oui | ✅ Oui | ✅ Oui |

---

### `auto_discovery.py` — Exploration autonome (WiFi)

Le robot explore son environnement de façon totalement autonome en utilisant un algorithme de type **frontier-based** : il identifie les zones inconnues aux frontières de la carte et s'y dirige automatiquement, jusqu'à avoir couvert tout l'espace accessible.

```bash
cd ~/ros2_ws/discovery
python3 auto_discovery.py
```

Le robot navigue, évite les obstacles (LiDAR + caméra OAK-D Lite pour les obstacles bas comme les pieds de chaises) et choisit sa prochaine destination sans intervention humaine.

<!-- 📸 IMAGE SUGGÉRÉE : Robot en exploration autonome dans la salle R&D -->

---

### `auto_discovery_radio.py` — Exploration autonome (radio TrellisWare)

Même comportement que `auto_discovery.py`, mais configuré pour fonctionner via la **liaison radio TW Spirit 860** au lieu du WiFi R&D. À utiliser quand le robot est connecté via les radios TrellisWare (voir section 9).

```bash
cd ~/ros2_ws/discovery
python3 auto_discovery_radio.py
```

---

### `auto_exploration.py` — Exploration et cartographie (WiFi)

Le robot explore et **construit simultanément une carte** (SLAM avec Google Cartographer). Les cartes sont **sauvegardées automatiquement** dans le dossier `discovery/` à la fin de chaque session, sous la forme `ma_carte_N.png` / `ma_carte_N.yaml` / `ma_carte_N.pbstream` (numéro auto-incrémenté).

```bash
cd ~/ros2_ws/discovery
python3 auto_exploration.py
```

Pour consulter la dernière carte générée :
```bash
ls ~/ros2_ws/discovery/ma_carte_*.png | tail -1
eog ~/ros2_ws/discovery/ma_carte_277.png   # remplacer par le bon numéro
```

<!-- 📸 IMAGE SUGGÉRÉE : Exemple de carte générée (ma_carte_N.png) montrant la salle R&D -->

---

### `auto_exploration_radio.py` — Exploration et cartographie (radio TrellisWare)

Même comportement que `auto_exploration.py`, mais pour une utilisation via la **liaison radio TW Spirit 860**. À utiliser quand le robot opère hors du réseau WiFi R&D (voir section 9).

```bash
cd ~/ros2_ws/discovery
python3 auto_exploration_radio.py
```

---

### `mavros_exploration.py` — Exploration avec supervision QGC et WinTAK

Ce script combine l'exploration autonome avec le **bridge MAVROS** : le robot envoie en temps réel sa télémétrie (état, batterie, mode, position) vers **QGroundControl** via MAVLink et vers **WinTAK** via messages CoT. Il reçoit également les commandes envoyées depuis ces interfaces.

```bash
cd ~/ros2_ws/discovery
python3 mavros_exploration.py
```

**Modes disponibles depuis QGroundControl :**
- **Manual** : contrôle direct des déplacements
- **Navigation** : navigation autonome vers un point objectif
- **Exploration** : exploration frontier-based automatique
- **Return** : retour au point de départ (RTL)

**Commandes disponibles depuis WinTAK (chat GeoChat) :**
- `START` : lance l'exploration
- `STOP` : stoppe le robot
- `RETURN` : retour au point de départ
- `END` : arrêt complet du script

> 💡 Ce script est celui à utiliser lors des démonstrations ou des opérations terrain, car il permet une supervision complète depuis QGC et WinTAK simultanément.

---

### `save_my_map.py` — Sauvegarde manuelle de la carte

Permet de sauvegarder la carte en cours de construction à n'importe quel moment sans arrêter le script d'exploration.

```bash
cd ~/ros2_ws/discovery
python3 save_my_map.py
```

La carte est enregistrée dans `~/ros2_ws/discovery/` avec un numéro incrémenté automatiquement.

---

## 6. Visualisation avec RViz2

RViz2 permet de visualiser en temps réel les données capteurs, la carte en construction et la position du robot pendant qu'un script tourne.

> ⚠️ RViz2 nécessite un affichage graphique. Dans MobaXterm, le **X11-Forwarding** doit être activé dans les paramètres de la session (voir section 2).

Ouvrir un second onglet MobaXterm (clic droit → `Duplicate tab`) puis :

```bash
cd ~/ros2_ws && source install/setup.bash
rviz2
```

### Topics utiles à afficher

Dans RViz2, cliquer sur **"Add"** (bas gauche) :

| Type | Topic | Description |
|------|-------|-------------|
| Map | `/map` | Carte en cours de construction |
| LaserScan | `/scan` | Points LiDAR (obstacles à hauteur du lidar) |
| LaserScan | `/oak/scan` | Points caméra (obstacles bas, pieds de chaises) |
| Odometry | `/odom` | Position estimée du robot |
| PointCloud2 | `/oak/points` | Nuage de points 3D OAK-D Lite |
| TF | — | Repères spatiaux de tous les composants |

**Changer le repère de référence :** panneau gauche → `Global Options` → `Fixed Frame` → `map`

> 💡 La costmap de Nav2 peut aussi être affichée pour comprendre comment le robot perçoit les obstacles et planifie sa trajectoire : ajouter `Map` sur le topic `/local_costmap/costmap`. Les zones roses/rouges indiquent le danger, les zones bleues l'espace libre.

### Sauvegarder la configuration RViz2

```
Dans RViz2 : File → Save Config As
→ sauvegarder dans ~/ros2_ws/src/my_robot_cartographer/rviz/ugv_beast.rviz
```

Relancer avec cette configuration :
```bash
rviz2 -d ~/ros2_ws/src/my_robot_cartographer/rviz/ugv_beast.rviz
```

<!-- 📸 IMAGE SUGGÉRÉE : RViz2 avec carte, LaserScan et costmap visibles -->

---

## 7. QGroundControl + MAVROS

QGroundControl (QGC) est la station de contrôle au sol. Il se connecte au robot via **MAVLink sur UDP** et permet de superviser et de piloter le robot avec une interface professionnelle, sans ligne de commande.

### Prérequis

Lancer le script `mavros_exploration.py` sur le robot (voir section 5), qui démarre automatiquement le nœud MAVROS.

### Architecture

```
Robot (Jetson) — 192.168.50.107            PC Opérateur
┌────────────────────────────────┐         ┌──────────────────────────┐
│  mavros_exploration.py         │  UDP    │                          │
│  └─ MAVROS node                │────────►│   QGroundControl         │
│     (bridge ROS2 ↔ MAVLink)    │ :14550  │                          │
└────────────────────────────────┘         └──────────────────────────┘
```

### Sur le PC — Installer et configurer QGroundControl

**1. Installer QGroundControl**

→ [https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)

**2. Ajouter la connexion UDP**

Dans QGC : ⚙️ `Application Settings` → `Comm Links` → `Add`

| Paramètre | Valeur |
|-----------|--------|
| Name | `UGV Beast R&D` |
| Type | `UDP` |
| Listening Port | `14550` |
| Server Address | `192.168.50.107` |

Cliquer **OK** puis **Connect**.

<!-- 📸 IMAGE SUGGÉRÉE : Fenêtre Comm Links de QGC avec les paramètres -->

**3. Interface QGC une fois connecté**

La barre de statut en haut affiche :
- **Mode actif** : Manual / Navigation / Exploration / Return
- **État** : Armed (moteurs actifs) / Disarmed (moteurs inertes)
- **Batterie** : niveau en pourcentage

> ⚠️ Le GPS n'est pas utilisé. L'affichage satellite de QGC est purement visuel — la position du robot sur la carte QGC est basée sur l'odométrie ROS2, pas sur des coordonnées GPS réelles.

<!-- 📸 IMAGE SUGGÉRÉE : Interface QGroundControl connectée au robot avec la barre de statut -->

### Vérifier la connexion MAVROS depuis le robot

```bash
# État de la connexion MAVLink
ros2 topic echo /mavros/state --once

# Position locale estimée
ros2 topic echo /mavros/local_position/pose --once
```

---

## 8. WinTAK — Situation Awareness

WinTAK affiche le robot comme un acteur géolocalisé sur une carte tactique partagée. Le robot envoie sa position et son état sous forme de messages **CoT (Cursor on Target)** en UDP, et reçoit les commandes envoyées depuis le chat GeoChat de WinTAK.

### Prérequis

Lancer `mavros_exploration.py` sur le robot (voir section 5).

### Architecture

```
Robot (Jetson) — 192.168.50.107            PC Opérateur
┌────────────────────────────────┐         ┌──────────────────────────┐
│  mavros_exploration.py         │  UDP    │                          │
│  └─ nœud CoT ROS2              │────────►│   WinTAK                 │
│     (position + état → XML CoT)│ :8087   │   Marqueur robot         │
│     (écoute GeoChat → Nav2)    │◄────────│   sur la carte           │
└────────────────────────────────┘         └──────────────────────────┘
```

### Sur le PC — Installer et configurer WinTAK

**1. Installer WinTAK**

Télécharger depuis : [https://tak.gov](https://tak.gov) *(inscription professionnelle requise)*

**2. Ajouter un flux réseau d'écoute CoT**

Dans WinTAK : Menu ☰ → `Settings` → `Network` → `Manage connections` → `Add Stream`

| Paramètre | Valeur |
|-----------|--------|
| Protocol | `UDP` |
| Address | `0.0.0.0` *(écoute toutes interfaces)* |
| Port | `8087` |
| Description | `UGV Beast` |

Cliquer **OK** et activer le flux (toggle vert).

<!-- 📸 IMAGE SUGGÉRÉE : Fenêtre Network de WinTAK avec le flux UDP configuré -->

**3. Vérifier la réception**

Le robot apparaît dans le panneau **Contacts** de WinTAK sous le nom `UGV Beast`. Son marqueur se déplace sur la carte en temps réel au fil de son exploration.

<!-- 📸 IMAGE SUGGÉRÉE : WinTAK avec le marqueur UGV Beast visible et le panneau Contacts -->

### Envoyer des commandes au robot depuis WinTAK

Dans WinTAK, ouvrir le **chat GeoChat** et sélectionner le contact `UGV Beast`. Les commandes suivantes sont reconnues par le nœud ROS2 :

| Commande | Action |
|----------|--------|
| `START` | Lance l'exploration autonome |
| `STOP` | Stoppe le robot sur place |
| `RETURN` | Retour au point de départ |
| `END` | Arrêt complet du script |

<!-- 📸 IMAGE SUGGÉRÉE : WinTAK GeoChat avec les commandes START/STOP/RETURN/END envoyées -->

---

## 9. Utilisation avec les radios TW Spirit 860

Les radios **TrellisWare TW Spirit 860** permettent d'utiliser le robot **sans réseau WiFi**. Elles créent un réseau maillé IP autonome entre le robot et le PC opérateur, ce qui rend le système utilisable en terrain ouvert ou dans des zones sans infrastructure réseau.

### Comment ça fonctionne

Chaque radio TW Spirit 860 attribue une adresse IP aux équipements connectés, exactement comme un point d'accès WiFi. Du point de vue logiciel, le robot et le PC voient simplement une interface réseau supplémentaire. Tout le stack ROS2, QGroundControl et les messages CoT WinTAK transitent par ce lien radio **sans modification de configuration**.

```
┌──────────────────┐     liaison radio     ┌──────────────────┐
│  Robot           │  ~~~~~~~~~~~~~~~~~~~  │  PC Opérateur    │
│  Jetson Orin     │    TW Spirit 860      │  Windows         │
│  + Radio TW860   │  (réseau maillé IP)   │  + Radio TW860   │
│  (port Ethernet) │                       │  (port USB/ETH)  │
└──────────────────┘                       └──────────────────┘
         │                                          │
    ROS2 topics                              QGroundControl
    MAVROS                                   WinTAK CoT
    CoT messages                             RViz2 (X11)
```

### Mise en place physique

- **Radio robot** : montée sur le robot et connectée au **port Ethernet de la Jetson Orin Nano**
- **Radio PC** : branchée sur le PC opérateur (port USB ou Ethernet selon le modèle)

Les deux radios s'appairent et établissent leur liaison **automatiquement dès leur mise sous tension**. Aucune configuration manuelle n'est nécessaire lors des utilisations courantes.

### Utiliser le robot via radio

Une fois les deux radios allumées et appairées, utiliser les scripts `_radio` dédiés :

```bash
# Exploration autonome via radio (sans cartographie)
python3 auto_discovery_radio.py

# Exploration + cartographie via radio
python3 auto_exploration_radio.py
```

Le script `mavros_exploration.py` fonctionne également sur la liaison radio sans modification.

### Points d'attention

> ⚠️ **Test préalable recommandé** : avant une opération terrain, toujours vérifier la stabilité de la liaison en lançant `ros2 topic list` depuis le PC et en vérifiant que tous les topics du robot sont bien visibles.

```bash
# Vérifier la visibilité des topics depuis le PC via radio
export ROS_DOMAIN_ID=0    # doit être identique sur le robot et le PC
ros2 topic list
```

<!-- 📸 IMAGE SUGGÉRÉE : Photo des deux radios TW Spirit 860 (robot + PC) -->

---

## 🔗 Ressources

- [ROS2 Humble — Documentation](https://docs.ros.org/en/humble/)
- [Google Cartographer](https://google-cartographer-ros.readthedocs.io/)
- [Nav2 — Navigation Stack](https://navigation.ros.org/)
- [rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry)
- [explore_lite (m-explore)](https://github.com/hrnr/m-explore)
- [DepthAI ROS — OAK-D Lite](https://github.com/luxonis/depthai-ros)
- [MAVROS ROS2](https://github.com/mavlink/mavros)
- [QGroundControl](https://docs.qgroundcontrol.com/)
- [WinTAK — TAK.gov](https://tak.gov)
- [TW Spirit 860 — TrellisWare](https://www.trellisware.com/tw-spirit-860/)
- [MobaXterm](https://mobaxterm.mobatek.net/)
