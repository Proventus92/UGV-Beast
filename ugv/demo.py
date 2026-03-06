#!/usr/bin/env python
import sys
import threading
import time
import atak_link
import trellisware
import status
import socket
import takproto
import xml.etree.ElementTree as ET
from argparse import ArgumentParser
sys.path.insert(0, '/home/jetson/ugv_jetson/ugv-env/lib/python3.10/site-packages/')
from ugv_jetson.base_ctrl import BaseController
import lidar
import struct
import math

def navigation(s):
	while s.running:
		data = lid.read_lidar_data(360,180)
		forward = True
		message = b''
		for point in data:
			message = message + struct.pack("<ff",point['angle'],point['distance'])
			x = math.cos(point['angle']*math.pi/180)*point['distance']
			y = math.sin(point['angle']*math.pi/180)*point['distance']
			if x >= -130 and x <= 130 :
				if y >= 0 and y <= 250 :
					forward = False
		if forward :
			base.send_command({"T":1,"L":0.2,"R":0.2})
		else :
			base.send_command({"T":1,"L":-0.2,"R":0.2})
	base.send_command({"T":1,"L":0,"R":0})

def orders(s):
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.bind((s.interface_ip,s.atak_chat_port))
	nav = None
	while True:
		message, address = sock.recvfrom(1024)
		event = takproto.parse_proto(message).cotEvent
		detail = ET.fromstring("<event>"+event.detail.xmlDetail+"</event>")
		for child in detail:
			if child.tag == "remarks" :
				if nav is None and child.text == "go" :
					s.running = True
					nav = threading.Thread(target=navigation,args=(s,))
					nav.start()
				elif nav is not None and child.text == "stop":
					s.running = False
					nav.join()
					nav = None

parser = ArgumentParser()

parser.add_argument("-i", "--interface", dest="interface",help="Address ip of interface", default="192.168.50.130")

args = parser.parse_args()

s = status.Status()
s.interface_ip = args.interface
s.atak_chat_port = 17012
s.radio_to_check = "00:1e:3f:16:13:e0"
s.current_radio = "00:1e:3f:16:13:d0"
s.ugv_pos = [48.885212,2.513488]
s.target_pos = [48.885212,2.313488]

lid = lidar.LIDAR(serial_port="/dev/ttyACM0", baudrate=230400)
base = BaseController('/dev/ttyTHS1', 115200)

atak = threading.Thread(target=atak_link.atak_link,args=(s,))
trellis = threading.Thread(target=trellisware.trellisware,args=(s,))
drone = threading.Thread(target=orders,args=(s,))

debug = threading.Thread(target=lid.debug_server)
debug.start()

trellis.start()
atak.start()
drone.start()

atak.join()
trellis.join()
drone.join()
