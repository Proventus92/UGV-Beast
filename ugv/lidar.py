#!/usr/bin/env python
import sys
import takproto
import socket
import multiprocessing
import math
import threading


 # adding Folder_2 to the system path
sys.path.insert(0, '/home/jetson/ugv_jetson/ugv-env/lib/python3.10/site-packages/')
from ugv_jetson.base_ctrl import BaseController
import time
import serial
import struct

crc_table = [ 0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
	0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5, 0x1F,
	0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43, 0xB6, 0xFB,
	0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA, 0x3E, 0x73, 0xA4,
	0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62, 0x97, 0xDA, 0x0D, 0x40,
	0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB, 0x21, 0x6C, 0xBB, 0xF6, 0x58,
	0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D, 0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC,
	0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4, 0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F,
	0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20, 0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B,
	0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89, 0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91,
	0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F, 0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75,
	0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96, 0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A,
	0x67, 0xC9, 0x84, 0x53, 0x1E, 0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE,
	0x60, 0x2D, 0xFA, 0xB7, 0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6,
	0x9B, 0x4C, 0x01, 0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32,
	0xE5, 0xA8,
]

def commands():
	base.send_command({"T":1,"L":0.2,"R":0.2})
	base.send_command({"T":134,"X":0,"Y":0,"SX":150,"SY":150})


class LIDAR:
	def __init__(self, serial_port, baudrate):
		self.PACKET_LENGTH = 49
		self.POINT_PER_PACK = 12
		self.serial_conn = serial.Serial(serial_port, baudrate=baudrate, timeout=1)
		self.data = []

	def calculate_crc8(self, data):
		crc = 0x00
		for byte in data:
			crc = crc_table[(crc ^ byte) & 0xFF]
		return crc

	def parse_packet(self, packet):
		if len(packet) != self.PACKET_LENGTH:
			print("Invalid packet length.")
			return

		if packet[0] != 0x54 or packet[1] != 0x2C:
			print("Invalid packet header")
			return

		received_crc = packet[self.PACKET_LENGTH - 3]
		calculated_crc = self.calculate_crc8(packet[: self.PACKET_LENGTH - 3])
		if received_crc != calculated_crc:
			print("CRC8 checksum mismatch")
			return

		header, ver_len, speed, start_angle = struct.unpack("<BBHH", packet[:6])

		points = []
		offset = 6  # 4 + 2 bytes
		for _ in range(self.POINT_PER_PACK):
			distance, intensity = struct.unpack(
				"<HB", packet[offset : offset + 3]
			)  # 3-> 2+1 for slicing
			points.append({"distance": distance, "intensity": intensity})
			offset += 3

		end_angle, timestamp = struct.unpack("<HH", packet[42:46])

		# convert angles from hundredth of a degree to degrees
		start_angle = (start_angle % 36000) / 100.0
		end_angle = (end_angle % 36000) / 100.0

		# calculate angle increment
		angle_diff = (end_angle - start_angle + 360.0) % 360.0
		angle_increment = angle_diff / 11  # 12 points, so 11 intervals

		# interpolate angles for each point
		angles = [
			(start_angle + i * angle_increment) % 360.0
			for i in range(self.POINT_PER_PACK)
		]

		scan_data = []
		for i, point in enumerate(points):
			scan_point = {
				"angle": angles[i],
				"distance": point["distance"],
				"intensity": point["intensity"],
			}
			scan_data.append(scan_point)

		return {
			"speed": speed,
			"start_angle": start_angle,
			"end_angle": end_angle,
			"timestamp": timestamp,
			"scan_data": scan_data,
		}

	def read_lidar_data(self,frm,to):
		ret = -1
		data = self.data
		first = None
		prev = None
		gt = True
		if len(data) > 0:
			first = data[0]['angle']
		if len(data) > 1:
			prev = data[-1]['angle']

		while ret < 0 :
			# read until header and ver_len values are found
			packet = self.serial_conn.read_until(b"\x54\x2C")
			while len(packet) != (self.PACKET_LENGTH - 2):
				packet = self.serial_conn.read_until(b"\x54\x2C")

			packet = bytes(b"\x54\x2C") + packet  # needed for checksum verification

			p = self.parse_packet(packet)
			if p is not None :
				for d in p['scan_data'] :
					if d['intensity'] > 0 and ( d['angle'] <= to or d['angle'] >= frm ) :
						data.append(d)
					if ret < 0:
						if prev is not None and gt and prev > d['angle'] :
							gt = False
						if not gt and d['angle'] > first:
							ret = len(data)-1
						if first is None:
							first = d['angle']
						else:
							prev = d['angle']

		self.data = data[ret:]
		return data[:ret]

	def close_serial_connection(self):
		self.serial_conn.close()

	def debug_server(self):
		while True :
			try:
				with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
					s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
					s.bind(("0.0.0.0", 12345))
					s.listen()
					conn, addr = s.accept()
					with conn:
						while True:
							conn.recv(1024)
							conn.sendall(self.data)
			except ConnectionResetError :
				print("Wait new cennection")

			
#lidar = LIDAR(serial_port="/dev/ttyACM0", baudrate=230400)
#base = BaseController('/dev/ttyTHS1', 115200)
#s = {"data":b''}
#debug = threading.Thread(target=debug_server,args=(s,))
#debug.start()
#
#while True:
#	data = lidar.read_lidar_data(360,180)
#	forward = True
#	message = b''
#	for point in data:
#		message = message + struct.pack("<ff",point['angle'],point['distance'])
#		x = math.cos(point['angle']*math.pi/180)*point['distance']
#		y = math.sin(point['angle']*math.pi/180)*point['distance']
#		if x >= -130 and x <= 130 :
#			if y >= 0 and y <= 250 :
#				forward = False
#	if forward :
#		base.send_command({"T":1,"L":0.2,"R":0.2})
#	else :
#		base.send_command({"T":1,"L":-0.2,"R":0.2})
#	s['data'] = message
