#!/usr/bin/env python

#import websocket
#
## Define a callback function to handle messages
#def on_message(ws, message):
#	print(f"Received message: {message}")
#
## Define a callback function for errors
#def on_error(ws, error):
#	print(f"Error: {error}")
#
## Define a callback function for closing the connection
#def on_close(ws,a,b):
#	print("Connection closed")
#
## Define a callback function for opening the connection
#def on_open(ws):
#	print("Connection opened")
#	ws.send(b'\x10')  # Send a message
#
## Create a WebSocket application
#ws = websocket.WebSocketApp("wss://10.1.117.254/mqtt",  # Replace with your WebSocket URL
#							on_message=on_message,
#							on_error=on_error,
#							on_close=on_close)
#
## Assign the on_open function
#ws.on_open = on_open
#
## Run the WebSocket client
#ws.run_forever(sslopt={"certfile":"oem-default.pem","ca_certs":"CA-oem-default.pem","server_hostname":"oem-default"})



import random
from paho.mqtt import client as mqtt_client
import struct
import status
import trellis_pb2

broker = "oem-default"
port = 443
# Generate a Client ID with the subscribe prefix.
client_id = f'mqttjs_{random.randint(0, 0x100000000):08x}'

class trellisAPI():
	def __init__(self):
		self.mqtt = []

	def addMqttProtobuff(self,topic,pb_type,func):
		self.mqtt.append({'t':topic,'pb':pb_type,'f':func})

	def run(self):
		while True:
			try :
				client = self.connect_mqtt()
				client.loop_forever()
			except TimeoutError:
				print("Timeout Error, retrying...")

	def subscribe(self,client: mqtt_client):
		def on_message(client, self, msg):
			for i in range(len(self.mqtt)) :
				if mqtt_client.topic_matches_sub(self.mqtt[i]['t'],msg.topic):
					pb = self.mqtt[i]["pb"]();
					pb.ParseFromString(msg.payload)
					self.mqtt[i]["f"](pb)
					break

		print(f"{self.mqtt}")
		for i in range(len(self.mqtt)) :
			client.subscribe([(self.mqtt[i]['t'],0)])

		client.on_message = on_message


	def connect_mqtt(self) -> mqtt_client:
		def on_connect(client, self, flags, rc,properties):
			if rc == 0:
				print("Connected to MQTT Broker!")
				self.subscribe(client)
			else:
				print("Failed to connect, return code %d\n", rc)

		def on_log(client, self, level, buf):
			print(buf)

		client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2,client_id=client_id,transport='websockets',userdata=self)
		client.tls_set(
			ca_certs='./CA-oem-default.pem',
			certfile='./oem-default.pem'
		)
		client.on_connect = on_connect
		#client.on_log = on_log
		client.connect_async(broker, port)
		return client
