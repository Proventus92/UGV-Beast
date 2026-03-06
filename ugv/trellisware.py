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


broker = "oem-default"
port = 443
# Generate a Client ID with the subscribe prefix.
client_id = f'mqttjs_{random.randint(0, 0x100000000):08x}'


def subscribe(client: mqtt_client):
	def on_message(client, userdata, msg):
		if msg.topic.startswith("twt/tsm/v1/sa/node/environment_status/") :
			payload = decodeEnvironmentStatus(msg.payload,{})
		elif msg.topic.startswith("twt/tsm/v1/sa/node/network_status/") and len(msg.payload) > 0 :
			payload = decodeNetworkStatus(msg.payload,{})
		elif msg.topic.startswith("twt/tsm/v1/sa/node/links/") and len(msg.payload) > 0 :
			payload = decodeLinks(msg.payload,{})
			if 'uint_id' in payload and payload['uint_id'].lower() == userdata.current_radio.lower() :
				for link in payload['links'] :
					if 'destination_id' in link and link['destination_id'].lower() == userdata.radio_to_check.lower():
						print(f" quality {link['dst_link_quality_snr']}")
						if 'dst_link_quality_snr' in link :
							if link['dst_link_quality_snr'][0] < 30:
								userdata.action = status.Action.WAIT
								userdata.takevent.set()
							elif userdata.action == status.Action.WAIT :
								userdata.action = status.Action.SLEEP
								userdata.takevent.set()
		else :
			print(f"Received `{msg.payload.hex()}` from `{msg.topic}` topic")

	client.subscribe([("twt/tsm/v1/sa/node/environment_status/+",0),
				   ("twt/tsm/v1/sa/node/network_status/+",0),
				   ("twt/tsm/v1/sa/node/links/+",0)
				   ])
	client.on_message = on_message


def connect_mqtt(userdata) -> mqtt_client:
	def on_connect(client, userdata, flags, rc,properties):
		if rc == 0:
			print("Connected to MQTT Broker!")
			subscribe(client)
		else:
			print("Failed to connect, return code %d\n", rc)

	def on_log(client, userdata, level, buf):
		print(buf)

	client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2,client_id=client_id,transport='websockets',userdata=userdata)
	client.tls_set(
		ca_certs='./CA-oem-default.pem',
		certfile='./oem-default.pem'
	)
	client.on_connect = on_connect
	#client.on_log = on_log
	client.connect_async(broker, port)
	return client

def decodeuint32(data,i):
	c = (127 & data[i])
	if len(data) <= i+1 or data[i+1] < 128:
		return c,i+1;
	c = (c | (127 & data[i]) << 7)
	if len(data) <= i+2 or data[i+2] < 128:
		return c,i+2;
	c = (c | (127 & data[i]) << 14)
	if len(data) <= i+3 or data[i+3] < 128:
		return c,i+3;
	c = (c | (127 & data[i]) << 21)
	if len(data) <= i+4 or data[i+4] < 128:
		return c,i+4;
	c = (c | (15 & data[i]) << 28)
	if len(data) <= i+5 or data[i+5] < 128:
		return c,i+5;
	return c,i+10

def decodestring(data,i):
	l = data[i]
	i += 1
	return data[i:l+i].decode("utf-8"), i + l

def decodeRadioUpdateHeader(data,payload) :
	i = 0
	while(i < len(data)) :
		c = data[i] >> 3
		i += 1
		if c == 1:
			l = data[i]
			i += 1
			payload["uint_id"] = data[i:l+i].decode("utf-8")
			i += l
		elif c == 2:
			payload["expected_update_period"] = data[i]
			i += 1
	return payload

def decodedouble(data,i):
	return struct.unpack("<d",data[i:i+8]),i+8

def decodeEnvironmentStatus(data,payload) :
	i = 0
	while(i < len(data)) :
		c = data[i] >> 3
		i += 1
		if c == 1:
			l = data[i]
			i += 1
			payload = decodeRadioUpdateHeader(data[i:l+i],payload)
			i += l
		elif c == 2:
			payload["noise_floor"] = struct.unpack("<b",data[i:i+1])[0]
			i += 1
	return payload

def decodeNetworkStatus(data,payload) :
	i = 0
	while(i < len(data)) :
		c = data[i] >> 3
		i += 1
		if c == 1:
			l = data[i]
			i += 1
			payload = decodeRadioUpdateHeader(data[i:l+i],payload)
			i += l
		elif c == 2:
			payload["network_span"],i = decodeuint32(data,i)
		elif c == 3:
			payload["command_node"] = struct.unpack("<b",data[i:i+1])[0]
			i += 1
		elif c == 4:
			payload["ntr_quality"],i = decodeuint32(data,i)
		elif c == 5:
			payload["local"] = struct.unpack("<b",data[i:i+1])[0]
			i += 1
		elif c == 6:
			payload["jammed"] = struct.unpack("<b",data[i:i+1])[0]
			i += 1
		elif c == 7:
			payload["one_hop_neighbor"],i = decodeuint32(data,i)
		elif c == 8:
			payload["network_fingerprint"],i = decodeuint32(data,i)
		elif c == 9:
			payload["is_ntr"] = struct.unpack("<b",data[i:i+1])[0]
			i += 1
		elif c == 10:
			payload["data_mode"],i = decodeuint32(data,i)
	return payload

def decodegoogle_protobuf_Int32Value(data,i):
	c = data[i] >> 3
	i += 1
	#if c == 1:
	return decodeuint32(data,i)

def decodeLink(data,payload,i) :
	if 'links' not in payload :
		payload['links'] = []
	link = {}
	while(i < len(data)) :
		c = data[i] >> 3
		i += 1
		if c == 1:
			link['destination_id'],i = decodestring(data,i)
			#remove trailing \x00
			if link['destination_id'][:-1] == 0 :
				link['destination_id'] = link['destination_id'][:-1]
		elif c == 2:
			link['src_link_quality_snr'],i = decodedouble(data,i)
		elif c == 3:
			link['src_link_quality_percent'],i = decodeuint32(data,i)
		elif c == 4:
			link['dst_link_quality_snr'],i = decodedouble(data,i)
		elif c == 5:
			link['dst_link_quality_percent'],i = decodeuint32(data,i)
		elif c == 6:
			link['range'],i = decodedouble(data,i)
		elif c == 7:
			link['range_confidence'],i = decodeuint32(data,i)
		elif c == 8:
			link['src_link_rssi'],i = decodegoogle_protobuf_Int32Value(data,i)
		elif c == 9:
			link['dst_link_rssi'],i = decodegoogle_protobuf_Int32Value(data,i)

	payload['links'].append(link)

	return i

def decodeLinks(data,payload) :
	i = 0
	while(i < len(data)) :
		c = data[i] >> 3
		i += 1
		if c == 1:
			l = data[i]
			i += 1
			payload = decodeRadioUpdateHeader(data[i:l+i],payload)
			i += l
		if c == 2:
			l,i = decodeuint32(data,i)
			i = decodeLink(data[0:i+l],payload,i)
	return payload

def trellisware(s):
	while True:
		try :
			client = connect_mqtt(s)
			client.loop_forever()
		except TimeoutError:
			print("Timeout Error, retrying...")
