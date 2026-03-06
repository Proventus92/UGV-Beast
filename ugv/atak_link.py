#!/usr/bin/env python3

import asyncio
import xml.etree.ElementTree as ET

from configparser import ConfigParser

import pytak
import status
import takproto

class Receiver(pytak.QueueWorker):
	"""Defines how you will handle events from RX Queue."""

	async def handle_data(self, data):
		"""Handle data from the receive queue."""
		#print(takproto.parse_proto(data))

	async def run(self):
		"""Read from the receive queue, put data onto handler."""
		while True:
			data = (
				await self.queue.get()
			)  # this is how we get the received CoT from rx_queue
			await self.handle_data(data)
	
	def set_status(self,s):
		self.s = s

class CotMsgs(pytak.QueueWorker):
	"""
	Defines how you process or generate your Cursor on Target Events.
	From there it adds the CoT Events to a queue for TX to a COT_URL.
	"""

	async def handle_data(self, data):
		"""Handle pre-CoT data, serialize to CoT Event, then puts on queue."""
		event = data
		await self.put_queue(event)

	async def message_timer(self):
		await asyncio.sleep(20)
		self.s.takevent.set()

	async def run(self):
		"""Run the loop for processing or generating pre-CoT data."""
		target_set=False
		asyncio.create_task(self.message_timer())
		while True:
			if self.s.action == status.Action.WAIT :
				data = self.target_cot(pytak.cot_time(3600))
				target_set=True
				await self.handle_data(data)
			elif target_set and self.s.action == status.Action.SLEEP :
				data = self.target_cot(pytak.cot_time(0))
				target_set=False
				await self.handle_data(data)
			data = self.ugv_cot()
			await self.handle_data(data)
			await self.s.takevent.wait()
			self.s.takevent.clear()
	
	def set_status(self,s):
		self.s = s

	def target_cot(self,stale):
		"""Generate a simple takPong CoT Event."""
		root = ET.Element("event")
		root.set("version", "2.0")
		root.set("type", "a-p-G")
		root.set("uid", "UGV-target")
		root.set("how", "m-g")
		root.set("time", pytak.cot_time())
		root.set("start", pytak.cot_time())
		root.set("stale", stale)
		point = ET.SubElement(root,"point")
		point.set("lat",str(self.s.target_pos[0]))
		point.set("lon",str(self.s.target_pos[1]))
		point.set("hae","0")
		point.set("ce","9999999")
		point.set("le","9999999")
		detail = ET.SubElement(root,"detail")
		__group = ET.SubElement(detail,"__group")
		__group.set("name","Red")
		__group.set("role","XX")
		return ET.tostring(root)

	def ugv_cot(self):
		"""Generate a simple takPong CoT Event."""
		"""<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
			<event version='2.0' uid='aa0b0312-b5cd-4c2c-bbbc-9c4c70216261' type='a-f-G-E-V-C' time='2020-02-08T18:10:44.000Z' start='2020-02-08T18:10:44.000Z' stale='2020-02-08T18:11:11.000Z' how='h-e'>
			<point lat='43.97957317' lon='-66.07737696' hae='26.767999' ce='9999999.0' le='9999999.0' />
			<detail>
				<uid Droid='UGV'/>
				<contact callsign='UGV' endpoint='192.168.50.5:4242:udp'/>
				<__group name='Yellow' role='HQ'/><status battery='100'/>
				<takv platform='UGV' device='jetson' os='Ubuntu' version='1.10.0.137'/>
				<track speed='0.00000000' course='0.00000000'/>
			</detail>
		</event>
		"""
		root = ET.Element("event")
		root.set("version", "2.0")
		root.set("type", "a-f-G-E-V-C")
		root.set("uid", "UGV-WAVE-RESCUE")
		root.set("how", "m-g")
		root.set("time", pytak.cot_time())
		root.set("start", pytak.cot_time())
		root.set("stale", pytak.cot_time(3600))
		point = ET.SubElement(root,"point")
		point.set("lat",str(self.s.ugv_pos[0]))
		point.set("lon",str(self.s.ugv_pos[1]))
		point.set("hae","0")
		point.set("ce","9999999.0")
		point.set("le","9999999.0")
		detail = ET.SubElement(root,"detail")
		uid = ET.SubElement(detail,"uid")
		uid.set("Droid","UGV")
		contact = ET.SubElement(detail,"contact")
		contact.set("callsign","UGV")
		contact.set("endpoint",self.s.interface_ip+":"+str(self.s.atak_chat_port)+":udp")
		__group = ET.SubElement(detail,"__group")
		__group.set("name","Yellow")
		__group.set("role","HQ")
		return ET.tostring(root)

def atak_link(s):
	asyncio.run(atak_link2(s))

async def atak_link2(s):
	config = ConfigParser()
	config["mycottool"] = {"COT_URL": "udp://239.2.3.1:6969","PYTAK_MULTICAST_LOCAL_ADDR": s.interface_ip}
	config = config["mycottool"]

	# Initializes worker queues and tasks.
	clitool = pytak.CLITool(config)
	await clitool.setup()

	c = CotMsgs(clitool.tx_queue, config)
	c.set_status(s)

	r = Receiver(clitool.rx_queue, config)
	r.set_status(s)

	# Add your serializer to the asyncio task list.
	clitool.add_tasks(set([c,r]))

	# Start all tasks.
	await clitool.run()

