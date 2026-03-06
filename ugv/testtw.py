#!/usr/bin/env python
import tw
import trellis_pb2

def printEnvironementStatus(ev):
	print(f"EnvStat {ev}")

a = tw.trellisAPI()

a.addMqttProtobuff("twt/tsm/v1/sa/node/links/+",trellis_pb2.Links,printEnvironementStatus)

a.run()
