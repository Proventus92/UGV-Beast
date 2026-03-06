#!/usr/bin/env python3
from enum import Enum
import asyncio

class Action(Enum):
    SLEEP = 1
    WAIT = 2
    TARGET = 3
    HOME = 4

class Status():
    action = Action.SLEEP.value
    interface_ip = "0.0.0.0"
    radio_to_check = "00:00:00:00:00:00"
    current_radio = "00:00:00:00:00:00"
    ugv_pos = [0.0,0.0]
    target_pos = [0.0,0.0]
    takevent = asyncio.Event()
    running = False
