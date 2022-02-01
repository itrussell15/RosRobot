#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 29 17:13:44 2022

@author: schmuck
"""

from PID import PI_Control
import time, datetime
import matplotlib.pyplot as plt


control = PI_Control(
    kp = 0.9,
    ki = 0.35,
    kd = 0)

target = 75
control.set_target(target)

start = -1
current = start
store = [start]

x = [i for i in range(1, 50)]

for i in x:
    current += control.gain(current)
    if i == 5 or i == 10:
        current += 100
    # print(current)
    store.append(current)
    time.sleep(1)
    
x_show = [0]
x_show.extend(x)    
plt.plot(x_show, store)
plt.axhline(y=target, color='r', linestyle='--')
