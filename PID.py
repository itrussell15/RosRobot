#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 29 16:11:50 2022

@author: schmuck
"""

import datetime

class PI_Control:
    
    def __init__(self, kp, ki, kd, target = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self._target = target
        
        self._P = 0
        self._I = 0
        # self._D = 0
        
        self._reset = 0
        self._previous = 0
        
        self._last_sample = None
        
    def set_target(self, target):
        self._target = target
        
    # def start(self):
    #     self._start = datetime.datetime.now()
    #     self._last_sample = self._start 
    
    def gain(self, val):
        if self._last_sample == None:
            self._last_sample = datetime.datetime.now()
        
        error = self._target - val
#        print("error {}".format(error))
        now = datetime.datetime.now()
        tau = now - self._last_sample
        self._last_sample = now
        
        self._P = self.kp * error
        self._I = self._integralResponse(error, tau.total_seconds())
        # print("P = {:.2f} I = {:.2f} --> Input = {:.2f}".format(self._P, self._I, val))
        return self._P + self._I + val
        # self._D = self._derivativeResponse(error, tau.total_seconds())
        
        # print("P = {:.2f}".format(self._P))
        # return self._P + self._I + self._D
        
    def _integralResponse(self, val, tau):
        return self._I + self.ki * val * tau
    
    # def _derivativeResponse(self, val, tau):
    #     # D = Kd*(e - e_prev)/(t - t_prev)
    #     # out = self.kd * val + (self.kd / tau) * (val - self._previous)
    #     out = self.kd * (val - self._previous) / tau
    #     self._previous = val
    #     return out
    

        
