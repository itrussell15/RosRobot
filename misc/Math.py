#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  1 21:06:38 2022

@author: schmuck
"""

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)