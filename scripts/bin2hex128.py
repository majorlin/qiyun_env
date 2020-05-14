#!/usr/bin/env python3
# -*- coding=utf8 -*-
"""
# Author: Major Lin
# Created Time : 06/11/19 14:31:03
# File Name: bin2hex.py
# Description:
"""

import sys
import struct

with open(sys.argv[1], "rb") as f:
    byte1 = f.read(4)
    while byte1:
        byte2 = f.read(4)
        byte3 = f.read(4)
        byte4 = f.read(4)
        if byte4:
            print("{:0>8X}".format(struct.unpack('I', byte4)[0]), end="")
        else:
            print("00000000", end="")
        if byte3:
            print("{:0>8X}".format(struct.unpack('I', byte3)[0]), end="")
        else:
            print("00000000", end="")
        if byte2:
            print("{:0>8X}".format(struct.unpack('I', byte2)[0]), end="")
        else:
            print("00000000", end="")
        print("{:0>8X}".format(struct.unpack('I', byte1)[0]))
        byte1 = f.read(4)
