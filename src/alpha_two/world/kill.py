#! /usr/bin/env python
from subprocess import Popen, PIPE
import subprocess

#Kill all processes that are created by pyGen.py
subprocess.Popen("killall R1",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall stageros",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall grass",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall HerdingBar",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall Gates",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall Farmer",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall farm",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall cloud",shell=True,stdout = PIPE, stderr = PIPE)
