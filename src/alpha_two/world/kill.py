#! /usr/bin/env python
from subprocess import Popen, PIPE
import subprocess

subprocess.Popen("killall R1",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall R0",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall stageros",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall grass",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall HerdingBar",shell=True,stdout = PIPE, stderr = PIPE)
subprocess.Popen("killall Gates",shell=True,stdout = PIPE, stderr = PIPE)
