#! /usr/bin/env python
from subprocess import Popen, PIPE
import subprocess

subprocess.Popen("killall R1",shell=True)
subprocess.Popen("killall R0",shell=True)
