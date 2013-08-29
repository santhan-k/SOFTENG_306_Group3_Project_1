#! /usr/bin/env python
from subprocess import Popen, PIPE
import subprocess
import sys
import random

fieldNumber = int(sys.argv[1])
openclose = int(sys.argv[2])

if (fieldNumber == 1):
  gate1 = 18
  gate2 = 19
elif (fieldNumber == 2):
  gate1 = 20
  gate2 = 21
elif (fieldNumber == 3):
  gate1 = 22
  gate2 = 23
elif (fieldNumber == 4):
  gate1 = 24
  gate2 = 25

subprocess.Popen("rosrun alpha_two HerdingBar "+str(gate1)+"  "+str(sys.argv[2]),shell=True,stdout = PIPE)

subprocess.Popen("rosrun alpha_two HerdingBar "+str(gate2)+"  "+str(sys.argv[2]),shell=True,stdout = PIPE)
