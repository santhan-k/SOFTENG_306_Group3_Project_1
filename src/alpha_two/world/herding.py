#! /usr/bin/env python
from subprocess import Popen, PIPE
import subprocess
import sys
import random

fieldNumber = int(sys.argv[1])
openclose = int(sys.argv[2])

if (fieldNumber == 1):
  bar1 = 18
  bar2 = 19
  gate = 5
elif (fieldNumber == 2):
  bar1 = 20
  bar2 = 21
  gate = 8 
elif (fieldNumber == 3):
  bar1 = 22
  bar2 = 23
  gate = 6 
elif (fieldNumber == 4):
  bar1 = 24
  bar2 = 25
  gate = 9

subprocess.Popen("rosrun alpha_two HerdingBar "+str(bar1)+"  "+str(sys.argv[2]),shell=True,stdout = PIPE)

subprocess.Popen("rosrun alpha_two HerdingBar "+str(bar2)+"  "+str(sys.argv[2]),shell=True,stdout = PIPE)

subprocess.Popen("rosrun alpha_two Gates "+str(gate)+"  "+str(sys.argv[2]),shell=True,stdout = PIPE)
