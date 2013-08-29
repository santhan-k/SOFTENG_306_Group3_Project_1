#! /usr/bin/env python
from subprocess import Popen, PIPE
import subprocess
import sys
import random


if (len(sys.argv)!=3):
    print ("Usage:  python herding.py <Field Number>  <0 - Open gate/inititate herding, 2 - Close gate/reset herding>")
    sys.exit()

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

if (fieldNumber==4 and int(sys.argv[2])==2):
  subprocess.Popen("rosrun alpha_two truck ",shell=True,stdout = PIPE)
