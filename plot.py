#!/cygdrive/c/Python39/python.exe

import os
import sys
import numpy as np
import re
import matplotlib.pyplot as plt

f = open("pwr.dat", "r")
pwrC = []
pwrV = []
rmsC = []
rmsV = []
data = False
for line in f:
    line = line.strip()
    if line == "pwr c":
        data = True
        pwr = pwrC
        continue
    elif line == "pwr v":
        data = True
        pwr = pwrV
        continue
    elif line == "rms c":
        data = True
        pwr = rmsC
        continue
    elif line == "rms v":
        data = True
        pwr = rmsV
        continue
    elif data:
        line = re.sub(r' +', ' ', line)
        line = line.split(' ')
        for val in line:
            pwr.append(int(val))
    else:
        data = False
f.close();

plt.plot(pwrV)
plt.plot(pwrC)
plt.show()
