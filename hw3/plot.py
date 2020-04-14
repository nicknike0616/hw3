import matplotlib.pyplot as plt
import numpy as np
import serial
import time

t = np.arange(0,10,0.1) 
X = np.arange(0,10,0.1) 
Y = np.arange(0,10,0.1) 
Z = np.arange(0,10,0.1)
tilt = np.arange(0,10,0.1) 

serdev = '/dev/ttyACM0'

s = serial.Serial(serdev,115200)

for x in range(0, 100):
    line=s.readline()
    # print line
    X[x] = float(line)

for x in range(0, 100):
    line=s.readline()
    # print line
    Y[x] = float(line)

for x in range(0, 100):
    line=s.readline()
    # print line
    Z[x] = float(line)

for x in range(0, 100):
    line=s.readline()
    # print line
    tilt[x] = float(line)

fig, ax = plt.subplots(2, 1)

l1,=ax[0].plot(t,X)
l2,=ax[0].plot(t,Y)
l3,=ax[0].plot(t,Z)
ax[0].set_xlabel('Time')
ax[0].set_ylabel('Acc Vector')
ax[0].legend((l1,l2,l3),('X','Y','Z'))
ax[1].stem(t,tilt,'b') 
ax[1].set_xlabel('Time')
ax[1].set_ylabel('Tilt')
plt.show()
s.close()