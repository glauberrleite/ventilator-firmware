import serial
import numpy as np
import time
import signal
import sys

def signal_handler(sig, frame):
    ser.close()
    
    # Implement code here
    print("XAU")

    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

k = 0
Ts = 0.01
x=list()
y=list()
k = 0
ser = serial.Serial('COM9', 9600)
ser.close()
ser.open()
#ser.write("START".encode())

state = ""
fl_int = ""
fl_pac_ins = ""
fl_pac_exp = ""
pres_pac = ""
pres_int = ""

while True:
    data = ser.readline().decode('utf-8')

    print(data)
    state, fl_int, fl_pac_ins, fl_pac_exp, pres_pac, pres_int = data.split('\t')
    
    x.append(k * Ts)
    #y.append(float(pres_pac))

    #plt.scatter(k * Ts, float(pres_pac), c='blue')
    #plt.plot(x, y, linewidth=2, c='blue')

    #print(state + '\t' + fl_int + '\t' + fl_pac_ins + '\t' + fl_pac_exp + '\t' + pres_pac + '\t' + pres_int)

    k += 1

    time.sleep(Ts)
    #lt.show()
    #plt.pause(0.00001)  # Note this correction
