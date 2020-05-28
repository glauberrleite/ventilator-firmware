import serial
import numpy as np
import time
import signal
import sys
import pandas as pd

def signal_handler(sig, frame):
    ser.close()
    
    df = pd.DataFrame(y, columns=["time", "state", "fl_int", "fl_pac_ins", "fl_pac_exp", "pres_pac", "pres_int"])
    df.to_csv('list.csv', index=False)
    print("XAU")

    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

k = 0
Ts = 0.01
x=list()
y=list()
k = 0
ser = serial.Serial('/dev/ttyACM0', 9600)
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
    
    y.append([k * Ts, float(state), float(fl_int), float(fl_pac_ins), float(fl_pac_exp), float(pres_pac), float(pres_int)])

    #plt.scatter(k * Ts, float(pres_pac), c='blue')
    #plt.plot(x, y, linewidth=2, c='blue')

    #print(state + '\t' + fl_int + '\t' + fl_pac_ins + '\t' + fl_pac_exp + '\t' + pres_pac + '\t' + pres_int)

    k += 1

    time.sleep(Ts)
    #lt.show()
    #plt.pause(0.00001)  # Note this correction
