import serial
import numpy as np
import time
import signal
import sys
import pandas as pd
import matplotlib.pyplot as plt

def signal_handler(sig, frame):
    ser.close()
    
    df = pd.DataFrame({'time':l_time, 'state':l_state, 'fl_int':l_fl_int, 'pres_int':l_pres_int, 'pres_pac':l_pres_pac, 'pres_exp':l_pres_exp, 'fl_pac':l_fl_pac})
    df.to_csv('list.csv', index=False)
    print("XAU")

    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#plt.ion()
#fig=plt.figure()

k = 0
Ts = 0.01
l_time = list()
l_state = list()
l_fl_int = list()
l_fl_pac = list()
l_pres_exp = list()
l_pres_pac = list()
l_pres_int = list()
k = 0
ser = serial.Serial('/dev/ttyACM0', 9600)
ser.close()
ser.open()
#ser.write("START".encode())

state = ""
fl_int = ""
fl_pac = ""
pres_pac = ""
pres_int = ""
pres_exp = ""

while True:
    data = ser.readline().decode('utf-8')

    #print(data)
    state, fl_int, pres_int, pres_pac, pres_exp, fl_pac = data.split('\t')
    
    l_time.append(k * Ts)
    l_state.append(state)
    l_fl_int.append(float(fl_int))
    l_fl_pac.append(float(fl_pac))
    l_pres_int.append(float(pres_int))
    l_pres_pac.append(float(pres_pac))
    l_pres_exp.append(float(pres_exp))

    #plt.scatter(k * Ts, float(pres_pac), c='blue')

    #plt.cla()
    #plt.plot(l_time[len(l_time)-100:len(l_time)-1], l_state[len(l_state)-100:len(l_state)-1], linewidth=2, c='blue')

    #print(state + '\t' + fl_int + '\t' + fl_pac_ins + '\t' + fl_pac_exp + '\t' + pres_pac + '\t' + pres_int)
    #print(y[:,0])

    k += 1

    time.sleep(Ts)
    #plt.show()
    #plt.pause(Ts)  # Note this correction
