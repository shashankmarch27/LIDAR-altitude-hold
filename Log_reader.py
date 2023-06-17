import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

flightLog = pd.read_csv('E:/log.csv')
time = flightLog[['Time']]
altitude = flightLog[['Altitude']]
plt.plot(time,altitude)
plt.title("PID")
plt.xlabel('time')
plt.ylabel('altitude')
plt.draw()
plt.show()