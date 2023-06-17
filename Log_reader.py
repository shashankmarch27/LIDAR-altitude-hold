import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

flightLog = pd.read_csv('/Users/reeshavchowdhury/Downloads/log.csv')
time = df[['Time']]
altitude = df[['Altitude']]
plt.plot(time,altitude)
plt.title("PID")
plt.xlabel('time')
plt.ylabel('altitude')
plt.draw()
plt.show()
print(flightLog)