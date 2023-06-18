import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

filename = 'test_log.csv'

flightLog = pd.read_csv(filename)
time = 0.000001 * flightLog[['Time']]
altitude = flightLog[['Altitude']]
throttle = flightLog[['Throttle']]
plt.plot(time,altitude)
plt.plot(time,throttle)
plt.title("PID")
plt.xlabel('time')
plt.ylabel('altitude / throttle')
plt.draw()
plt.show()