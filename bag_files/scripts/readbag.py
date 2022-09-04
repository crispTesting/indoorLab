import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
import os

path_csv = r'/home/robotlab/CriSp/braketest_1/vesc-odom.csv'
path_bag = r'/home/robotlab/CriSp/braketest_1.bag'

os.system('cls||clear')
print()

# This creates a new folder for the contents of the bag file.
b = bagreader(path_bag)

# Prints all available topics in the bagfile to the console
print()
print(b.topic_table)
print()

# This saves a specific topic to a csv file
#data = b.message_by_topic('/vesc/sensors/core')
#print("File saved: {}".format(data))

#data = b.message_by_topic('/vesc/odom')
#print("File saved: {}".format(data))

# Using pandas to read the data.
#df_imu = pd.read_csv(data)

# Using pandas to read straigt from csv file. 
# If you already have the csv files you can skip all lines above.
df_imu = pd.read_csv(path_csv)
print(df_imu)

# In paranteses choose how many plots to create
fig, ax = bagpy.create_fig(2)

# Example of plotting data from vesc-sensors-core.csv
#ax[0].scatter(x = 'Time', y = 'state.distance_traveled', data  = df_imu, s= 1, label = 'state.distance_traveled')

# Example of plotting data from vesc-odom.csv
ax[0].scatter(x = 'Time', y = 'pose.pose.position.x', data  = df_imu, s= 1, label = 'pose.pose.position.x')
ax[1].scatter(x = 'Time', y = 'twist.twist.linear.x', data  = df_imu, s= 1, label = 'twist.twist.linear.x')

# Use this to show the plotted data
plt.show()

