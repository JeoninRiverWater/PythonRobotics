import numpy as np
import matplotlib.pyplot as plt

map_array = np.loadtxt('c:/Users/User/OneDrive/바탕 화면/coding/AutoServingRobot/PythonRobotics/Mapping/lidar_to_grid_map/output.txt', delimiter=',')

plt.imshow(map_array, cmap='gray', interpolation='none')
plt.show()