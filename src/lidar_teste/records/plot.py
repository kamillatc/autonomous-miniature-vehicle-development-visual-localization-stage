import yaml
import numpy as np
import matplotlib.pyplot as plt

with open('scans.yaml', 'r') as f:
    scans = yaml.safe_load(f)

scan = scans[0]  # pega o primeiro

ranges = np.array(scan['ranges'])
intensities = np.array(scan['intensities'])
angle_min = scan['angle_min']
angle_increment = scan['angle_increment']

angles = angle_min + np.arange(len(ranges)) * angle_increment

x = ranges * np.cos(angles)
y = ranges * np.sin(angles)

plt.figure(figsize=(6,6))
plt.scatter(x, y, c=intensities, s=3, cmap='inferno')
plt.colorbar(label='Intensidade')
plt.axis('equal')
plt.title("LiDAR com Intensidade")
plt.show()