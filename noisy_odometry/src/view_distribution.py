from math import *
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
# 20cm translation distance
dx=0.2
dy=0.35
trans = sqrt(dx*dx + dy*dy)
# rotation1 = 30 degree
theta1 = 30.0*pi/180.0
# rotation2 = 45 degree
theta2 = 45.0*pi/180.0

rot1 = atan2(dy, dx) - theta1
rot2 = theta2-theta1-rot1

a1 = 15.0*pi/180.0
a2 = 15.0*pi/180.0
a3 = 0.2
a4 = 0.01
sd_rot1 = a1*abs(rot1) + a2*trans
sd_rot2 = a1*abs(rot2) + a2*trans
sd_trans = a3*trans + a4*(abs(rot1) + abs(rot2))


x= []
y = []

for i in range(0, 1000):
    t = trans  + np.random.normal(0,sd_trans*sd_trans)
    r1 = rot1 + np.random.normal(0, sd_rot1*sd_rot1)
    r2 = rot2 + np.random.normal(0, sd_rot2*sd_rot2)
    x.append(t*cos(theta1+r1))
    y.append(t*sin(theta1+r1))



#matplotlib.rcParams['axes.unicode_minus'] = False
fig, ax = plt.subplots()
#ax.plot(x, y)
ax.scatter(x, y)
ax.set_title('Gaussian noise of motion model with dx=0.2 and dy=0.35')
plt.show()