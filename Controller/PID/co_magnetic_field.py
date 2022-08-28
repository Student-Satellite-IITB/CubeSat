# Author: Ajay Tak
# Date: 31-07-2022

from turtle import shape
from pyIGRF import *
from co_frames import *
import numpy as np

time = np.arange(0, 54000, 0.1)
omega_orbit = (2*np.pi)/5400  # T = 5400 sec is the assumed time period of our satellite in a orbit
theta_orbit = omega_orbit*time # value of theta at different time steps
r = 400 # altitude in km
v = 400000*omega_orbit # magnitude of velocity of cubesat in m/s
x = r*np.cos(theta_orbit) # x-component in perifocal plane
y = r*np.sin(theta_orbit) # y-component in perifocal plane
inc = 0 # inclination of orbit in radians
X = x # x-component in ECIF
Y = y*np.cos(inc) # y-component in ECIF
Z = y*np.sin(inc) # z-component in ECIF
v_X = -v*np.sin(theta_orbit)
v_Y = v*np.cos(theta_orbit)
v_Z = np.zeros_like(v_X)
lat = (180/np.pi)*np.arcsin(Z/r) # latitude array at different time steps in degrees
long = (180/np.pi)*np.arctan2(Y, X) # longitude array at different time steps in degrees
date = 2020.25
B = []
for i in range(len(time)):
    B.append(igrf_value(lat[i], long[i], 400, date)[3:6]) # magnetic field in NED frame in nT

for i in range(len(time)):
    B[i] = ned2ecef(B[i], lat[i], long[i]) # magnetic field in ECEF frame in nT

for i in range(len(time)):
    B[i] = ecef2ecif(B[i]) # magnetic field in ECI frame in nT

for i in range(len(time)):
    B[i] = ecif2orbit(np.array([X[i], Y[i], Z[i]]), np.array([v_X[i], v_Y[i], v_Z[i]]), (1e-9)*B[i]) # magnetic field in Orbit frame in T

B = np.array(B)
print(np.size(B))
np.savetxt('magfield.csv', B)




