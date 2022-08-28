
import numpy as np
from math import radians, sin, cos, acos, pi
from co_constants import W_EARTH, EPOCH, EQUINOX, STEPRUT

def ned2ecef(v,lat,lon):
	#rotate vector from North-East-Down frame to ecef
	#Input:
	#	lat: latittude in degrees ranges from -90 to 90
	#	lon: longitude in degrees ranges from (-180,180]	 
	if lat == 90 or lat == -90:
		raise ValueError('Latittude value +/-90 occured. NED frame is not defined at north and south pole !!')
	theta = -lat + 90. #in degree, polar angle (co-latitude)

	if lon<0:
		phi = 360. - abs(lon)
	else:
		phi = lon #in degree, azimuthal angle
	theta = radians(theta)
	phi = radians(phi)

	m_DCM_n2e = np.array([[ -cos(theta)*cos(phi),	-sin(phi),	-sin(theta)*cos(phi)],
						[	-cos(theta)*sin(phi),	cos(phi),	-sin(theta)*sin(phi)],
						[	sin(theta),	0.0,	-cos(theta)]])
	
	y = np.dot(m_DCM_n2e,v)

	return y

def ecef2ecif(v_x_e):
	#Input ecef cector
	# output ecif vector
	ut_sec = (EPOCH - EQUINOX).total_seconds()# universal time vector in sec
	theta = W_EARTH*ut_sec #in radian
	m_DCM = np.array([[cos(theta), -1*sin(theta), 0.], [sin(theta), cos(theta),0.],[ 0.,0.,1.]])
	v_x_i = np.dot(m_DCM,v_x_e)
	
	return v_x_i

def ecif2orbit(v_pos_i,v_vel_i,v_x_i):
	#Input: v_pos_i is position in eci frame , v_vel_i is velocity in eci frame, v_x_i is vector to be transformed
	#output: vector components in orbit frame
	z = -v_pos_i/np.linalg.norm(v_pos_i)
	y = np.cross(v_vel_i,v_pos_i)/np.linalg.norm(np.cross(v_vel_i,v_pos_i))
	x = np.cross(y,z)/np.linalg.norm(np.cross(y,z))
	m_DCM_OI = np.array([x,y,z])
	v_x_o = np.dot(m_DCM_OI,v_x_i)

	return v_x_o

def orbit2body(v_i, roll, pitch, yaw):
	# v_i the vector which need to be transformed from orbit to body frame
	# roll, pitch, yaw are euler angles

	m_tf = np.matrix([[np.cos(yaw)*np.cos(pitch), np.sin(yaw)*np.cos(pitch), -np.sin(pitch)],
					  [(np.cos(yaw)*np.sin(pitch)*np.sin(roll))-(np.sin(yaw)*np.cos(roll)), (np.sin(yaw)*np.sin(pitch)*np.sin(roll))+(np.cos(yaw)*np.cos(roll)), np.cos(pitch)*np.sin(roll)],
					  [(np.cos(yaw)*np.sin(pitch)*np.cos(roll))+(np.sin(yaw)*np.sin(roll)), (np.sin(yaw)*np.sin(pitch)*np.cos(roll))-(np.cos(yaw)*np.sin(roll)), np.cos(pitch)*np.cos(roll)]])
	v = np.dot(m_tf, v_i)
	v_b = np.array([v[0, 0], v[0, 1], v[0, 2]])

	return v_b