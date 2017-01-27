#!/usr/bin/python3.4
# -*-coding:utf-8 -*

# Common constants

U_alim = 12.2	# Voltage of the alim in V


# physical dimentions of the vehicule
dxLeft = 0.042*3.141/48.0
dxRight= 0.042*3.141/48.0		# Perimeter of the wheel / nbre of encoder tilt per rotation
wheels_width = 0.128   			# distance beetwin wheels
radius_G = 0.14					# Heigh of the weightPoint relative to the wheels axis

# motor constants
Kv_left = 30.9 * dxLeft			# Speed / U_motor  (m/s /V)
Kv_right= 30.9 * dxRight
KVr	= Kv_right* U_alim / 1200.0	# Speed / %pwm
KVl	= Kv_left * U_alim / 1200.0
R_intern = 17.1				# Calculated from stall torque, stall current und kv

# Pins Encoders
pinLeftA = 6
pinLeftB = 13
pinRightA= 19
pinRightB= 26


