#!/usr/bin/python3.4
# -*-coding:utf-8 -*

# powers my two motors throught Adafruit i2c servo driver

import math
from time import time, sleep
import re
import smbus
from constants import *
from threading import Thread

# Channels
DIRg = 12	
PWMg = 13
DIRr = 14
PWMr = 15

# ===========================================================================
# Adafruit_I2C Class
# ===========================================================================

class Adafruit_I2C(object):

	def __init__(self, address, busnum=-1, debug=False):
		self.address = address
		self.bus = smbus.SMBus(1); # Force I2C1 (512MB Pi's)
		
	def reverseByteOrder(self, data):
		#Reverses the byte order of an int (16-bit) or long (32-bit) value
		# Courtesy Vishal Sapre
		byteCount = len(hex(data)[2:].replace('L','')[::2])
		val			 = 0
		for i in range(byteCount):
			val		= (val << 8) | (data & 0xff)
			data >>= 8
		return val

	def write8(self, reg, value):
		try:
			self.bus.write_byte_data(self.address, reg, value)
		except:
			pass

	def write16(self, reg, value):
		try:
			self.bus.write_word_data(self.address, reg, value)
		except:
			pass

	def writeRaw8(self, value):
		try:
			self.bus.write_byte(self.address, value)
		except:
			pass

	def readU8(self, reg):
	# Read unsigned Byte
		try:
			result = self.bus.read_byte_data(self.address, reg)
			return result
		except:
			pass
	
	def readS8(self, reg):
	# Read signed Byte
		try:
			result = self.bus.read_byte_data(self.address, reg)
			if result > 127: result -= 256
			return result
		except:
			pass

	def readU16(self, reg, little_endian=True):
	#Reads an unsigned 16-bit value from the I2C device
		try:
			result = self.bus.read_word_data(self.address,reg)
			# Swap bytes if using big endian because read_word_data assumes little 
			# endian on ARM (little endian) systems.
			if not little_endian:
				result = ((result << 8) & 0xFF00) + (result >> 8)
			return result
		except:
			pass

	def readS16(self, reg, little_endian=True):
	#Reads a signed 16-bit value from the I2C device"
		try:
			result = self.readU16(reg,little_endian)
			if result > 32767: result -= 65536
			return result
		except:
			pass


# ============================================================================
# Adafruit PCA9685 16-Channel PWM Servo Driver
# ============================================================================

class PWM() :
	# Registers/etc.
	__MODE1				= 0x00
	__MODE2				= 0x01
	__SUBADR1			= 0x02
	__SUBADR2			= 0x03
	__SUBADR3			= 0x04
	__PRESCALE			= 0xFE
	__LED0_ON_L			= 0x06
	__LED0_ON_H			= 0x07
	__LED0_OFF_L		= 0x08
	__LED0_OFF_H		= 0x09
	__ALL_LED_ON_L		= 0xFA
	__ALL_LED_ON_H		= 0xFB
	__ALL_LED_OFF_L		= 0xFC
	__ALL_LED_OFF_H		= 0xFD

	# Bits
	__RESTART			= 0x80
	__SLEEP				= 0x10
	__ALLCALL			= 0x01
	__INVRT				= 0x10
	__OUTDRV			= 0x04

	general_call_i2c = Adafruit_I2C(0x00)

	@classmethod
	def softwareReset(cls):
		cls.general_call_i2c.writeRaw8(0x06)		# SWRST

	def __init__(self, address=0x40):
		self.i2c = Adafruit_I2C(address)
		self.address = address
		self.setAllPWM(0, 0)
		self.i2c.write8(self.__MODE2, self.__OUTDRV)
		self.i2c.write8(self.__MODE1, self.__ALLCALL)
		sleep(0.005)					 # wait for oscillator
		mode1 = self.i2c.readU8(self.__MODE1)
		mode1 = mode1 & ~self.__SLEEP			 # wake up (reset sleep)
		self.i2c.write8(self.__MODE1, mode1)
		sleep(0.005)					 # wait for oscillator
		self.dirG_old = self.dirR_old = self.vg_old = self.vr_old = 0

	def setPWMFreq(self, freq):
		#Sets the PWM frequency
		prescaleval = 25000000.0		# 25MHz
		prescaleval /= 4096.0			 # 12-bit
		prescaleval /= float(freq)
		prescaleval -= 1.0
		prescale = math.floor(prescaleval + 0.5)
		oldmode = self.i2c.readU8(self.__MODE1);
		newmode = (oldmode & 0x7F) | 0x10	 # sleep
		self.i2c.write8(self.__MODE1, newmode)				# go to sleep
		self.i2c.write8(self.__PRESCALE, int(math.floor(prescale)))
		self.i2c.write8(self.__MODE1, oldmode)
		sleep(0.005)
		self.i2c.write8(self.__MODE1, oldmode | 0x80)

	def setPWM(self, channel, on, off):
		#Sets a single PWM channel
		self.i2c.write8(self.__LED0_ON_L+4*channel, on & 0xFF)
		self.i2c.write8(self.__LED0_ON_H+4*channel, on >> 8)
		self.i2c.write8(self.__LED0_OFF_L+4*channel, off & 0xFF)
		self.i2c.write8(self.__LED0_OFF_H+4*channel, off >> 8)
	def setAllPWM(self, on, off):
		#Sets a all PWM channels
		self.i2c.write8(self.__ALL_LED_ON_L, on & 0xFF)
		self.i2c.write8(self.__ALL_LED_ON_H, on >> 8)
		self.i2c.write8(self.__ALL_LED_OFF_L, off & 0xFF)
		self.i2c.write8(self.__ALL_LED_OFF_H, off >> 8)	

	def set_speed(self, v1, v2):
		dirG = 0
		if v1>=0:
			if v1>=100:
				v1=100
			dirG=0
		elif v1<0:
			if v1<= -100:
				v1=-100
			v1 = -v1
			dirG=4095

		dirR = 0
		if v2>=0:
			if v2>=100:
				v2=100
			dirR=0
		elif v2<0:
			if v2<= -100:
				v2=-100
			v2 = -v2
			dirR=4095

		vg = math.trunc(4095*v1/100.0)
		vr = math.trunc(4095*v2/100.0)

		if (dirG != self.dirG_old):
			self.setPWM(DIRg, 0, dirG)
		if (dirR != self.dirR_old):
			self.setPWM(DIRr, 0, dirR)
		if (vg != self.vg_old):
			self.setPWM(PWMg, 0, vg)
		if (vr != self.vr_old):
			self.setPWM(PWMr, 0, vr)
		self.dirG_old = dirG
		self.dirR_old = dirR
		self.vr_old = vr
		self.vg_old = vg

class mpu6050():
	#register adress
	power_mgmt_1 = 0x6b
	power_mgmt_2 = 0x6c
	set_scale = 0x1b
	sample_rate = 0x19
	low_pass_filter = 0x1a
	offset_x = 0
	
	def get_x(self):
		try:
			v = self.i2c.readS16(0x43, False)
			return ( v - self.offset_x ) * self.scale
		except:
			return(0)
			pass

	def save_offset(self, n=500):
		self.offset_x = 0
		for i in range(1, n, 1):
			self.offset_x += self.i2c.readS16(0x43, False)
			sleep(0.0005)
		self.offset_x /= n
		print("offset_x: ", self.offset_x)

	def __init__(self, address=0x68):
		try:
			self.i2c = Adafruit_I2C(address)
			self.address = address
		except:
			print("I2C bus not responding")
		# Wake up mpu6050
		try:
			self.i2c.write8(self.power_mgmt_1, 0)
		except:
			print("Connexion mpu6050 failed, trying again...")
			sleep(0.01)
			self.i2c.write8(self.power_mgmt_1, 0)		
			pass

		self.i2c.write8(self.set_scale, 2) 		# 2 => +/- 1000 °/sec
		self.i2c.write8(self.sample_rate, 7)		# 7=>1kHz; 6=>2kHz? rate
		self.i2c.write8(self.low_pass_filter, 0x0)	# 6=>5Hz ; 0=>256Hz  bandpass
		
		self.scale = 1000.0 * 3.141 / ( 8 * 32768 * 180 ) # rad/sec
		self.offset_x = self.offset_y = self.offset_z = 0 
		sleep(0.005)

class i2c_devices(Thread):
	def reset(self):
		self.v1 = self.v2 = 0
		self.gyro_x = 0.0
		self.estimated_incl = 0.0
		self.new_val = True
		
	def __init__(self):
		Thread.__init__(self)
		self.motors = PWM()
		self.gyro = mpu6050()
		self.t_filter = 0.005
		self.reset()
		self.dt_mot = time()

	def save_gyro_offset(self, n=200):
		self.gyro.save_offset(500)
		
	def set_speed(self, v1, v2):
		self.v1 = v1
		self.v2 = v2
		self.new_val = True
		self.dt_mot = time()

	def get_gyro_x(self):
		return self.gyro_x

	def get_estimated_incl(self):
		return self.estimated_incl

	def finish(self):
		self.motors.set_speed(0,0)
		#self.motors.softwareReset()
		Thread.__init__(self)
		self.notDone = False
		self.v1 = self.v2 = 0
		self.gyro_x = 0.0
		self.estimated_incl = 0.0
		#self.__init__()
		sleep(0.05)

	def run(self):
		tmin = 10
		tmax = 0
		mmin = 10
		mmax = 0
		self.notDone = True
		print("begin thread i2c")
		while self.notDone :
			t = t_old = time()
			while (self.notDone and not self.new_val):
				sleep(0.0001)
				r = self.gyro.get_x()
				t = time()
				dt = t - t_old
				self.estimated_incl += r * dt
				self.gyro_x = (self.gyro_x * self.t_filter + dt * r) / (dt + self.t_filter)
				t_old = t
				#if dt > tmax: tmax = dt
				#elif dt<tmin: tmin = dt
				if (not self.notDone) or self.new_val: break
				sleep(0.0001)
								
			self.motors.set_speed(self.v1, self.v2)
			self.new_val = False
			#dtMot = time()-self.dt_mot
			#if dtMot > mmax: mmax = dtMot
			#elif dtMot<mmin: mmin = dtMot
		#print("I2cDevice loop time min ", tmin, "\max ", tmax)
		#print("I2cMots react time min ", mmin, "\max ", mmax)
		print("exit thread i2c")
		#END_LOOP; exit thread

if __name__ == "__main__":
	myI2cDev = i2c_devices()
	myI2cDev.save_gyro_offset(500)
	myI2cDev.start()
	for i in range(0, 90, 1):
		myI2cDev.set_speed(i, i)
		sleep(0.1)
		print(myI2cDev.get_gyro_x(), myI2cDev.get_estimated_incl())
	for i in range(90, 0, -1):
		myI2cDev.set_speed(i,i)
		sleep(0.1)
		print(myI2cDev.get_gyro_x(), myI2cDev.get_estimated_incl())
	myI2cDev.finish()

