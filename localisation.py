#!/usr/bin/python3.4
# -*-coding:utf-8 -*

from constants import *
import RPi.GPIO as GPIO
import math
from time import time, sleep
from threading import Thread
from automation import Z_Filter

# GPIO.cleanup()
GPIO.setmode(GPIO.BCM)  # use hardware pin numbers
GPIO.setwarnings(True)


class Encoder():
    # This class could be use by itself, in case you have only one quadrature encoder
    # The point is to minimise calculation wihtin the interruptions in order not to miss any

    def changeA(self, term):
        self.A = GPIO.input(self.pinA)
        if self.A:  # risinging edge on A
            if self.B:
                self.count -= 1
            else:
                self.count += 1
        else:  # fallinging edge on A
            if self.B:
                self.count += 1
            else:
                self.count -= 1

    def changeB(self, term):
        self.B = GPIO.input(self.pinB)
        if self.B:  # risinging edge on B
            if self.A:
                self.count += 1
            else:
                self.count -= 1
        else:  # falling edge on B
            if self.A:
                self.count -= 1
            else:
                self.count += 1

    def __init__(self, pinA, pinB):
        self.pinA = pinA
        self.pinB = pinB
        self.count = 0
        self.erreurs = 0
        GPIO.setup(pinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # pull_up_down=GPIO.PUD_UP
        GPIO.setup(pinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.A = GPIO.input(pinA)
        self.B = GPIO.input(pinB)
        done = False
        # Setting the interuptions. Sometimes it fails, so try again
        while not done:
            try:
                GPIO.add_event_detect(pinA, GPIO.BOTH, callback=self.changeA)
            except:
                pass
            else:
                done = True
        done = False
        while not done:
            try:
                GPIO.add_event_detect(pinB, GPIO.BOTH, callback=self.changeB)
            except:
                pass
            else:
                done = True
        self.A = GPIO.input(pinA)
        self.B = GPIO.input(pinB)

    # this is the getter you wanna use for sensor displacement
    def get_count(self):
        c = self.count
        self.count = 0
        return c


# Create an item of the loc class to get track of you position
# The run() method must be called frequently
class loc(Thread):
    def reset(self):
        self.left_speed = self.right_speed = self.pos_speed = 0.0
        self.pos_x = self.pos_y = self.pos_teta = self.pos_way = 0.0

    def __init__(self):
        Thread.__init__(self)
        self.cycle_time = 0.004
        self.t_filter = 0.05
        self.reset()
        self.NotDone = False

    def run(self):
        self.NotDone = True
        Enc_Right = Encoder(pinRightA, pinRightB)
        Enc_Left = Encoder(pinLeftA, pinLeftB)
        count_l = count_r = 0
        DxLeft = dxLeft
        DxRight = dxRight
        DtetaLeft = math.atan(dxLeft / (2.0 * wheels_width))
        DtetaRight = math.atan(dxRight / (2.0 * wheels_width))
        t_old = time()
        tmin = 10
        tmax = 0
        print("begin thread localisation")
        while (self.NotDone):
            count_l = - Enc_Left.get_count()
            count_r = Enc_Right.get_count()
            # print(self.count_l, ";\t", self.count_r)
            t = time()
            # self.r += self.count_r
            # self.l += self.count_l
            dt = t - t_old
            ls = DxLeft * count_l / dt
            rs = DxRight * count_r / dt

            self.left_speed = (self.left_speed * self.t_filter + dt * ls) / (dt + self.t_filter)
            self.right_speed = (self.right_speed * self.t_filter + dt * rs) / (dt + self.t_filter)

            t_old = t

            if (count_l != 0 or count_r != 0):
                d_way = (count_l * DxLeft + count_r * DxRight) / 2.0  # absolut deplacement step
                self.pos_way += d_way  # "Abcisse curviligne" position on the roboter's trajectorie
                half_d_teta = - count_l * DtetaLeft + count_r * DtetaRight  # half rotation step
                local_teta = self.pos_teta + half_d_teta  # local variable to store absolut orientation
                self.pos_teta = local_teta + half_d_teta
                self.pos_x += d_way * math.cos(local_teta)  # Note that the position is updated this only half of
                self.pos_y += d_way * math.sin(local_teta)  # ...the rotation step

            # self.pos_errors = self.Enc_Right.erreur + self.Enc_Left.erreur

            t_sleep = self.cycle_time - time() + t
        # if t_sleep > tmax: tmax=t_sleep
        # elif t_sleep<tmin: tmin=t_sleep
        # if t_sleep < 0:
        #	print ("Tread location : cycle time too short")
        #	t_sleep = 0.001
        # sleep(t_sleep)
        # End_While
        print("Exit thread localisation")

    # print("loc t_cycle", self.cycle_time, "\tmin ", tmin, "\tmax ", tmax)
    # End_run


    def get_teta(self):
        return (self.pos_teta)

    def get_speed(self):
        return ((self.right_speed + self.left_speed) / 2.0)

    def get_left_speed(self):
        return (self.left_speed)

    def get_right_speed(self):
        return (self.right_speed)

    def get_x(self):
        return (self.pos_x)

    def get_y(self):
        return (self.pos_y)

    def get_way(self):
        return (self.pos_way)

    def finish(self):
        self.__init__()


# End class loc



if __name__ == "__main__":
    from i2cDev import i2c_devices

    i2cDevs = i2c_devices()
    i2cDevs.start()
    i2cDevs.set_speed(20, 20)
    MyLoc = loc()
    MyLoc.start()

    GPIO.setup(pinLeftA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(pinLeftB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(pinRightA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(pinRightB, GPIO.IN, pull_up_down=GPIO.PUD_UP)


    def test_kv(Motors):
        def test_value(value_pwm):
            i2cDevs.set_speed(value_pwm, value_pwm)
            sleep(0.01)
            t = time()
            sum_l = sum_r = 0.0
            n = 0
            while (time() - t < 0.001):
                n += 1
                sum_l += MyLoc.get_left_speed()
                sum_r += MyLoc.get_right_speed()
                sleep(0.001)
            sum_r /= n
            sum_l /= n
            print(value_pwm, ";\t", sum_l, ",\t", sum_r, ";\t", MyLoc.get_speed())

        for i in range(0, 90, 1):
            test_value(i)
        for i in range(90, -90, -1):
            test_value(i)
        for i in range(-90, 0, 1):
            test_value(i)


    mla = mra = mlb = mrb = 0.0
    x = 0
    print("pos_x \t pos_Y \t pos_teta \t pos_speed")
    t = timer = time()
    n = 0
    while (t - timer < 60):
        n = n + 1
        t = time()
        i2cDevs.set_speed(40, 40)
        # print (MyLoc.get_x(),"\t", MyLoc.get_y(),"\t", MyLoc.get_teta(),"\t", MyLoc.get_speed())
        mla = 0.99 * mla + 0.01 * GPIO.input(pinLeftA)
        mra = 0.99 * mra + 0.01 * GPIO.input(pinRightA)
        mlb = 0.99 * mlb + 0.01 * GPIO.input(pinLeftB)
        mrb = 0.99 * mrb + 0.01 * GPIO.input(pinRightB)
        if n > 5:
            print(mla, "\t", mlb, "\t", mra, "\t", mrb)
            n = 0
        #		print(GPIO.input(pinLeftA),GPIO.input(pinLeftB),GPIO.input(pinRightA),GPIO.input(pinRightB))
        sleep(0.001)
        x += 1

    #	test_kv(pwmMotors)

    while math.copysign(x, 1) > 10:
        i2cDevs.set_speed(0, x)
        x -= math.copysign(1, x)
        sleep(0.01)
    i2cDevs.set_speed(0, 0)
    i2cDevs.finish()
    MyLoc.finish()
