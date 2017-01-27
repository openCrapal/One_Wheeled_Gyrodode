#!/usr/bin/python3.4
# -*-coding:utf-8 -*

from threading import Thread
from constants import *
import automation as Z
import time


# PI inclinaison
Pa = 500.0     # 30.0
Ia = 0.        # 1000
Da = 0.0       # 0.00005
Fa = 150.      #
sata = 150.0   # 200

# PID orientation
Po = 2.        # 1.2
Io = 500.      # 400
Do = 0.000017  # 0.0006
Fo = 10.1      # 0.001
sato = 0.0

# PID position
Pw = 0.2       # 0.5
Iw = 0.2
Dw = 1.6
Fw = 100.0
satw = 0.0


class Robot(Thread):
    def __init__(self, i2cdevice, localisation):
        Thread.__init__(self)
        self.cycle_time = 0.004
        self.loc = localisation
        self.i2c = i2cdevice
        self.done = True

    def run(self):
        print("Beginn thread Robot")
        self.done = False
        
        Z.Z_Index = 0
        t_begin_program = time.time()

        self.loc.reset()
        self.i2c.reset()

        # The trajectories must be generated, time continuous. Constant zero is as continuous as it gets
        W_Goal = Z.Z_Constant(0.0)
        V_Goal = Z.Z_Derivative(W_Goal)

        # Rotation speed of the bot afak falling speed
        Gyro = Z.Z_Gain(Z.Z_Sensor(self.i2c.get_gyro_x), -1.0)
        D_Gyro = Z.Z_Derivative(Gyro)
        Estimated_Incl = Z.Z_Sensor(self.i2c.get_estimated_incl)

        # Speed of the point between the wheels: M
        V_M = Z.Z_Sensor(self.loc.get_speed)
        # Speed of the weightPoint: G
        V_G = Z.Z_Sum(Gyro, V_M, radius_G, 1.0)
        # A_G = Z.Z_Derivative(V_G)

        # Absolute position of the weightPoint (G) on it's trajectorie
        Way_G = Z.Z_Sum(Z.Z_Sensor(self.loc.get_way), Estimated_Incl, 1.0, radius_G)

        # Inclinaison you want to achieve, depending on speed and position
        # This is the very critical point of a balancing bot without absolut level sensor
        I_Goal = Z.Z_Gain(Z.Z_PID(Pw, Iw, Dw, Fw, satw, W_Goal, Way_G), -1.0)
        # I_Goal = Z.Z_BandStopFilter(Z.Z_Gain(Z.Z_PID(kvw, kpw, tpw, kdw, satw, W_Goal, Way_G, V_Goal, V_G), -1.0), 0.05, 0.0005)
        # D_I_Goal = Z.Z_Derivative(I_Goal)

        # All about orientation
        Orientation = Z.Z_Filter(Z.Z_Sensor(self.loc.get_teta), 0.1)
        D_Orientation = Z.Z_Filter(Z.Z_Derivative(Orientation), 0.5)
        Teta_Goal = Z.Z_Constant(0.0)

        Dir = Z.Z_Gain(Z.Z_PID(Po, Io, Do, Fo, sato * U_alim, Teta_Goal, Orientation), 1.0 / U_alim)

        Motor = Z.Z_Gain(Z.Z_PID(Pa, Ia, Da, Fa, sata * U_alim, I_Goal, Estimated_Incl),  1 / U_alim)
        # Motor = Z.Z_Sum(Z.Z_PID(kva, kpa, tpa, kda, sata*U_alim, I_Goal, Estimated_Incl, D_I_Goal, Gyro), D_Gyro, 1/U_alim, -kaa/U_alim)

        i = 0
        max = 0.0
        min = 10.0
        while not self.done:
            i = i + 1
            # W_Goal.set_val(0.05 * (1 - math.cos((time.time()-t_begin_program)*6.28*0.1)))
            # Teta_Goal.set_val((time.time()-t_begin_program)*0.4)
            t_begin_loop = time.time()
            # W_Goal.set_val(W_Goal.get_val() + 0.001)
            # Teta_Goal.set_val( Teta_Goal.get_val()+0.00)
            # print(I_Goal.get_val())
            # print(Gyro.get_val())
            # ampl = 0.1
            # phase = (time.time() - t_begin_program) * 6.281 * 1.0
            # I_Goal.set_val(ampl * math.sin(phase))


            d = Dir.get_val()
            m = Motor.get_val()
            self.i2c.set_speed(m - d, m + d)

            if i > 100:
                print("w: {0}\tv: {1}\tI_soll: {2}\tWgoal: {3} ".format(Way_G.get_val(), V_G.get_val(), I_Goal.get_val(),
                                                                        W_Goal.get_val()))
                i = 0
            Z.Z_Index += 1
            

            # print(time.time()-t_begin_program, "\t", Orientation.get_val())
            t2 = self.cycle_time + t_begin_loop - time.time()
            # print (Estimated_Incl.get_val())
            # print (t2)

            if t2 > max: max = t2
            if t2 < min: min = t2
            if t2 < 0:
                t2 = 0
            time.sleep(t2)
        print("min: ", min, " ; max: ", max)
        self.i2c.set_speed(0, 0)
        print("Exit thread Robot")
    
    def finish(self):
        self.done = True
        Thread.__init__(self)


# End Robot
