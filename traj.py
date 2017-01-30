#!/usr/bin/python3
# -*-coding:utf-8 -*

import math
import time

debug = True

# Goal hier is to generate goal value that the controller can follow
# degree = n => the nth derivative as a finite maximal value
# 2 is usualy enought, 3 is state of the art for CNC-machines
# [X, X', X'', ...]


class Traj7Seg:
    def __init__(self, v_m=0.5, a_m=5, j_m=500):
        self.vm = v_m   # Absolut max Speed m/s
        self.am = a_m   # Absolut max Acceleeration m/s/s
        self.jm = j_m   # Absolut max Jerk m/s/s/s

        self.ampls = []
        self.vector = []
        for i in range(0, 7):
            self.ampls.append(0.0)
            self.vector.append([0.0, 0.0])  # Jerk, duration

        self.pos_init = [0.0, 0.0, 0.0]     # Position, Speed, Acceleration
        self.pos_current= [0.0, 0.0, 0.0]   # Position, Speed, Acceleration
        self.pos_goal = [0.0, 0.0, 0.0]  # Position, Speed, Acceleration

        self.up_to_date = True

    def tf(self, index=6):
        n = index
        tf = 0.0
        if n < 0:
            return tf
        if n > 6:
            n = 6
        for i in range(0, n+1):
            tf += self.vector[i][1]
        return tf

    def evaluate(self, t):
        if t < 0:
            return self.pos_init
        else:
            self.pos_current = [0.0, 0.0, 0.0]
            for i in range(0, len(self.ampls)):  # Summing up the influance of each phase of acceleration
                if t > self.tf(i-1):        # Dont evaluate the phases not yet begonen
                    if t <= self.tf(i):     # Current phase
                        self.pos_current[2] += self.vector[i][0] * (t-self.tf(i-1))
                        self.pos_current[1] += self.vector[i][0] * math.pow(t - self.tf(i-1), 2) / 2.0
                        self.pos_current[0] += self.vector[i][0] * math.pow(t - self.tf(i-1), 3) / 6.0
                    else:                   # Past phases
                        tf = self.tf(i)
                        a = self.vector[i][0] * (self.vector[i][1])
                        v = self.vector[i][0] * math.pow(self.vector[i][1], 2) / 2.0
                        x = self.vector[i][0] * math.pow(self.vector[i][1], 3) / 6.0
                        self.pos_current[2] += a
                        self.pos_current[1] += v + a * (t-tf)
                        self.pos_current[0] += x + v * (t-tf) + a * math.pow(t-tf, 2) / 2.0
                else:
                    break
            self.pos_current[2] += self.pos_init[2]
            self.pos_current[1] += self.pos_init[1] + t * self.pos_init[2]
            self.pos_current[0] += self.pos_init[0] + t * self.pos_init[1] + t*t * self.pos_init[2] / 2.0
            return self.pos_current

    def set_start_point(self, x, v, a):
        self.pos_init[0] = x
        self.pos_init[1] = v
        self.pos_init[2] = a
        self.up_to_date = False

    def set_goal_point(self, x, v, a):
        self.pos_goal[0] = x
        self.pos_goal[1] = v
        self.pos_goal[2] = a
        self.up_to_date = False

    def _fill_vector(self):
        for i in [0, 2, 4, 6]:
            self.vector[i][0] = math.copysign(self.jm, self.ampls[i])
            self.vector[i][1] = math.copysign(self.ampls[i], 1)
        for i in [1, 3, 5]:
            self.vector[i][0] = 0.0
            self.vector[i][1] = math.copysign(self.ampls[i], 1)

    def generate(self):
        if not self.up_to_date:
            # Initialise
            self.ampls = []
            self.vector = []
            for i in range(0, 7):
                self.ampls.append(0.0)
                self.vector.append([0.0, 0.0])  # Jerk, duration
            # Set corrections of acceleration
            self.ampls[0] = -1 * self.pos_init[2] / self.jm
            self.ampls[6] = self.pos_goal[2] / self.jm
            self._fill_vector()

            # Set corrections of speed, begin
            buffer_xVa = self.evaluate(self.tf(2))
            dv_begin = -1 * buffer_xVa[1]                  # Speed to build in phases 0 to 2
            buffer_xVa2 = self.evaluate(self.tf(6))
            dv_end = - self.pos_goal[1] - buffer_xVa[1] + buffer_xVa2[1]   # Speed to build in phases 4 to 6

            qv_min = -self.am / self.jm
            qv_max = self.am / self.jm

            qv0 = math.copysign(math.sqrt(math.copysign(dv_begin/self.jm, 1)), dv_begin)

            if qv0 < qv_min:    # -a_min at the end of phase 0
                self.ampls[0] += qv_min
                self.ampls[2] = - qv_min
                self.ampls[1] = math.copysign((dv_begin + self.jm * math.pow(qv_min, 2)) / self.am, 1)

            elif qv0 > qv_max:  # a_max
                self.ampls[0] += qv_max
                self.ampls[2] = - qv_max
                self.ampls[1] = math.copysign((dv_begin - self.jm * math.pow(qv_max, 2)) / self.am, 1)

            else:   # not saturated
                self.ampls[0] += qv0
                self.ampls[1] = 0
                self.ampls[2] = - qv0

            # correction of end speed
            qv6 = math.copysign(math.sqrt(math.copysign(dv_end / self.jm, 1)), dv_end)

            if qv6 < qv_min:  # a_max at the end of phase 5
                self.ampls[6] += qv_min
                self.ampls[4] = - qv_min
                self.ampls[5] = math.copysign((dv_end + self.jm * math.pow(qv_min, 2)) / self.am, 1)

            elif qv6 > qv_max:  # - a_max
                self.ampls[6] += qv_max
                self.ampls[4] = - qv_max
                self.ampls[5] = math.copysign((dv_end - self.jm * math.pow(qv_max, 2)) / self.am, 1)

            else:  # not saturated
                self.ampls[6] += qv6
                self.ampls[5] = 0
                self.ampls[4] = - qv6

            self._fill_vector()
            # Set correction of deplacement

            buffer_xVa = self.evaluate(self.tf())
            ds = self.pos_goal[0] - buffer_xVa[0]

            #qs =


# test du module
if __name__ == "__main__":
    myTraj = Traj7Seg(10, 5, 5)
    myTraj.set_start_point(0, -5, 0.1)
    myTraj.set_goal_point(0, 0, 0)
    myTraj.generate()
    print(myTraj.vector)
    #myTraj.generate()
    #print(myTraj.vector)
    #print(myTraj.ampls)
    for i in range(0,101):
        pos = myTraj.evaluate(i* myTraj.tf()/100)
        print(i* myTraj.tf()/100, pos[0], pos[1], pos[2])



