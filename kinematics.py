#!/usr/bin/python3
# -*-coding:utf-8 -*-

from time import time, sleep
import numpy as np
from scipy.optimize import newton_krylov
from numpy import cos, sin, sqrt, tan, arcsin, radians, pi

# Front direction is +y
# left is +x top is +z
# The origin belongs to the main corper of the bot

# parameters, metric system
servo_center = [0.04, 0.03, -0.02]
ball_joint_wheel = [0.05, 0.005, -0.072]
rod_length = 0.08
hub_length = 0.035

wheel_position = [0, 0, 0, 0] # a, b, c, Z
servos_position = [0, 0, 0, 0]

limits = [[-0.15, 0, 0.15],
          [-0.20, 0, 0.20],
          [-0.70, 0, 0.70],
          [-0.02, -0.015, +0.00]]


def rotation_matrix(axe='x', teta=0):
    c, s = cos(teta), sin(teta)

    if axe == 'x':
        return np.matrix([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    elif axe == 'y':
        return np.matrix([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    else:
        return np.matrix([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1]])


class Kin4Dof:
    def __init__(self, pos_servo, pos_joints, l_hub, l_rod):
        self.X = [0, 0, 0, 0]
        self.Q = [0, 0, 0, 0]

        self.hub_length = l_hub
        self.rod_length = l_rod
        self.servos = []
        self.joints = []
        self.hub_servos = []
        self.repx = [1, 1, -1, -1]
        self.repy = [1, -1, -1, 1]
        for i in range(0, 4):
            self.servos.append([pos_servo[0]*self.repx[i], pos_servo[1]*self.repy[i], pos_servo[2]])
            self.joints.append([pos_joints[0]*self.repx[i], pos_joints[1]*self.repy[i], pos_joints[2]])
            self.hub_servos.append([0, self.repy[i] * self.hub_length, 0])  # Initial position of hubs

        #print(self.servos)
        #print(self.joints)
        #print(self.hub_servos)

    def direct_kin(self, servos_angular_postion):
        self.Q = servos_angular_postion
        l_rod_sq = self.rod_length**2
        servos_end = []
        joints = self.joints

        for i in range(0, 4):
            l = np.array(self.hub_servos[i])
            r = rotation_matrix('x', servos_angular_postion[i]*self.repy[i])
            servos_end.append(np.matmul(r, l))

            for j in range(0, 3):
                servos_end[i][0, j] += self.servos[i][j]

            print(servos_end[i])

        def ecart(x):
            ecarts = [0.0, 0.0, 0.0, 0.0]
            for i in range(0, 4):
                # translation de x[3] suivant z
                tr_joints = np.array(joints[i])
                tr_joints[2] += x[3]
                # rotations
                pos = np.matmul(rotation_matrix('z', x[2]).dot(rotation_matrix('y', x[1])).dot(rotation_matrix('x', x[0])), tr_joints)
                #print(pos)
                # Difference between the actual lengths of the rods and the estimated ones (squared)
                ecarts[i] = (pos[0, 0] - servos_end[i][0, 0]) ** 2 + (pos[0, 1] - servos_end[i][0, 1])**2 + (pos[0, 2] - servos_end[i][0, 2])**2
                ecarts[i] -= l_rod_sq
            #print(ecarts)
            return ecarts

        # uses the previous position as initial gess. Great when slowly mooving
        guess = self.X
        #print(ecart([0, 0, 0, 0]))
        self.X = newton_krylov(ecart, guess, method='lgmres', verbose=0, f_tol=0.00001)
        return

    def reverse_kin(self, pos):
        mat_rotation = rotation_matrix('z', pos[2]).dot(rotation_matrix('y', pos[1])).dot(rotation_matrix('x', pos[0]))
        sing_detection = False
        # The analitic way. Newton_krylow optimation did work but was unstable.
        # https://fr.wikipedia.org/wiki/Système_bielle-manivelle
        for i in range(0, 4):
            joints = np.array(self.joints[i])
            # Translation
            joints[2] += pos[3]
            # Rotation
            joints = mat_rotation.dot(joints)

            # Projecting the rod in the plan of the hub x=x_hub
            l_rod_proj = sqrt(self.rod_length**2 - (self.servos[i][0]-joints[0, 0])**2)

            # Projecting the distance beetween the balljoints on the [joint_center; servo_center] +pi/2 axe
            teta_proj = tan((joints[0, 1]-self.servos[i][1])/(joints[0, 2]-self.servos[i][2]))
            distance_proj = sqrt((joints[0, 2]-self.servos[i][2]) ** 2 + (joints[0, 1] - self.servos[i][1]) ** 2)
            ratio = (self.hub_length**2 + distance_proj**2 - l_rod_proj**2) / (2.0*hub_length*distance_proj)
            if abs(ratio) > 0.95:  # close to a singularity
                sing_detection = True
            else:
                teta_prim = arcsin(ratio)
                self.Q[i] = teta_prim + self.repy[i] * teta_proj
                self.X = pos
                sing_detection = False
        return sing_detection

    def search_z0(self):
        # First step, with all servo's hubs horizontal
        servo_end = []
        for i in range(0, 3):
            servo_end.append(self.servos[0][i]+self.hub_servos[0][i])

        dx = servo_end[0] - self.joints[0][0]
        dy = servo_end[1] - self.joints[0][1]
        dz = sqrt(self.rod_length ** 2 - dx ** 2 - dy ** 2)

        print("Neutral value for z_joint: ", servo_end[2] - dz)
        for k in range(0, 4):
            self.joints[k][2] = servo_end[2] - dz
        return self.joints[0][2]

    def search_borders(self):
        borders = [[]]
        for i in limits[0]:
            for j in limits[1]:
                for k in limits[2]:
                    for l in limits[3]:
                        param = [i, j, k, l]
                        if self.reverse_kin(param):
                            print(param)
                            borders[0].append(param)

    def jacobian(self, x):  #dQ/dX
        jac = np.zeros([4, 4])
        dx = 0.00005
        for i in range(0, 4):
            x0 = x
            x0[i] += dx
            self.reverse_kin(x0)
            qm = self.Q.copy()
            x0 = x
            x0[i] -= dx
            self.reverse_kin(x0)
            for j in range(0, 4):
                jac[i, j] = (self.Q[j] - qm[j]) / (2.0*dx)
        return jac


class Gyropode:
    def __init__(self, kin4dof,  delta_z_wheel, radius_wheel, radius_tire=0.02, z_middle_point=0.0):
        self.Kin = kin4dof
        self.m_point = np.array([0.0, 0.0, z_middle_point])
        self.z_w = delta_z_wheel + self.Kin.joints[0][2]
        self.axe_wheel = np.array([0.0, 0.0, self.z_w])
        self.r1 = radius_wheel
        self.r2 = radius_tire

        self.heading = [0.0, 0.0, 0.0]  # [tilt, roll , heading] Absolute

    def reverse_kin(self, contact_point):
        x_roue = [0,0,0,0]
        x_roue[0] = arcsin(contact_point[0] / (self.z_w + self.r1))
        x_roue[1] = arcsin(contact_point[1] / (self.z_w + self.r1))
        x_roue[2] = contact_point[3]
        x_roue[3] = self.z_w
        sing = self.Kin.reverse_kin(x_roue)
        return sing


if __name__ == "__main__":
    t0 = time()
    myKin = Kin4Dof(servo_center, ball_joint_wheel, hub_length, rod_length)
    myKin.search_z0()
    #myKin.search_borders()
    #print("zmax \n", myKin.jacobian([0,0,0,limits[3][0]]))
    #print("zmoy \n", myKin.jacobian([0,0,0,limits[3][1]]))
    #print("zmin \n", myKin.jacobian([0,0,0,limits[3][2]]))

    #myKin.reverse_kin([0, 0, 0, 0])

    #myGyro = Gyropode(myKin, 0.1, 0.05, 0.02)
    for i in range(0, 200):
        s = 0.01 * sin(i * pi/100)
        c = 0.015 * cos(i * pi / 100)
        myKin.reverse_kin([s, c, 0, -0.01])
        print([s, c, 0, -0.01], myKin.Q)
    print("time: ", time()-t0)


