#!/usr/bin/python3
# -*-coding:utf-8 -*-

#from __future__ import print_function, division
from sympy import symbols, simplify, trigsimp
from sympy.physics.mechanics import dynamicsymbols, ReferenceFrame, Point, inertia, RigidBody, KanesMethod



r_wheel = symbols('r')
l_fourche = symbols('l')

OG1, OG2, OG3 = symbols('OG1, OG2, OG3')

m1 = 0.5
m2 = 0.1
m3 = 0.1

g = -9.81

ground = ReferenceFrame('0')
body = ReferenceFrame('1')
leg = ReferenceFrame('2')
wheel = ReferenceFrame('3')
ref_contact = ReferenceFrame('ref_vi')

j1 = inertia(body, 1.0, 1.0, 1.0)
j2 = inertia(body, 0.2, 0.2, 0.2)
j3 = inertia(body, 0.3, 0.1, 0.1)

tz21 = dynamicsymbols('Z2')  # Set with the servos
Euler10 = dynamicsymbols('A1, B1, C1')  # Mesured
Euler21 = dynamicsymbols('A2, B2, C2')  # Set with the servos
Euler32 = [dynamicsymbols('theta'), 0, 0]  # Set with the motor

w_wheel = dynamicsymbols('w_wheel')
w_10 = dynamicsymbols("A1', B1', C1'")
w_21 = dynamicsymbols("A2', B2', C2'")

OG1, OG2, OG3 = symbols('OG1, OG2, OG3')

#sensor_quaternion = [0, 0, 0, 0]
#body.orient(ground, 'Quaternion', sensor_quaternion)

body.orient(ground, 'Body', Euler10, 'XYZ')
leg.orient(body, 'Body', Euler21, 'XYZ')
ref_contact.orient(ground, 'Body', [0, 0, Euler10[2]], 'XYZ')
#print(leg.dcm(ground))
body.set_ang_vel(ground, w_10[0]*body.x + w_10[1]*body.y +  w_10[2]*body.z)
leg.set_ang_vel(body, w_21[0]*body.x + w_21[1]*body.y +  w_21[2]*body.z)
wheel.set_ang_vel(leg, w_wheel * leg.x)

contact = Point('I')
ball_joint = Point('O')
g1 = Point('G1')
g2 = Point('G2')
g3 = Point('G3')

ball_joint.set_pos(contact, (r_wheel + l_fourche + tz21) * leg.z)
g1.set_pos(ball_joint, OG1 * body.z)
g2.set_pos(ball_joint, (OG2+tz21) * leg.z)
g3.set_pos(ball_joint, (OG3+tz21) * leg.z)

contact.set_vel(ground, r_wheel*(w_wheel+w_10[0]+w_21[0])*ref_contact.y)

g3.v2pt_theory(contact, ground, leg)
g2.v2pt_theory(contact, ground, leg)
g1.v2pt_theory(ball_joint, ground, body)


kinematic_differential_equations = [w_wheel - Euler32[0].diff()]
for i in range(0, 3):
    kinematic_differential_equations.append(w_10[i] - Euler10[i].diff())
    kinematic_differential_equations.append(w_21[i] - Euler21[i].diff())

# Rigid Bodys
Body = RigidBody('Body', g1, body, m1, j1)
Leg = RigidBody('Leg', g2, leg, m2, j2)
Wheel = RigidBody('Wheel', g3, wheel, m3, j3)

# Forces
grav1 = (g1, g * m1 * ground.z)
grav2 = (g2, g * m2 * ground.z)
grav3 = (g3, g * m3 * ground.z)
# perturbation...

# Torques : Friction of the wheel on the ground generate a torque in direction z
torque03 = (contact, -0.1 * wheel.dcm(ground).diff().dot(ground.z))

# Equations of motion

print(kinematic_differential_equations)



