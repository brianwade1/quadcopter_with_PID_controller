#Quadcopter simulation

# import libraries
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import copy


def c(x):
    ''' cosine of angle in radians'''
    return np.cos(x)


def s(x):
    ''' sine of angle in radians'''
    return np.sin(x)


def t(x):
    ''' tangent of angle in radians'''
    return np.tan(x)

class Quadcopter():

    def __init__(self, pos, vel, angle, ang_vel, pos_ref, dt):

        mass =  0.506 # total mass of the vehicle, kg
        gravity = 9.8 # acceleration due to gravity, m/s^2
        num_motors = 4 # number of motors on the vehicle
        kt = 1e-7 # proportionality constant to convert motor rotational speed into thrust (T=kt*omega^2), N/(rpm)^2
      
        # Initial Sim variables
        self.pos = pos  # position of vehicle in inertial frame [x,y,z], meters
        self.vel = vel  #velocity of vehicle in inertial frame [x_dot, y_dot, z_dot], m/s
        self.angle = angle # orintation of vehicle inertial frame in radians [roll, pitch, yaw] -> [phi, theta, psi]
        self.ang_vel = ang_vel # angular velocity of inertial angles in rads/sec [phi_dot, theta_dot, psi_dot]
        self.lin_acc = np.array([0., 0., 0.]) # linear acceleration of vehicle in inertial frame [d^2(x)/dt, d^2(y)/dt, d^2(z)/dt], m/s^2
        self.ang_acc = np.array([0., 0., 0.]) #angular acceleration of vehicle in inertial frame [d^2(phi)/dt, d^2(theta)/dt, d^2(psi)/dt], rad/s^2

        # Desired reference states
        self.pos_ref = pos_ref #desired position [x, y, z] in inertial frame, m
        self.vel_ref = [0., 0., 0.]  #desired velocity [d(x)/dt, d(y)/dt, d(z)/dt] in inertial frame, m/s
        self.lin_acc_ref = [0., 0., 0.]  #desired acceleration [d^2(x)/dt, d^2(y)/dt, d^2(z)/dt] in inertial frame, m/s^2
        self.angle_ref = [0., 0., 0.] #desired angle [phi, theta, psi], radians
        self.ang_vel_ref = [0., 0., 0.,] #desired angular velocity [d(phi)/dt, d(theta)/dt, d(psi)/dt], radians/sec
        self.ang_acc_ref = [0., 0., 0.,] #desired angular acceleration [d^2(phi)/dt, d^2(theta)/dt, d^2(psi)/dt], radians/sec

        #Time measures
        self.time = 0
        self.dt = dt
       
        #Environment variables
        self.gravity = gravity # acceleration due to gravity, m/s^2
        self.density = 1.225 # air density, kg/m^3

        # Environment boundary
        self.bound_x = 10.
        self.bound_y = 10.
        self.bound_z = 10.

        # Vehicle constants
        self.num_motors = num_motors # number of motors on the vehicle
        self.mass = mass # total mass of the vehicle, kg
        self.Ixx = 8.11858e-5  # mass-moment of inertial about x-axis, kg-m^2
        self.Iyy = 8.11858e-5  # mass-moment of inertial about y-axis, kg-m^2
        self.Izz = 6.12233e-5 # mass-moment of inertial about z-axis, kg-m^2
        self.A_ref = 0.02 # reference area for drag calcs, m^2 
        self.L = 0.2 # length from body center to prop center, m
        self.kt = kt # proportionality constant to convert motor rotational speed into thrust (T=kt*omega^2), N/(rpm)^2
        self.b_prop = 1e-9 # proportionality constant to convert motor speed to torque (torque = b*omega^2), (N*m)/(rpm)^2
        self.Cd = 1 # drag coefficient
        self.thrust = mass * gravity
        self.speeds = np.ones(num_motors) * ((mass * gravity) / (kt * num_motors)) # initial speeds of motors
        self.tau = np.zeros(3)

        self.maxT = 16.5 #  max thrust from any single motor, N
        self.minT = .5 # min thrust from any single motor, N 
        self.max_angle = math.pi/12 #radians, max angle allowed at any time step

        self.I = np.array([[self.Ixx, 0, 0],[0, self.Iyy, 0],[0, 0, self.Izz]])
        self.g = np.array([0, 0, -gravity])

        self.done = False


    def calc_pos_error(self, pos):
        ''' Returns the error between actual position and reference position'''
        pos_error = self.pos_ref - pos
        return pos_error
        
    def calc_vel_error(self, vel):
        ''' Returns the error between actual velocity and reference velocity'''
        vel_error = self.vel_ref - vel
        return vel_error

    def calc_ang_error(self, angle):
        ''' Returns the error between actual angle and reference angle'''
        angle_error = self.angle_ref - angle
        return angle_error

    def calc_ang_vel_error(self, ang_vel):
        ''' Returns the error between angular velocity position and reference angular velocity'''
        ang_vel_error = self.ang_vel_ref - ang_vel
        return ang_vel_error


    def body2inertial_rotation(self):
        ''' 
        Euler rotations from body-frame to global inertial frame
        angle 0 = roll (x-axis, phi)
        angle 1 = pitch (y-axis, theta)
        angle 2 = yaw (z-axis, psi)
        '''

        c1 = c(self.angle[0]) 
        s1 = s(self.angle[0])
        c2 = c(self.angle[1])
        s2 = s(self.angle[1])
        c3 = c(self.angle[2])
        s3 = s(self.angle[2])

        R = np.array([[c2*c3, c3*s1*s2 - c1*s3, s1*s3 + c1*s2*c3],
            [c2*s3, c1*c3 + s1*s2*s3, c1*s3*s2 - c3*s1],
            [-s2, c2*s1, c1*c2]])
    
        return R


    def inertial2body_rotation(self):
        ''' 
        Euler rotations from inertial to body frame
            (Transpose of body-to-internal rotation)
        '''
        R = np.transpose(self.body2inertial_rotation())

        return R


    def thetadot2omega(self):
        '''rotate body angular velocity (Euler_dot) to inertial angular velocity (omega) '''

        R = np.array([[1, 0, -s(self.angle[1])],
            [0, c(self.angle[0]), c(self.angle[1])*s(self.angle[0])],
            [0, -s(self.angle[0]), c(self.angle[1])*c(self.angle[0])]])

        omega = np.matmul(R, self.ang_vel)
        return omega


    def omegadot2Edot(self,omega_dot):
        '''rotate inertial angular velocity (omega) to body angular velocity (Euler_dot) '''

        R = np.array([[1, s(self.angle[0])*t(self.angle[1]), c(self.angle[0])*t(self.angle[1])],
            [0, c(self.angle[0]), -s(self.angle[0])],
            [0, s(self.angle[0])/c(self.angle[1]), c(self.angle[0])/c(self.angle[1])]])

        E_dot = np.matmul(R, omega_dot)
        self.ang_acc = E_dot


    def find_omegadot(self, omega):
        ''' Find the angular acceleration in the inertial frame in rad/s '''
        omega = self.thetadot2omega() 
        #omega_dot = np.matmul(np.linalg.inv(self.I), (self.tau - np.cross(omega, np.matmul(self.I, omega))))
        omega_dot = np.linalg.inv(self.I).dot(self.tau - np.cross(omega, np.matmul(self.I, omega)))
        return omega_dot


    def find_lin_acc(self):
        ''' Find linear acceleration in m/s '''
        R_B2I = self.body2inertial_rotation()
        R_I2B = self.inertial2body_rotation()

        #body forces
        Thrust_body = np.array([0, 0, self.thrust])
        Thrust_inertial = np.matmul(R_B2I, Thrust_body) #convert to inertial frame

        vel_bodyframe = np.matmul(R_I2B, self.vel)
        drag_body = -self.Cd * 0.5 * self.density * self.A_ref * (vel_bodyframe)**2
        drag_inertial = np.matmul(R_B2I, drag_body)
        weight = self.mass * self.g

        acc_inertial = (Thrust_inertial + drag_inertial + weight) / self.mass
        self.lin_acc = acc_inertial


    def des2speeds(self,thrust_des, tau_des):
        ''' finds speeds of motors to achieve a desired thrust and torque '''

        # Needed torque on body
        e1 = tau_des[0] * self.Ixx
        e2 = tau_des[1] * self.Iyy
        e3 = tau_des[2] * self.Izz

        #less typing
        n = self.num_motors

        # Thrust desired converted into motor speeds
        weight_speed = thrust_des / (n*self.kt)

        # Thrust differene in each motor to achieve needed torque on body
        motor_speeds = []
        motor_speeds.append(weight_speed - (e2/((n/2)*self.kt*self.L)) - (e3/(n*self.b_prop)))
        motor_speeds.append(weight_speed - (e1/((n/2)*self.kt*self.L)) + (e3/(n*self.b_prop)))
        motor_speeds.append(weight_speed + (e2/((n/2)*self.kt*self.L)) - (e3/(n*self.b_prop)))
        motor_speeds.append(weight_speed + (e1/((n/2)*self.kt*self.L)) + (e3/(n*self.b_prop)))

        # Ensure that desired thrust is within overall min and max of all motors
        thrust_all = np.array(motor_speeds) * (self.kt)
        over_max = np.argwhere(thrust_all > self.maxT)
        under_min = np.argwhere(thrust_all < self.minT)

        if over_max.size != 0:
            for i in range(over_max.size):
                motor_speeds[over_max[i][0]] = self.maxT / (self.kt)
        if under_min.size != 0:
            for i in range(under_min.size):
                motor_speeds[under_min[i][0]] = self.minT / (self.kt)
        
        self.speeds = motor_speeds


    def find_body_torque(self):
        tau = np.array([(self.L * self.kt * (self.speeds[3] - self.speeds[1])),
          (self.L * self.kt * (self.speeds[2] - self.speeds[0])),
          (self.b_prop * (-self.speeds[0] + self.speeds[1] - self.speeds[2] + self.speeds[3]))])

        self.tau = tau


    def step(self):
        #Thrust of motors
        self.thrust = self.kt * np.sum(self.speeds)

        #Linear and angular accelerations in inertial frame
        self.find_lin_acc()

        #torque on body from motor thrust differential
        self.find_body_torque()

        #Angular acceleration in inertial frame
        omega = self.thetadot2omega()  #angles in inertial frame
        omega_dot = self.find_omegadot(omega) #angular acceleration in inertial frame

        #Angular acceleration in body frame
        self.omegadot2Edot(omega_dot)

        # Update states based on time step
        self.ang_vel += self.dt * self.ang_acc
        self.angle += self.dt * self.ang_vel
        self.vel += self.dt * self.lin_acc
        self.pos += self.dt * self.vel
        self.time += self.dt

