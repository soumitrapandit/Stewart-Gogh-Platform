import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Hexapod:
    def __init__(self, upper_rad=300/2, lower_rad=650/2, upper_lim=1100, lower_lim=100, alpha=40, beta=80, euler_order='xyz'):
        self.alpha = alpha
        self.beta = beta
        self.upper_rad = upper_rad
        self.lower_rad = lower_rad
        self.upper_lim = upper_lim
        self.lower_lim = lower_lim
        self.euler_order = euler_order
        self.create_base()
        self.create_body()
        self.create_legs()

    def ik(self, pose):
        upper_lim = self.upper_lim
        lower_lim = self.lower_lim
        self.transform_body(pose)
        for leg_length in self.leg_lengths:
            assert lower_lim < leg_length < upper_lim
        return self.leg_vectors

    def fk(self, leg_lengths, error, guess_pose):
        self.transform_body(guess_pose)
        delta_l = np.array(leg_lengths) - self.leg_lengths
        if (error >= delta_l).all():
            return guess_pose
        else:
            return self.fk(error=error, guess_pose=guess_pose + self.JT_inv @ delta_l)

    def show_robot(self):
        self.calc_Rsi()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        base_radius = self.lower_rad
        nominal_leg_length = 100
        ax.set_xlim(-base_radius, base_radius)
        ax.set_ylim(-base_radius, base_radius)
        ax.set_zlim(0, nominal_leg_length * 2)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.scatter(*self.base_center, marker='.', c='red', s=100)
        ax.scatter(*self.body_center, marker='.', c='k', s=50)
        for i in range(6):
            ax.plot(*zip(self.base_points[i], self.body_points[i]), marker='.', color='red')
        plt.show()

    def create_base(self):
        self.base_center = np.array([0, 0, 0])
        self.base_points = [np.array([np.cos(theta) * self.lower_rad, np.sin(theta) * self.lower_rad, 0])
                            for theta in np.deg2rad([-self.alpha/2 + i * (120 if i % 2 else self.alpha) for i in range(6)])]

    def calc_Rsi(self):
        self.R_si = [bp - self.body_center for bp in self.body_points]

    def create_body(self):
        self.body_center = np.array([0, 0, 0])
        self.body_points = [np.array([np.cos(theta) * self.upper_rad, np.sin(theta) * self.upper_rad, 0])
                            for theta in np.deg2rad([-self.beta/2 + i * (120 if i % 2 else self.beta) for i in range(6)])]

    def create_legs(self):
        self.legs = [(self.base_points[i], self.body_points[i]) for i in range(6)]
        self.leg_vectors = [bp - self.base_points[i] for i, bp in enumerate(self.body_points)]
        self.ni = [vec / np.linalg.norm(vec) for vec in self.leg_vectors]
        self.leg_lengths = [np.linalg.norm(vec) for vec in self.leg_vectors]

    def euler_to_R(self, theta_list):
        self.R_mat = np.identity(3)
        for index, theta in enumerate(theta_list):
            rotation = getattr(self, f'rotation_{self.euler_order[index]}')(np.deg2rad(theta))
            self.R_mat = self.R_mat @ rotation

    def rotation_x(self, roll):
        return np.array([[1, 0, 0], [0, np.cos(roll), np.sin(roll)], [0, -np.sin(roll), np.cos(roll)]])

    def rotation_y(self, pitch):
        return np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])

    def rotation_z(self, yaw):
        return np.array([[np.cos(yaw), np.sin(yaw), 0], [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    def T_mat(self):
        pose = self.pose
        self.euler_to_R(pose[3:])
        p = pose[:3]
        self.T_matrix = np.vstack((np.column_stack((self.R_mat, p)), [0, 0, 0, 1]))

    def transform_body(self, pose):
        self.create_body()
        self.pose = pose
        self.T_mat()
        self.body_center = self.T_matrix[:3, 3]
        self.body_points = [np.dot(self.T_matrix, np.append(point, 1))[:3] for point in self.body_points]
        self.calc_Rsi()
        self.create_legs()
        self.Jacobian()
        self.JT_mat()

    def B_matrix(self):
        a, b = self.pose[3:5]
        self.B_mat = np.array([[1, 0, np.sin(b)], [0, np.cos(a), -np.sin(a) * np.cos(b)], [0, np.sin(a), np.cos(a) * np.cos(b)]])

    def Jacobian(self):
        self.J = np.array([np.hstack((ni, np.cross(R, ni))) for R, ni in zip(self.R_si, self.ni)])

    def JT_mat(self):
        self.B_matrix()
        T_alpha = np.vstack((np.hstack((np.identity(3), np.zeros((3, 3)))), np.hstack((np.zeros((3, 3)), self.B_mat))))
        self.JT_matrix = self.J @ T_alpha
        self.JT_inv = np.linalg.inv(self.JT_matrix)

# Create an instance of the Hexapod object
my_hexa = Hexapod()
