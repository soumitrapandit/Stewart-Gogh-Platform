import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Hexapod:
    """
    A class representing a six-legged robotic platform with inverse and forward kinematics,
    capable of simulating transformations and visualizing its movement in 3D.
    """
    def __init__(self, upper_rad=300/2, lower_rad=650/2, upper_lim=1100, lower_lim=100, alpha=40, beta=80, euler_order='xyz'):
        """
        Initializes the Hexapod with base and body properties, creating its legs and structural layout.
        :param upper_rad: Radius of the upper body
        :param lower_rad: Radius of the lower base
        :param upper_lim: Maximum extension limit of legs
        :param lower_lim: Minimum retraction limit of legs
        :param alpha: Angle configuration for base placement
        :param beta: Angle configuration for body placement
        :param euler_order: Order of Euler rotations
        """
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
        """
        Performs inverse kinematics to calculate leg vectors based on the desired pose.
        Ensures the leg lengths remain within physical limits.
        :param pose: Desired position and orientation of the hexapod
        :return: Computed leg vectors
        """
        upper_lim = self.upper_lim
        lower_lim = self.lower_lim
        self.transform_body(pose)
        for leg_length in self.leg_lengths:
            assert lower_lim < leg_length < upper_lim, "Leg length out of bounds!"
        return self.leg_vectors

    def fk(self, leg_lengths, error, guess_pose):
        """
        Performs forward kinematics to estimate the hexapod's pose given leg lengths.
        :param leg_lengths: Lengths of the hexapod legs
        :param error: Allowed error threshold
        :param guess_pose: Initial guess for the pose
        :return: Estimated pose of the hexapod
        """
        self.transform_body(guess_pose)
        delta_l = np.array(leg_lengths) - self.leg_lengths
        if (error >= delta_l).all():
            return guess_pose
        else:
            return self.fk(error=error, guess_pose=guess_pose + self.JT_inv @ delta_l)

    def show_robot(self):
        """
        Visualizes the hexapod structure in 3D using Matplotlib.
        """
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
        """
        Creates the base of the hexapod with 6 attachment points evenly spaced.
        """
        self.base_center = np.array([0, 0, 0])
        self.base_points = [np.array([np.cos(theta) * self.lower_rad, np.sin(theta) * self.lower_rad, 0])
                            for theta in np.deg2rad([-self.alpha/2 + i * (120 if i % 2 else self.alpha) for i in range(6)])]

    def calc_Rsi(self):
        """
        Computes the transformation matrix for each leg.
        """
        self.R_si = [bp - self.body_center for bp in self.body_points]

    def transform_body(self, pose):
        """
        Applies transformations to the hexapod body based on the given pose.
        """
        self.create_body()
        self.pose = pose
        self.T_mat()
        self.body_center = self.T_matrix[:3, 3]
        self.body_points = [np.dot(self.T_matrix, np.append(point, 1))[:3] for point in self.body_points]
        self.calc_Rsi()
        self.create_legs()
        self.Jacobian()
        self.JT_mat()

# Create an instance of the Hexapod object
my_hexa = Hexapod()
