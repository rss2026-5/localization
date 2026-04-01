import numpy as np


class MotionModel:

    def __init__(self, node):
        node.declare_parameter('deterministic', False)
        self.deterministic = node.get_parameter('deterministic').get_parameter_value().bool_value

        self.translation_noise = 0.05
        self.rotation_noise    = 0.05
        self.drift_noise       = 0.01

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y1 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the same size
        """
        dx, dy, dtheta = odometry

        cos_t = np.cos(particles[:, 2])
        sin_t = np.sin(particles[:, 2])

        world_dx = cos_t * dx - sin_t * dy
        world_dy = sin_t * dx + cos_t * dy

        if self.deterministic:
            particles[:, 0] += world_dx
            particles[:, 1] += world_dy
            particles[:, 2] += dtheta
        else:
            N = particles.shape[0]
            dist = np.sqrt(dx**2 + dy**2)
            std_xy    = self.translation_noise * dist + self.drift_noise
            std_theta = self.rotation_noise * abs(dtheta) + self.drift_noise

            particles[:, 0] += world_dx + np.random.normal(0, std_xy, N)
            particles[:, 1] += world_dy + np.random.normal(0, std_xy, N)
            particles[:, 2] += dtheta   + np.random.normal(0, std_theta, N)

        return particles
