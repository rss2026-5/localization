import numpy as np
from scan_simulator_2d import PyScanSimulator2D
# Try to change to just `from scan_simulator_2d import PyScanSimulator2D`
# if any error re: scan_simulator_2d occurs

from tf_transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid

import sys

np.set_printoptions(threshold=sys.maxsize)


class SensorModel:

    def __init__(self, node):
        node.declare_parameter('map_topic', "default")
        node.declare_parameter('num_beams_per_particle', 1)
        node.declare_parameter('scan_theta_discretization', 1.0)
        node.declare_parameter('scan_field_of_view', 1.0)
        node.declare_parameter('lidar_scale_to_map_scale', 1.0)

        self.map_topic = node.get_parameter('map_topic').get_parameter_value().string_value
        self.num_beams_per_particle = node.get_parameter('num_beams_per_particle').get_parameter_value().integer_value
        self.scan_theta_discretization = node.get_parameter(
            'scan_theta_discretization').get_parameter_value().double_value
        self.scan_field_of_view = node.get_parameter('scan_field_of_view').get_parameter_value().double_value
        self.lidar_scale_to_map_scale = node.get_parameter(
            'lidar_scale_to_map_scale').get_parameter_value().double_value

        ####################################
        # Sensor model mixing weights
        self.alpha_hit   = 0.74
        self.alpha_short = 0.07
        self.alpha_max   = 0.07
        self.alpha_rand  = 0.12
        self.sigma_hit   = 8.0   # in table-index units (not meters)

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        ####################################

        node.get_logger().info("%s" % self.map_topic)
        node.get_logger().info("%s" % self.num_beams_per_particle)
        node.get_logger().info("%s" % self.scan_theta_discretization)
        node.get_logger().info("%s" % self.scan_field_of_view)

        # Precompute the sensor model table
        self.sensor_model_table = np.empty((self.table_width, self.table_width))
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
            self.num_beams_per_particle,
            self.scan_field_of_view,
            0,  # This is not the simulator, don't add noise
            0.01,  # This is used as an epsilon
            self.scan_theta_discretization)

        # Subscribe to the map
        self.map = None
        self.map_set = False
        self.map_subscriber = node.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            1)

    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.

        Table is indexed as table[z_index, d_index] where:
          - z_index is the (discretized) measured range
          - d_index is the (discretized) expected range from ray tracing
        Both indices run from 0 to table_width-1, mapping linearly to
        [0, z_max] where z_max = table_width - 1 (index units).

        args:
            N/A

        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        W = self.table_width
        z_max = W - 1  # max index value

        # 1D arrays of index values for z (rows) and d (columns)
        z = np.arange(W, dtype=float)   # shape (W,)
        d = np.arange(W, dtype=float)   # shape (W,)

        # Broadcast to 2D: rows = z, cols = d
        Z = z[:, np.newaxis]  # (W, 1)
        D = d[np.newaxis, :]  # (1, W)

        # ---- p_hit: Gaussian centered at d — normalize per column ----
        p_hit = np.exp(-0.5 * ((Z - D) / self.sigma_hit) ** 2)
        col_sums_hit = p_hit.sum(axis=0, keepdims=True)
        col_sums_hit = np.where(col_sums_hit == 0, 1.0, col_sums_hit)
        p_hit = p_hit / col_sums_hit

        # ---- p_short: (2/d)*(1-z/d) for z<=d, raw (not column-normalized) ----
        with np.errstate(divide='ignore', invalid='ignore'):
            p_short = np.where(
                (Z <= D) & (D > 0),
                (2.0 / D) * (1.0 - Z / D),
                0.0
            )
        p_short = np.nan_to_num(p_short, nan=0.0)

        # ---- p_max: spike at z_max (from debug table: already normalized) ----
        p_max = np.zeros((W, W))
        p_max[z_max, :] = 1.0

        # ---- p_rand: uniform — from debug table uses 1/table_width ----
        p_rand = np.ones((W, W)) / W

        # ---- Mix, then final column normalization ----
        table = (self.alpha_hit   * p_hit  +
                 self.alpha_short * p_short +
                 self.alpha_max   * p_max   +
                 self.alpha_rand  * p_rand)

        col_sums_final = table.sum(axis=0, keepdims=True)
        col_sums_final = np.where(col_sums_final == 0, 1.0, col_sums_final)
        self.sensor_model_table = table / col_sums_final

    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y1 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar. THIS IS Z_K. Each range in Z_K is Z_K^i

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        # Ray trace expected distances for all particles: shape (N, num_beams)
        scans = self.scan_sim.scan(particles)

        # Convert from meters to table index units
        scale = self.resolution * self.lidar_scale_to_map_scale
        z_max_idx = self.table_width - 1

        # Expected ranges (from ray tracer) → indices, shape (N, num_beams)
        d_indices = np.clip(
            (scans / scale).astype(int),
            0, z_max_idx
        )

        # Observed ranges → indices, shape (num_beams,)
        z_indices = np.clip(
            (observation / scale).astype(int),
            0, z_max_idx
        )

        # Lookup probabilities: table[z, d] for each beam of each particle
        # z_indices broadcast over N particles: (num_beams,) vs (N, num_beams)
        probs = self.sensor_model_table[z_indices[np.newaxis, :], d_indices]  # (N, num_beams)

        # Product over beams per particle, then squash to reduce peaking
        log_probs = np.sum(np.log(probs + 1e-300), axis=1)  # (N,)
        weights = np.exp(log_probs)

        # Squash: raise to power < 1 to broaden distribution
        weights = weights ** (1.0 / 3.0)

        return weights

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double) / 100.
        self.map = np.clip(self.map, 0, 1)

        self.resolution = map_msg.info.resolution

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = euler_from_quaternion((
            origin_o.x,
            origin_o.y,
            origin_o.z,
            origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
            self.map,
            map_msg.info.height,
            map_msg.info.width,
            map_msg.info.resolution,
            origin,
            0.5)  # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
