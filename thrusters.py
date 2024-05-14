import math
import sys
from abc import ABC, abstractmethod

import numpy as np
import quaternion
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Vector3, Twist, Quaternion
from pca import PCA9685
from thruster_data import *

# from typing import override

sys.path.append('/home/jetson/catkin_ws/src/wurov2_0/src/controllers')

from controllers import PIDController, QuatPIDController

# 1: Front Left
# 2: Front Right
# 3: Back Left
# 4: Back Right

THRUSTER_ANGLE_DEG = 45
thruster_angles = np.array(
    [THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG]) * np.pi / 180
THRUSTER_LENGTH_DISTANCE_M = 0.5842
THRUSTER_WIDTH_DISTANCE_M = 0.4826
BAR_02_LENGTH_OFFSET = -0.2667
BAR_02_WIDTH_OFFSET = 0.04318
BAR_02_HEIGHT_OFFSET = 0.0254

BUOYANT_FORCE = -1000 * -9.8 * 000  # TODO: APPROXIMATE BUOYANT FORCE BY MEASURING vertical THRUSTER OUTPUT while holding depth TO GET VOLUME DENSITY

THRUSTER_DIAGONAL_DISTANCE_M = math.sqrt(THRUSTER_WIDTH_DISTANCE_M ** 2 + THRUSTER_LENGTH_DISTANCE_M ** 2)
HALF_LENGTH = THRUSTER_LENGTH_DISTANCE_M / 2
HALF_WIDTH = THRUSTER_WIDTH_DISTANCE_M / 2
HALF_DIAGONAL = THRUSTER_DIAGONAL_DISTANCE_M / 2
LENGTH_DIAGONAL_ANGLE_RAD = -math.acos(THRUSTER_LENGTH_DISTANCE_M / THRUSTER_DIAGONAL_DISTANCE_M)

YAW_TANGENTIAL_FORCE = math.sin(
    thruster_angles[0] - LENGTH_DIAGONAL_ANGLE_RAD)

horizontal_thruster_config = np.array([[math.cos(thruster_angles[0]),
                                        math.cos(thruster_angles[1]),
                                        -math.cos(thruster_angles[2]),
                                        -math.cos(thruster_angles[3])],
                                       [math.sin(thruster_angles[0]),
                                        -math.sin(thruster_angles[1]),
                                        math.sin(thruster_angles[2]),
                                        -math.sin(thruster_angles[3])],
                                       [HALF_DIAGONAL * math.sin(
                                           thruster_angles[0] - LENGTH_DIAGONAL_ANGLE_RAD),
                                        -HALF_DIAGONAL * math.sin(
                                            thruster_angles[1] - LENGTH_DIAGONAL_ANGLE_RAD),
                                        -HALF_DIAGONAL * math.sin(
                                            thruster_angles[2] - LENGTH_DIAGONAL_ANGLE_RAD),
                                        HALF_DIAGONAL * math.sin(
                                            thruster_angles[3] - LENGTH_DIAGONAL_ANGLE_RAD)]])

h_U, h_S, h_V_T = np.linalg.svd(horizontal_thruster_config)
h_S = np.diag(h_S)
h_S_inv = np.linalg.inv(h_S)

h_V = np.transpose(h_V_T)
h_S_inv_0 = np.vstack([h_S_inv, [0, 0, 0]])
h_U_T = np.transpose(h_U)

# Assuming positive thrust forces up
vertical_thruster_config = np.array([[1, 1, 1, 1],
                                     [-HALF_LENGTH, -HALF_LENGTH, HALF_LENGTH, HALF_LENGTH],
                                     [-HALF_WIDTH, HALF_WIDTH, -HALF_WIDTH, HALF_WIDTH]])

v_U, v_S, v_V_T = np.linalg.svd(vertical_thruster_config)
v_S = np.diag(v_S)
v_S_inv = np.linalg.inv(v_S)

v_V = np.transpose(v_V_T)
v_S_inv_0 = np.vstack([v_S_inv, [0, 0, 0]])
v_U_T = np.transpose(v_U)

horizontal_factor = h_V @ h_S_inv_0 @ h_U_T
vertical_factor = v_V @ v_S_inv_0 @ v_U_T

# Constants to use for feedforward control
MAX_THRUST_KGF = 1.768181818
MAX_NET_X_KGF = MAX_THRUST_KGF * 4 * math.cos(thruster_angles[0])
MAX_NET_Y_KGF = MAX_THRUST_KGF * 4 * math.sin(thruster_angles[0])
MAX_NET_Z_KGF = MAX_THRUST_KGF * 4
MAX_NET_YAW_MOMENT_KGF = MAX_THRUST_KGF * 4 * YAW_TANGENTIAL_FORCE
MAX_NET_PITCH_MOMENT_KGF = MAX_THRUST_KGF * 4 * HALF_LENGTH
MAX_NET_ROLL_MOMENT_KGF = MAX_THRUST_KGF * 4 * HALF_WIDTH
MAX_DIAGONAL_THRUST = MAX_THRUST_KGF * 2

MAX_YAW_RATE_RAD_S = 1
MAX_PITCH_RATE_RAD_S = 1
MAX_ROLL_RATE_RAD_S = 1

NET_CURRENT_LIMIT = 20


# Logistic boolean input: input(t) = 1 / (1 + e^(-8 * (t - .5)))


def quadratic_solve(y, a, b, c):
    x1 = -b / (2 * a)
    x2 = math.sqrt(b ** 2 - 4 * a * (c - y)) / (2 * a)
    return (x1 + x2), (x1 - x2)


def desaturate_thrust_outputs(outputs, max_thrust):
    real_max_thrust = 0
    for i in range(len(outputs)):
        real_max_thrust = max(real_max_thrust, abs(outputs[i]))

    if real_max_thrust > max_thrust:
        for i in range(len(outputs)):
            outputs[i] = outputs[i] / real_max_thrust * max_thrust


class Thruster(ABC):
    @abstractmethod
    def get_us_from_thrust(self, thrust_kgf):
        pass

    @abstractmethod
    def get_current_from_us(self, us):
        pass

    @abstractmethod
    def get_us_from_current(self, current, reverse):
        pass


def current_limit(thrusters: "list[Thruster]", pwm_us: "list[float]", net_current_limit):
    current_sum = 0
    currents = [0] * len(thrusters)
    for i in range(len(thrusters)):
        c = thrusters[i].get_current_from_us(pwm_us[i])
        current_sum += c
        currents[i] = c

    if current_sum > net_current_limit:
        current_scale_back = net_current_limit / current_sum
        for i in range(len(thrusters)):
            pwm_us[i] = thrusters[i].get_us_from_current(currents[i] * current_scale_back, pwm_us[i] < 1500)


def get_vert_thruster_outputs(z_force, pitch, roll):
    forces = np.array([z_force, pitch, roll])
    return vertical_factor @ forces


def get_horizontal_thruster_outputs(x, y, yaw):
    net_thrust_desired = np.array([x, y, yaw])
    return horizontal_factor @ net_thrust_desired


def get_thruster_outputs(x, y, z, roll, pitch, yaw, max_thrust=MAX_THRUST_KGF) -> np.array:
    horizontal = get_horizontal_thruster_outputs(x, y, yaw)
    vertical = get_vert_thruster_outputs(z, pitch, roll)
    outputs = np.concatenate((horizontal, vertical))
    desaturate_thrust_outputs(outputs, max_thrust)
    return outputs


def rotate_2d(x, y, angle_rad):
    x_p = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_p = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_p, y_p


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def to_np_quat(q: Quaternion) -> quaternion:
    return np.quaternion(q.w, q.x, q.y, q.z)


class T100Thruster(Thruster):
    t100_mid = int(len(t100_thrust_12v) / 2)
    t100_thrust_left = np.poly1d(np.polyfit(t100_pwm_value[:(t100_mid - 2)], t100_thrust_12v[:(t100_mid - 2)], 2))
    t100_thrust_right = np.poly1d(np.polyfit(t100_pwm_value[(t100_mid + 2):], t100_thrust_12v[(t100_mid + 2):], 2))
    t100_current_full = np.poly1d(np.polyfit(t100_pwm_value, t100_current_12v, 4))
    t100_current_left = np.poly1d(np.polyfit(t100_pwm_value[:(t100_mid - 2)], t100_current_12v[:(t100_mid - 2)], 2))
    t100_current_right = np.poly1d(np.polyfit(t100_pwm_value[(t100_mid + 2):], t100_current_12v[(t100_mid + 2):], 2))

    def get_us_from_thrust(self, thrust_kgf):
        reverse_thrust = thrust_kgf < 0

        if thrust_kgf > 0:
            micros = \
                quadratic_solve(thrust_kgf, T100Thruster.t100_thrust_right.c[0], T100Thruster.t100_thrust_right.c[1],
                                T100Thruster.t100_thrust_right.c[2])[1 if reverse_thrust else 0]
        elif thrust_kgf < 0:
            micros = \
                quadratic_solve(-thrust_kgf, T100Thruster.t100_thrust_left.c[0], T100Thruster.t100_thrust_left.c[1],
                                T100Thruster.t100_thrust_left.c[2])[1 if reverse_thrust else 0]
        else:
            micros = 1500

        return micros

    def get_current_from_us(self, us):
        if us < 1500 - 20 or us > 1500 + 20:
            return T100Thruster.t100_current_full(us)
        else:
            return 0.0

    def get_us_from_current(self, current, reverse):
        coeffs = T100Thruster.t100_current_full.c.copy()
        coeffs[-1] -= current
        solved = np.roots(coeffs)
        actual_roots = []
        for r in solved:
            if 1000 < r < 2000:
                actual_roots.append(r)

        if reverse:
            return min(actual_roots)
        else:
            return max(actual_roots)


# noinspection DuplicatedCode
class T200Thruster(Thruster):
    # abs the list of thrusts because the t100 thrust values are all positive
    for i in range(len(t200_thrust_12v)):
        t200_thrust_12v[i] = abs(t200_thrust_12v[i])

    t200_mid = int(len(t200_thrust_12v) / 2)
    t200_thrust_left = np.poly1d(np.polyfit(t200_pwm_value[:(t200_mid - 9)], t200_thrust_12v[:(t200_mid - 9)], 2))
    t200_thrust_right = np.poly1d(np.polyfit(t200_pwm_value[(t200_mid + 9):], t200_thrust_12v[(t200_mid + 9):], 2))
    t200_current_full = np.poly1d(np.polyfit(t200_pwm_value, t200_current_12v, 8))

    t200_current_left = np.poly1d(np.polyfit(t200_pwm_value[:(t200_mid - 9)], t200_current_12v[:(t200_mid - 9)], 2))
    t200_current_right = np.poly1d(np.polyfit(t200_pwm_value[(t200_mid + 9):], t200_current_12v[(t200_mid + 9):], 2))

    def get_us_from_thrust(self, thrust_kgf):
        reverse_thrust = thrust_kgf < 0

        if thrust_kgf > 0:
            micros = \
                quadratic_solve(thrust_kgf, T200Thruster.t200_thrust_right.c[0], T200Thruster.t200_thrust_right.c[1],
                                T200Thruster.t200_thrust_right.c[2])[1 if reverse_thrust else 0]
        elif thrust_kgf < 0:
            micros = \
                quadratic_solve(-thrust_kgf, T200Thruster.t200_thrust_left.c[0], T200Thruster.t200_thrust_left.c[1],
                                T200Thruster.t200_thrust_left.c[2])[1 if reverse_thrust else 0]
        else:
            micros = 1500

        return micros

    def get_current_from_us(self, us):
        if us < 1500 - 4 * 9 or us > 1500 + 4 * 9:
            return T200Thruster.t200_current_full(us)
        else:
            return 0.0

    def get_us_from_current(self, current, reverse):
        coeffs = T200Thruster.t200_current_full.c.copy()
        coeffs[-1] -= current
        solved = np.roots(coeffs)
        actual_roots = []
        for r in solved:
            if 1000 < r < 2000:
                actual_roots.append(r)

        if reverse:
            return min(actual_roots)
        else:
            return max(actual_roots)


class Thrusters:
    # Horizontal thruster PCA slots
    __FLH_ID = 0
    __FRH_ID = 5
    __BLH_ID = 1
    __BRH_ID = 6
    # Vertical thruster PCA slots__FLH_ID
    __FLV_ID = 3
    __FRV_ID = 4
    __BLV_ID = 2
    __BRV_ID = 7

    # Thruster creation
    flh_thruster = T200Thruster()
    frh_thruster = T200Thruster()
    blh_thruster = T200Thruster()
    brh_thruster = T200Thruster()
    flv_thruster = T200Thruster()
    frv_thruster = T200Thruster()
    blv_thruster = T200Thruster()
    brv_thruster = T200Thruster()
    all_thrusters = [flh_thruster, frh_thruster, blh_thruster, brh_thruster,
                     flv_thruster, frv_thruster, blv_thruster, brv_thruster]

    def __init__(self):
        self.__pca = PCA9685(0x40, 100, measured_frequency_hz=105.6)
        self.__pca.software_reset()
        self.__pca.setup()
        self.__pca.set_sleep(False)

        self.desired_twist = Twist()

        self.rotation_quat = Quaternion(0, 0, 0, 1)
        self.desired_rotation = Quaternion(0, 0, 0, 1)
        self.rotation_offset = np.quaternion(1, 0, 0, 0)
        self.test_rot_setpoint = np.quaternion(1, 0, 0, 0)

        self.thrust_outputs = [0] * 8
        self.us_outputs = [0] * 8

        self.previous_roll = 0
        self.previous_pitch = 0
        self.previous_yaw = 0

        self.measured_depth = 0

        # self.roll_controller = PIDController(p=0.1111, i=0, d=0)
        # self.pitch_controller = PIDController(p=0.1111)
        # self.yaw_controller = PIDController(p=0.11111)
        self.quat_controller = QuatPIDController(p=1.1)
        self.depth_controller = PIDController(p=.5, d=0)

    @property
    def pca(self):
        return self.__pca

    def set_rotation(self, r: Quaternion) -> None:
        """
        Set the current ROV rotation

        Args:
            r (Quaternion): ROV rotation
        """
        # a = Quaternion(r.z, r.y, r.x, r.w)
        self.rotation_quat = r

    def get_ros_rotation(self):
        return self.rotation_quat

    def set_desired_rotation(self, r_d: Quaternion) -> None:
        self.desired_rotation = r_d

    def set_rotation_offset(self):
        # self.rotation_offset = self.rotation_quat
        self.rotation_offset = to_np_quat(self.rotation_quat)

    def set_measured_depth(self, depth_m):
        self.measured_depth = depth_m

    def get_current_rotation(self) -> quaternion:
        r_c = to_np_quat(self.rotation_quat)  # Rotation current
        r_o = self.rotation_offset  # Rotation offset
        r_r = r_c.inverse() * r_o  # Rotation of ROV
        # print(r_r)
        return r_r

    def get_depth(self):
        q = self.get_current_rotation()
        q_sci = Rotation.from_quat([q.x, q.y, q.z, q.w])
        bar02_offset = (BAR_02_LENGTH_OFFSET, BAR_02_WIDTH_OFFSET, BAR_02_HEIGHT_OFFSET)  # xyz point
        position = q_sci.apply(bar02_offset)
        depth_offset = position[2]  # Only care about z because z is depth

        return self.measured_depth - depth_offset  # TODO Add instead?

    def set_test_rot_setpoint(self):
        self.test_rot_setpoint = self.get_current_rotation()

    def get_ros_quat(self):
        ros_quat = Quaternion()
        q = self.get_current_rotation()
        q_arr = quaternion.as_float_array(q)

        ros_quat.w = q_arr[0]
        ros_quat.x = q_arr[1]
        ros_quat.y = q_arr[2]
        ros_quat.z = q_arr[3]

        return ros_quat

    def get_ros_test_rot_setpoint_euler(self):
        ros_rot = Vector3()
        e = quaternion.as_euler_angles(self.test_rot_setpoint)
        ros_rot.x = e[2] * (180. / math.pi)
        ros_rot.y = e[1] * (180. / math.pi)
        ros_rot.z = e[0] * (180. / math.pi)
        return ros_rot

    def get_ros_current_rotation_euler(self):
        ros_rot = Vector3()
        q = self.get_current_rotation()
        q_arr = quaternion.as_float_array(q)
        e = euler_from_quaternion(q_arr[0], q_arr[1], q_arr[2], q_arr[3])
        # e = quaternion.as_euler_angles(q)
        ros_rot.x = e[2] * (180.0 / math.pi)
        ros_rot.y = e[1] * (180.0 / math.pi)
        ros_rot.z = e[0] * (180.0 / math.pi)
        return ros_rot

    def get_roll_pitch_feedforwards(self, rot: Quaternion):
        r = euler_from_quaternion(rot.w, rot.y, rot.z, rot.w)
        pitch_ff = math.sin(r[1]) * HALF_LENGTH
        roll_ff = math.sin(r[0]) * HALF_WIDTH
        return roll_ff, pitch_ff

    def get_thrust_outputs(self):
        return self.thrust_outputs

    def get_pwm_period_outputs(self):
        return self.us_outputs

    def print_thrust_vector(self, thruster_outputs, print_vector_numbers=False):
        horizontal_input = horizontal_thruster_config @ thruster_outputs[:4]
        vertical_input = vertical_thruster_config @ thruster_outputs[4:]

        if horizontal_input[0] > 0.0001:
            x_direction = 'FORWARD'
        elif horizontal_input[0] < -0.0001:
            x_direction = 'BACKWARD'
        else:
            x_direction = 'ZERO'

        if horizontal_input[1] > 0.0001:
            y_direction = 'LEFT'
        elif horizontal_input[1] < -0.0001:
            y_direction = 'RIGHT'
        else:
            y_direction = 'ZERO'

        if vertical_input[0] > 0.0001:
            z_direction = 'UP'
        elif vertical_input[0] < -0.0001:
            z_direction = 'DOWN'
        else:
            z_direction = 'ZERO'

        if horizontal_input[2] > 0.0001:
            yaw_direction = 'LEFT (CCW)'
        elif horizontal_input[2] < -0.0001:
            yaw_direction = 'RIGHT (CW)'
        else:
            yaw_direction = 'ZERO'

        if vertical_input[1] > 0.0001:
            pitch_direction = 'UP (CCW)'
        elif vertical_input[1] < -0.0001:
            pitch_direction = 'DOWN (CW)'
        else:
            pitch_direction = 'ZERO'

        if vertical_input[2] > 0.0001:
            roll_direction = 'LEFT (CCW)'
        elif vertical_input[2] < -0.0001:
            roll_direction = 'RIGHT (CW)'
        else:
            roll_direction = 'ZERO'

        if print_vector_numbers:
            print(f'Linear XYZ: ({horizontal_input[0]}, {horizontal_input[1]}, {vertical_input[0]}), ' \
                  f'Angular XYZ: ({vertical_input[2], vertical_input[1], horizontal_input[2]})')
        print(
            f'X: {x_direction}, Y: {y_direction}, Z: {z_direction}, Roll: {roll_direction}, Pitch: {pitch_direction}, Yaw: {yaw_direction}\n')

    def copy_twist(self, t):
        new_twist = Twist()

        new_twist.linear.x = t.linear.x
        new_twist.linear.y = t.linear.y
        new_twist.linear.z = t.linear.z

        new_twist.angular.x = t.angular.x
        new_twist.angular.y = t.angular.y
        new_twist.angular.z = t.angular.z

        return new_twist

    def set_thrust(self, t: Twist, depth_lock: bool = False,
                   depth_command: float = None, depth_setpoint: float = None) -> None:
        """
        Set the desired thrust vector

        Args:
            t (Twist):               Desired ROV twist
            depth_lock (bool, optional):        Drive the ROV at a constant depth. Defaults to False.
            depth_command (float, optional):    Control the ROV depth thrust manually. Defaults to None.
            depth_setpoint (float, optional):   Use PIDF to go to a certain depth setpoint
        """

        thrust_twist = self.copy_twist(t)

        q = self.get_current_rotation()
        try:
            # Turn current rotation quat into scipy rotation
            current_rot_sci = Rotation.from_quat([q.x, q.y, q.z, q.w])

            # current_rot_sci_no_z = Rotation.from_quat()
        except ValueError:
            print('depth_lock not applied (invalid quat)')
            self.desired_twist = thrust_twist
            return

        if depth_setpoint:
            self.depth_controller.set_setpoint(depth_setpoint)
            depth_vec = np.array([0, 0, -self.depth_controller.calculate(self.get_depth())])
            depth_command = current_rot_sci.apply(depth_vec)
        elif depth_command:
            depth_vec = np.array([0, 0, depth_command])
            depth_command = current_rot_sci.apply(depth_vec)
        else:
            depth_command = np.array([0, 0, 0])

        # print(f'depth_command = {depth_command}')
        if depth_lock:
            # Keep only x and y components of the direction
            rov_relative_linear = rotate_2d(thrust_twist.linear.x, thrust_twist.linear.y,
                                            current_rot_sci.as_euler('xyz')[2])
            desired_direction = np.array([rov_relative_linear[0], rov_relative_linear[1], 0])

            # Rotate current rotation to the desir ed direction
            # Rotate desired direction by the current rotation
            rov_direction = current_rot_sci.apply(desired_direction)
            thrust_twist.linear.x = rov_direction[0]
            thrust_twist.linear.y = rov_direction[1]
            thrust_twist.linear.z = rov_direction[2]

        thrust_twist.linear.x += depth_command[0]
        thrust_twist.linear.y += depth_command[1]
        thrust_twist.linear.z += depth_command[2]
        # print(f'thrust_twist.linear.x = {thrust_twist.linear.x}')

        self.desired_twist = thrust_twist

    def update(self, control_orientation=False) -> None:
        """
        Calculate PID outputs and set PCA PWM values
        """

        d = self.desired_twist

        if control_orientation:
            q_r = self.get_current_rotation()
            q_d = to_np_quat(self.test_rot_setpoint)
            self.quat_controller.set_setpoint(q_d)
            qc_output = self.quat_controller.calculate(q_r)
            d.angular.x = qc_output[0]
            d.angular.y = qc_output[1]
            d.angular.z = qc_output[2]

        thrust_outputs = get_thruster_outputs(d.linear.x, d.linear.y, d.linear.z, d.angular.x, d.angular.y, d.angular.z)

        print(
            'FLH: %02.04f FRH: %02.04f BLH: %02.04f BRH: %02.04f | FLV: %02.04f FRV: %02.04f BLV: %02.04f BRV: %02.04f'
            % (thrust_outputs[0], thrust_outputs[1], thrust_outputs[2], thrust_outputs[3],
               thrust_outputs[4], thrust_outputs[5], thrust_outputs[6], thrust_outputs[7]))

        # Calculate PCA microsecond +period for each thruster
        us_outputs = [Thrusters.all_thrusters[i].get_us_from_thrust(thrust_outputs[i]) for i in
                      range(len(thrust_outputs))]

        self.thrust_outputs = thrust_outputs
        self.us_outputs = us_outputs

        q = self.get_current_rotation()
        e = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)
        print(e)

        # Current limit to avoid exploding the robot
        # current_limit(Thrusters.all_thrusters, us_outputs, NET_CURRENT_LIMIT)

        # Re-order us_outputs to allow just one I2C block write
        us_outputs_reordered = [us_outputs[0], us_outputs[2], us_outputs[6], us_outputs[4], us_outputs[5],
                                us_outputs[1], us_outputs[3], us_outputs[7]]

        # print(f'PCA outputs: {us_outputs_reordered}')
        try:
            self.__pca.set_us(Thrusters.__FLH_ID, us_outputs_reordered)
            pass
        except OSError as e:
            print(f"[ERROR] Resetting PCA outputs \n {e}")
            us_outputs_reordered = [1500] * 8
            self.__pca.set_us(Thrusters.__FLH_ID, us_outputs_reordered)


if __name__ == '__main__':
    rot = [0, 0, 0]
    rot_q = Rotation.from_euler('xyz', rot, degrees=True).as_quat(canonical=False)
    # Rotation.from_quat()
    # rot_q = quaternion.from_euler_angles([c * math.pi / 180.0 for c in rot])

    rot_q_ros = Quaternion(rot_q[0], rot_q[1], rot_q[2], rot_q[3])
    print('%f, %f, %f, %f' % (rot_q_ros.w, rot_q_ros.x, rot_q_ros.y, rot_q_ros.z))

    thrusters = Thrusters()

    thrusters.set_rotation(rot_q_ros)
    thrusters.set_measured_depth(1)
    print(f'Depth = {thrusters.get_depth()}')

    tw = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    thrusters.set_thrust(tw, depth_lock=False, depth_setpoint=1)
    thrusters.update()
