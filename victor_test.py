import numpy as np
import matplotlib.pyplot as plt
import math
from abc import ABC, abstractmethod
from typing import override

def quadratic_solve(y, a, b, c):
    x1 = -b / (2 * a)
    x2 = math.sqrt(b ** 2 - 4 * a * (c - y)) / (2 * a)
    return (x1 + x2), (x1 - x2)


class Thruster(ABC):
    @abstractmethod
    def get_us(self, thrust_kgf):
        pass


class T100Thruster(Thruster):
    t100_pwm_value = [1100, 1110, 1120, 1130, 1140, 1150, 1160, 1170, 1180, 1190, 1200, 1210, 1220, 1230, 1240, 1250,
                      1260, 1270, 1280, 1290, 1300, 1310, 1320, 1330, 1340, 1350, 1360, 1370, 1380, 1390, 1400, 1410,
                      1420, 1430, 1440,
                      1450, 1460, 1470, 1480, 1500, 1510, 1520, 1530, 1540, 1550, 1560, 1570, 1580, 1590, 1600, 1610,
                      1620, 1630, 1640,
                      1650, 1660, 1670, 1680, 1690, 1700, 1710, 1720, 1730, 1740, 1750, 1760, 1770, 1780, 1790, 1800,
                      1810, 1820, 1830,
                      1840, 1850, 1860, 1870, 1880, 1890, 1900]

    t100_thrust_12v = [1.768181818, 1.640909091, 1.577272727, 1.527272727, 1.440909091, 1.4, 1.322727273, 1.259090909,
                       1.209090909, 1.163636364, 1.104545455, 1.040909091, 0.990909091, 0.927272727, 0.854545455,
                       0.790909091, 0.754545455, 0.704545455, 0.668181818, 0.622727273, 0.581818182, 0.531818182,
                       0.472727273, 0.427272727, 0.4, 0.368181818, 0.327272727, 0.272727273, 0.231818182, 0.2,
                       0.168181818, 0.140909091, 0.104545455, 0.072727273, 0.05, 0.031818182, 0.013636364, 0.009090909,
                       0, 0, 0, 0, 0.009090909, 0.036363636, 0.063636364, 0.104545455, 0.145454545, 0.195454545,
                       0.254545455,
                       0.309090909, 0.368181818, 0.431818182, 0.481818182, 0.545454545, 0.613636364, 0.686363636,
                       0.736363636, 0.804545455, 0.881818182, 0.963636364, 1.059090909, 1.131818182, 1.186363636,
                       1.254545455, 1.304545455, 1.386363636, 1.490909091, 1.577272727, 1.654545455, 1.727272727,
                       1.822727273, 1.959090909, 2.045454545, 2.1, 2.181818182, 2.263636364, 2.322727273, 2.418181818,
                       2.486363636, 2.518181818]

    t100_current_12v = [13.33, 12.24, 11.4, 10.67, 10.01, 9.32, 8.64, 8.06, 7.41, 6.93, 6.36, 5.85, 5.29, 4.82, 4.44,
                        4.09, 3.73, 3.37, 2.99, 2.76, 2.46, 2.19, 1.91, 1.67, 1.47, 1.29, 1.11, 0.95, 0.79, 0.66, 0.55,
                        0.46, 0.37, 0.29, 0.23, 0.17, 0.13, 0.09, 0.03, 0.05, 0.04, 0.04, 0.1, 0.14, 0.18, 0.23, 0.3,
                        0.39, 0.49, 0.6, 0.72, 0.8, 0.98, 1.2, 1.4, 1.61, 1.82, 2.06, 2.36, 2.67, 3.01, 3.32, 3.67,
                        4.04, 4.38, 4.75, 5.22, 5.66, 6.16, 6.73, 7.24, 7.86, 8.43, 8.91, 9.57, 10.24, 10.87, 11.62,
                        12.4, 13.26]

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


    def get_us(self, t):
        return self.get_us_from_thrust(t)


    def get_current_from_us(self, us):
        if us < 1500 - 20:
            return T100Thruster.t100_current_left(us)
        elif us > 1500 + 20:
            return T100Thruster.t100_current_right(us)
        else:
            return 0.0

    def get_thrust_from_us(self, us):
        if us < 1500 - 20:
            return T100Thruster.t100_thrust_left(us)
        elif us > 1500 + 20:
            return T100Thruster.t100_thrust_right(us)
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



class T200Thruster(Thruster):
    t200_pwm_value = [1100, 1104, 1108, 1112, 1116, 1120, 1124, 1128, 1132, 1136, 1140, 1144, 1148, 1152, 1156, 1160,
                      1164,
                      1168, 1172, 1176, 1180, 1184, 1188, 1192, 1196, 1200, 1204, 1208, 1212, 1216, 1220, 1224, 1228,
                      1232,
                      1236, 1240, 1244, 1248, 1252, 1256, 1260, 1264, 1268, 1272, 1276, 1280, 1284, 1288, 1292, 1296,
                      1300,
                      1304, 1308, 1312, 1316, 1320, 1324, 1328, 1332, 1336, 1340, 1344, 1348, 1352, 1356, 1360, 1364,
                      1368,
                      1372, 1376, 1380, 1384, 1388, 1392, 1396, 1400, 1404, 1408, 1412, 1416, 1420, 1424, 1428, 1432,
                      1436,
                      1440, 1444, 1448, 1452, 1456, 1460, 1464, 1468, 1472, 1476, 1480, 1484, 1488, 1492, 1496, 1500,
                      1504,
                      1508, 1512, 1516, 1520, 1524, 1528, 1532, 1536, 1540, 1544, 1548, 1552, 1556, 1560, 1564, 1568,
                      1572,
                      1576, 1580, 1584, 1588, 1592, 1596, 1600, 1604, 1608, 1612, 1616, 1620, 1624, 1628, 1632, 1636,
                      1640,
                      1644, 1648, 1652, 1656, 1660, 1664, 1668, 1672, 1676, 1680, 1684, 1688, 1692, 1696, 1700, 1704,
                      1708,
                      1712, 1716, 1720, 1724, 1728, 1732, 1736, 1740, 1744, 1748, 1752, 1756, 1760, 1764, 1768, 1772,
                      1776,
                      1780, 1784, 1788, 1792, 1796, 1800, 1804, 1808, 1812, 1816, 1820, 1824, 1828, 1832, 1836, 1840,
                      1844,
                      1848, 1852, 1856, 1860, 1864, 1868, 1872, 1876, 1880, 1884, 1888, 1892, 1896, 1900]

    t200_thrust_12v = [-2.90, -2.92, -2.89, -2.83, -2.79, -2.76, -2.72, -2.67, -2.60, -2.59, -2.56, -2.49, -2.44, -2.43,
                       -2.39, -2.34, -2.30, -2.25, -2.23, -2.18, -2.14, -2.10, -2.07, -2.01, -1.98, -1.95, -1.88, -1.85,
                       -1.81, -1.78, -1.73, -1.66, -1.65, -1.61, -1.56, -1.53, -1.49, -1.47, -1.44, -1.40, -1.37, -1.33,
                       -1.29, -1.28, -1.22, -1.19, -1.15, -1.12, -1.08, -1.04, -1.02, -0.99, -0.96, -0.93, -0.90, -0.87,
                       -0.83, -0.79, -0.77, -0.74, -0.72, -0.69, -0.66, -0.64, -0.60, -0.57, -0.54, -0.52, -0.49, -0.47,
                       -0.44, -0.42, -0.39, -0.37, -0.34, -0.32, -0.29, -0.27, -0.24, -0.23, -0.20, -0.18, -0.16, -0.15,
                       -0.12, -0.11, -0.09, -0.07, -0.06, -0.05, -0.04, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                       0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.04, 0.05, 0.07, 0.10, 0.11,
                       0.13, 0.15, 0.18, 0.20, 0.22, 0.25, 0.28, 0.31, 0.33, 0.37, 0.39, 0.43, 0.46, 0.49, 0.52, 0.55,
                       0.59,
                       0.63, 0.65, 0.68, 0.71, 0.76, 0.79, 0.83, 0.86, 0.89, 0.93, 0.97, 1.00, 1.04, 1.08, 1.14, 1.16,
                       1.20,
                       1.23, 1.28, 1.31, 1.35, 1.40, 1.43, 1.48, 1.53, 1.56, 1.63, 1.67, 1.71, 1.77, 1.82, 1.85, 1.91,
                       1.92,
                       1.96, 2.03, 2.09, 2.13, 2.18, 2.24, 2.27, 2.33, 2.40, 2.46, 2.51, 2.56, 2.62, 2.65, 2.71, 2.78,
                       2.84,
                       2.87, 2.93, 3.01, 3.04, 3.08, 3.16, 3.23, 3.26, 3.32, 3.38, 3.40, 3.45, 3.50, 3.57, 3.64, 3.71,
                       3.69,
                       3.71]

    t200_current_12v = [17.03, 17.08, 16.76, 16.52, 16.08, 15.69, 15.31, 15, 14.51, 14.17, 13.82, 13.46, 13.08, 12.8,
                        12.4, 12, 11.66, 11.31, 11.1, 10.74, 10.5, 10.11, 9.84, 9.5, 9.2, 8.9, 8.6, 8.3, 8, 7.7, 7.4,
                        7.1, 6.9, 6.6, 6.4, 6.2, 5.99, 5.77, 5.5, 5.32, 5.17, 4.9, 4.7, 4.56, 4.3, 4.1, 3.9, 3.73, 3.6,
                        3.4, 3.3, 3.1, 2.98, 2.8, 2.7, 2.41, 2.3, 2.1, 2, 1.9, 1.8, 1.7, 1.6, 1.5, 1.31, 1.3, 1.2, 1.1,
                        1, 0.9, 0.8, 0.8, 0.7, 0.6, 0.5, 0.5, 0.41, 0.4, 0.4, 0.3, 0.29, 0.2, 0.2, 0.2, 0.1, 0.1, 0.1,
                        0.05, 0.05, 0.05, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0.05,
                        0.05, 0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.3, 0.3, 0.4, 0.4, 0.5, 0.5, 0.6, 0.7, 0.7, 0.8, 0.8,
                        1, 1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 2, 2.1, 2.2, 2.3, 2.5, 2.8, 2.9, 3, 3.2, 3.3, 3.5,
                        3.67, 3.8, 4, 4.2, 4.4, 4.6, 4.8, 5, 5.2, 5.4, 5.69, 5.8, 6, 6.3, 6.5, 6.7, 7, 7.2, 7.5, 7.8, 8,
                        8.32, 8.64, 8.9, 9.24, 9.5, 9.82, 10.14, 10.45, 10.72, 11.1, 11.32, 11.62, 12.01, 12.37, 12.61,
                        13.04, 13.44, 13.7, 14.11, 14.4, 14.76, 15.13, 15.52, 15.87, 16.3, 16.74, 16.86, 16.91]

    # abs the list of thrusts because the t100 thrust values are all positive
    for i in range(len(t200_thrust_12v)):
        t200_thrust_12v[i] = abs(t200_thrust_12v[i])

    t200_mid = int(len(t200_thrust_12v) / 2)
    t200_thrust_left = np.poly1d(np.polyfit(t200_pwm_value[:(t200_mid - 9)], t200_thrust_12v[:(t200_mid - 9)], 2))
    t200_thrust_right = np.poly1d(np.polyfit(t200_pwm_value[(t200_mid + 9):], t200_thrust_12v[(t200_mid + 9):], 2))
    t200_current_full = np.poly1d(np.polyfit(t200_pwm_value, t200_current_12v, 8))

    t200_current_left = np.poly1d(np.polyfit(t200_pwm_value[:(t200_mid - 9)], t200_current_12v[:(t200_mid - 9)], 2))
    t200_current_right = np.poly1d(np.polyfit(t200_pwm_value[(t200_mid + 9):], t200_current_12v[(t200_mid + 9):], 2))

    def get_us(self, thrust_kgf):
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

    def get_current(self, us):
        if us < 1500 - 4 * 9:
            return T200Thruster.t200_current_left(us)
        elif us > 1500 + 4 * 9:
            return T200Thruster.t200_current_right(us)
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



# TODO: Redefine these parameters once robot is ready
THRUSTER_ANGLE_DEG = 45
# THRUSTER_ANGLE_DEG = math.acos(.875) * 180/np.pi # Sanity check angle
thruster_angles = np.array(
    [THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG]) * np.pi / 180
THRUSTER_LENGTH_DISTANCE_M = 0.5842
THRUSTER_WIDTH_DISTANCE_M = 0.4826
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
# Logistic boolean input: input(t) = 1 / (1 + e^(-8 * (t - .5)))

desired = np.array([0, 0, 3])
outputs = horizontal_factor @ desired
print(outputs)
check = horizontal_thruster_config @ outputs

print(f'{desired}\n{check}')

t2 = T200Thruster()
t1 = T100Thruster()

print(f't1.get_us(0) = {t1.get_us(0)}')
print(f't2.get_us(0) = {t2.get_us(0)}')
print(f't1.get_us(1) = {t1.get_us(1)}')
print(f't2.get_us(1) = {t2.get_us(1)}')
print(f't1.get_us(-1) = {t1.get_us(-1)}')
print(f't2.get_us(-1) = {t2.get_us(-1)}')



amps = 3.
current_us = t2.get_us_from_current(amps, True)
current_us_2 = t2.get_us_from_current(amps, False)
currents = []
x_values = []
for i in np.linspace(0, 17, 1000):
    currents.append(i)
    x_values.append(t2.get_us_from_current(i, False))
for i in np.linspace(0, 17, 1000):
    currents.append(i)
    x_values.append(t2.get_us_from_current(i, True))

plt.scatter(x_values, currents, marker='x', s=10)
# plt.plot(t200_pwm_value, t100_thrust_12v)
# plt.title('T100 Current')
# plt.plot(T100Thruster.t100_pwm_value, T100Thruster.t100_current_12v, label="Current")
# plt.plot(T100Thruster.t100_pwm_value, [T100Thruster.t100_current_left(x) for x in T100Thruster.t100_pwm_value], label='Left')
# plt.plot(T100Thruster.t100_pwm_value, [T100Thruster.t100_current_right(x) for x in T100Thruster.t100_pwm_value], label='Right')
# plt.plot(T100Thruster.t100_pwm_value, [T100Thruster.t100_current_full(x) for x in T100Thruster.t100_pwm_value], label='Full')

plt.title('T200 Current')
plt.plot(T200Thruster.t200_pwm_value, T200Thruster.t200_current_12v, label='Current')
plt.plot(T200Thruster.t200_pwm_value, [T200Thruster.t200_current_full(x) for x in T200Thruster.t200_pwm_value], label='Fit')

plt.legend()
plt.show()

