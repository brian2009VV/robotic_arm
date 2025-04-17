#!/usr/bin/env python3
import math
import numpy as np

class RoboticController:

    def __init__(self):
        self.port_handler = None
        self.packet_handler = None

    def calculate_angle(self, goal_position_x, goal_position_y, goal_position_z):
        L1 = 248.25
        L2 = 18.5
        L3 = 193.25
        L4 = math.sqrt(goal_position_x ** 2 + goal_position_y ** 2 + goal_position_z ** 2)

        if L4 > L1 + L3:
            return None

        if goal_position_x == 0:
            sita = 0
        elif goal_position_y == 0:
            sita = pi / 2
        else:
            sita = math.atan(goal_position_x/goal_position_y)

        t = (L1 ** 2 + L2 ** 2 + L4 ** 2 - L3 ** 2) / (2 * math.sqrt(L1 ** 2 + L2 ** 2) * L4)

        if goal_position_x ** 2 + goal_position_y ** 2 == 0:
            alpha =  math.pi / 2 - math.atan(L2/L1) - math.acos(t) - math.pi / 2
        else:
            alpha = math.pi / 2 - math.atan(L2/L1) - math.acos(t) - math.atan(goal_position_z/math.sqrt(goal_position_x ** 2 + goal_position_y ** 2))

        xa = L1 * math.sin(alpha) * math.sin(sita)
        ya = L1 * math.sin(alpha) * math.cos(sita)
        za = L1 * math.cos(alpha)

        xb = xa + L2 * math.cos(alpha) * math.sin(sita)
        yb = ya + L2 * math.cos(alpha) * math.cos(sita)
        zb = za - L2 * math.sin(alpha)

        k = (xa - xb) * (goal_position_x - xb) + (ya - yb) * (goal_position_y - yb) + (za - zb) * (goal_position_z - zb)

        beta = math.pi - math.acos(k / (L2 * L3))

        m = xb - xa
        n = yb - ya
        p = zb - za

        if n != 0:
            l = (goal_position_y - ya) / n
        else:
            l = (goal_position_x - xa) / m
        standard_z = l * p + za

        if goal_position_z > standard_z:
            beta = -beta
        else:
            beta = beta

        result = [sita / np.pi * 180, alpha / np.pi * 180, beta / np.pi * 180, xa, ya, za, xb, yb, zb]
        return result

    def calculate_relative_coordinate(self, goal_x, goal_y, goal_z, angle1):
        L = 180

        if goal_x == 0:
            angle0 = 0
        elif goal_y == 0:
            angle0 = np.pi / 2
        else:
            angle0 = math.atan(goal_x/ goal_y)

        nx = goal_x - L * math.cos(angle1) * math.sin(angle0)
        ny = goal_y - L * math.cos(angle0) * math.cos(angle1)
        nz = L * math.sin(angle1) + goal_z

        res = [nx, ny, nz]
        return res

    def calculate_auxiliary_angle(self, x0, y0, z0, x1, y1, z1, x2, y2, z2):
        L1 = math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2 + (z0 - z1) ** 2)
        L2 = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

        k = (x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1) + (z0 - z1) * (z2 - z1)
        f = np.pi - math.acos(k / (L1 * L2))

        m = x1 - x2
        n = y1 - y2
        p = z1 - z2
        if n != 0:
            l = (y0 - y2) / n
        else:
            l = (x0 - x2) / m
        standard_z = l * p + z2

        if z0 > standard_z:
            f = -f
        else:
            f = f
        return f / np.pi * 180

    def get_to_real(self, goal_x, goal_y, goal_z, horizontal_angle):
        p = self.calculate_relative_coordinate(goal_x, goal_y, goal_z, horizontal_angle)
        print(p[0], p[1], p[2])
        l = self.calculate_angle(p[0], p[1], p[2])
        print(l)

        if l is None:
            return None

        a = self.calculate_auxiliary_angle(goal_x, goal_y, goal_z, p[0], p[1], p[2], l[6], l[7], l[8])

        res = [l[0], l[1], l[2], a]
        return res