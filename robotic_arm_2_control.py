#!/usr/bin/env python3
import time
import math
import numpy as np
from dynamixel_control import DynamixelController
Dy = DynamixelController()
class RoboticController:

    def __init__(self):
        self.port_handler = None
        self.packet_handler = None

    def open_robotic_arm(self, com, id_list):
        Dy.open(com)
        for i in id_list:
            Dy.torque_enable(i)

    def calulate_to_xyza(self, goal_x, goal_y, goal_z, horizontal_angle):
        horizontal_angle = horizontal_angle / 180 * np.pi
        p = self.calculate_relative_coordinate(goal_x, goal_y, goal_z, horizontal_angle)
        l = self.calculate_angle(p[0], p[1], p[2], goal_x, goal_y, goal_z)

        if l is None:
            return None

        a = self.calculate_auxiliary_angle(goal_x, goal_y, goal_z, p[0], p[1], p[2], l[6], l[7], l[8])

        res = [l[0], l[1], l[2], -a]
        print(res)
        return res

    def calculate_auxiliary_angle(self, x0, y0, z0, x1, y1, z1, x2, y2, z2):
        L1 = math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2 + (z0 - z1) ** 2)
        L2 = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

        k = (x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1) + (z0 - z1) * (z2 - z1)
        f = np.pi - math.acos(k / (L1 * L2))
        m = x1 - x2
        n = y1 - y2
        p = z1 - z2

        if m == 0 and n == 0:
            if p < 0:
                if y0 > y1:
                    f = -f
            else:
                if y0 < y1:
                    f = -f
            return f / np.pi * 180

        if m != 0:
            l = (x0 - x2) / m
        else:
            l = (y0 - y2) / n
        standard_z = l * p + z2

        if y2 - y1 > 0:
            if z0 < standard_z:
                f = -f
        else:
            if z0 > standard_z:
                f = -f

        return f / np.pi * 180
    def calculate_relative_coordinate(self, goal_x, goal_y, goal_z, angle1):
        L = 180

        if goal_x == 0:
            angle0 = 0
        elif goal_y == 0:
            if goal_x > 0:
                angle0 = np.pi / 2
            else:
                angle0 = -np.pi / 2
        else:
            angle0 = math.atan(goal_x / goal_y)

        nx = goal_x - L * math.cos(angle1) * math.sin(angle0)
        ny = goal_y - L * math.cos(angle0) * math.cos(angle1)
        nz = L * math.sin(angle1) + goal_z

        if abs(nx) < 0.001:
            nx = 0
        if abs(ny) < 0.001:
            ny = 0
        if abs(nz) < 0.001:
            nz = 0

        res = [nx, ny, nz]
        print(res)
        return res

    def calculate_angle(self, goal_position_x, goal_position_y, goal_position_z, original_x, original_y, original_z):
        L1 = 248.25
        L2 = 18.5
        L3 = 193.25
        L4 = math.sqrt(goal_position_x ** 2 + goal_position_y ** 2 + goal_position_z ** 2)

        if L4 > L1 + L3:
            return None

        if L4 < 80:
            return None

        if original_x == 0:
            sita = 0
        elif original_y == 0:
            sita = math.pi / 2 * abs(original_x) / original_x
        else:
            sita = math.atan(original_x/original_y)

        t = (L1 ** 2 + L2 ** 2 + L4 ** 2 - L3 ** 2) / (2 * math.sqrt(L1 ** 2 + L2 ** 2) * L4)

        if goal_position_y == 0:
            if original_x > 0:
                alpha = - math.acos(t) - math.atan(L2 / L1)
            else:
                alpha = math.acos(t) + math.atan(L2 / L1)
        else:
            gamma = math.atan(goal_position_z / math.sqrt(goal_position_x ** 2 + goal_position_y ** 2) * abs(goal_position_y) / goal_position_y)
            if goal_position_y > 0 and original_y >= 0:
                alpha = math.pi / 2 - math.acos(t) - math.atan(L2 / L1) - gamma
            elif goal_position_y < 0 and original_y > 0:
                alpha = -math.pi / 2 - math.acos(t) - math.atan(L2 / L1) - gamma
            elif goal_position_y > 0 and original_y <= 0:
                alpha = math.pi / 2 + math.acos(t) + math.atan(L2 / L1) - gamma
            elif goal_position_y < 0 and original_y < 0:
                alpha = -math.pi / 2 + math.acos(t) + math.atan(L2 / L1) - gamma

        xa = L1 * math.sin(alpha) * math.sin(sita)
        ya = L1 * math.sin(alpha) * math.cos(sita)
        za = L1 * math.cos(alpha)

        xb = xa + L2 * math.cos(alpha) * math.sin(sita)
        yb = ya + L2 * math.cos(alpha) * math.cos(sita)
        zb = za - L2 * math.sin(alpha)

        if abs(xa) < 0.001:
            xa = 0
        if abs(ya) < 0.001:
            ya = 0
        if abs(za) < 0.001:
            za = 0
        if abs(xb) < 0.001:
            xb = 0
        if abs(yb) < 0.001:
            yb = 0
        if abs(zb) < 0.001:
            zb = 0

        print(xa, ya, za, xb, yb, zb)
        k = (xa - xb) * (goal_position_x - xb) + (ya - yb) * (goal_position_y - yb) + (za - zb) * (goal_position_z - zb)
        print("k = ", k / L2 / L3)
        ######

        beta = math.pi - math.acos(k / (L2 * L3))

        m = xb - xa
        n = yb - ya
        p = zb - za

        if m == 0 and n == 0:
            if p > 0:
                if goal_position_y < yb:
                    beta = -beta
            else:
                if goal_position_y > yb:
                    beta = -beta
        else:
            if n != 0:
                l = (goal_position_y - yb) / n
            else:
                l = (goal_position_x - xb) / m
            standard_z = l * p + zb

            if n > 0:
                if goal_position_z > standard_z:
                    beta = -beta
            else:
                if goal_position_z < standard_z:
                    beta = -beta

        result = [sita / np.pi * 180, alpha / np.pi * 180, beta / np.pi * 180, xa, ya, za, xb, yb, zb]
        return result

    def calculate_vel(self, ang, id_list):
        p = []
        for i in id_list:
            if i == 0 or i == 14:
                continue
            p.append(Dy.present_position(i))

        m = 0
        for i in range(len(p)):
            m = max(m, abs(ang[i] - p[i]))

        vel_percent = []
        for i in range(len((p))):
            vel_percent.append(abs(ang[i] - p[i]) / m)

        velocity = []
        v_max = 20
        for i in range(len(vel_percent)):
            k = int(v_max * vel_percent[i])
            velocity.append(k)

        return velocity

    def turn_clip(self, id, alpha):

        if alpha < -90 or alpha > 90:
            print("fail to turn the clip")
            return None

        Dy.goal_absolute_direction(id, alpha)

    def open_close_clip(self, id, alpha):
        angle_max = 90
        angle_min = -8
        if alpha > 90 or alpha < -8:
            print("fail to open or close the clip")
            return None

        Dy.goal_absolute_direction(id, alpha)

    def go_to_real_xyz_alpha(self, id_list, goal_position_list, horizontal_angle, turn_clip_angle, open_clip_angle, state):
        if state == 0:
            turn_clip_angle = -turn_clip_angle

        px = goal_position_list[0]
        py = goal_position_list[1]
        pz = goal_position_list[2]

        data = self.calulate_to_xyza(px, py, pz, horizontal_angle)

        if data is None:
            print("data is None")
            print("fail to get to point")
            return False

        ang = [data[0], data[1], data[2], data[3], turn_clip_angle, open_clip_angle]

        v = self.calculate_vel(ang, id_list)

        print("ang = ", ang)
        print("vel = ", v)

        vel = [v[0], v[1], v[1], v[2], v[2], v[3], v[4], v[5]]
        for i in range(len(id_list)):
            if vel[i] == 0:
                continue
            Dy.profile_velocity(id_list[i], vel[i])

        present_angle = []
        for i in id_list:
            present_angle.append(Dy.present_position(i))

        target_angle = [ang[0], ang[1], -ang[1], ang[2], -ang[2], ang[3], ang[4], ang[5]]

        for i in range(len(id_list)):
            if abs(present_angle[i] - target_angle[i]) < 2:
                continue

            if i <= 5:
                Dy.goal_absolute_direction(id_list[i], target_angle[i])
            elif i == 6:
                self.turn_clip(id_list[i], target_angle[i])
            elif i == 7:
                self.open_close_clip(id_list[i], target_angle[i])

        time.sleep(5)
