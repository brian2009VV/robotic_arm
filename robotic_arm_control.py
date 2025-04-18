#!/usr/bin/env python3
import time
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
            if goal_position_y >= 0:
                alpha = math.pi / 2 - math.atan(L2 / L1) - math.acos(t) - math.atan(goal_position_z / math.sqrt(goal_position_x ** 2 + goal_position_y ** 2))
            else:
                alpha = math.pi / 2 - math.atan(L2 / L1) - math.acos(t) - (np.pi - math.atan(goal_position_z / math.sqrt(goal_position_x ** 2 + goal_position_y ** 2)))


        xa = L1 * math.sin(alpha) * math.sin(sita)
        ya = L1 * math.sin(alpha) * math.cos(sita)
        za = L1 * math.cos(alpha)

        xb = xa + L2 * math.cos(alpha) * math.sin(sita)
        yb = ya + L2 * math.cos(alpha) * math.cos(sita)
        zb = za - L2 * math.sin(alpha)

        k = (xa - xb) * (goal_position_x - xb) + (ya - yb) * (goal_position_y - yb) + (za - zb) * (goal_position_z - zb)
        print(k / (L2 * L3))
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

    def calulate_to_xyza(self, goal_x, goal_y, goal_z, horizontal_angle):
        horizontal_angle = horizontal_angle / 180 * np.pi
        p = self.calculate_relative_coordinate(goal_x, goal_y, goal_z, horizontal_angle)
        print(p[0], p[1], p[2])
        l = self.calculate_angle(p[0], p[1], p[2])
        print(l)

        if l is None:
            return None

        a = self.calculate_auxiliary_angle(goal_x, goal_y, goal_z, p[0], p[1], p[2], l[6], l[7], l[8])

        res = [l[0], l[1], l[2], a]
        return res

    def open_close_clip(self, id, alpha, Dy):
        angle_max = 90
        angle_min = -8
        if alpha > 90 or alpha < -8:
            return None

        Dy.goal_absolute_direction(id, alpha)

    def turn_clip(self, id, alpha, Dy):
        if alpha < -90 or alpha > 90:
            return None

        Dy.goal_absolute_direction(id, alpha)

    def calculate_vel(self, ang, id_list, Dy):
        p = []
        for i in id_list:
            if i == 15:
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

            if k == 0:
                k = 1

            velocity.append(k)

        return velocity

    def open_robotic_arm(self, com, id_list, Dy):
        Dy.open(com)
        for i in id_list:
            Dy.torque_enable(i)

    def go_to_real_xyz_alpha(self, id_list, goal_position_list, horizontal_angle, turn_clip_angle, open_clip_angle, state, Dy):
        if state == 0:
            turn_clip_angle = -turn_clip_angle

        px = goal_position_list[0]
        py = goal_position_list[1]
        pz = goal_position_list[2]

        data = self.calulate_to_xyza(px, py, pz, horizontal_angle)

        if data is None:
            return None

        ang = [data[0], data[1], data[2], data[3], turn_clip_angle, open_clip_angle]

        vel = self.calculate_vel(ang, id_list, Dy)

        print("ang = ", ang)
        print("vel = ", vel)

        Dy.profile_velocity(id_list[0], vel[0])
        Dy.profile_velocity(id_list[1], vel[1])
        Dy.profile_velocity(id_list[2], vel[1])
        Dy.profile_velocity(id_list[3], vel[2])
        Dy.profile_velocity(id_list[4], vel[3])
        Dy.profile_velocity(id_list[5], vel[4])
        Dy.profile_velocity(id_list[6], vel[5])

        Dy.goal_absolute_direction(id_list[0], ang[0])
        Dy.goal_absolute_direction(id_list[1], ang[1])
        Dy.goal_absolute_direction(id_list[2], -ang[1])
        Dy.goal_absolute_direction(id_list[3], ang[2])
        Dy.goal_absolute_direction(id_list[4], ang[3])
        self.turn_clip(id_list[5], ang[4], Dy)
        self.open_close_clip(id_list[6], ang[5], Dy)
        time.sleep(5)

