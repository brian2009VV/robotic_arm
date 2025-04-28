#!/usr/bin/env python3
import time
import math
import numpy as np
from ultralytics import YOLO
import cv2

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

        if L4 < 80:
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

        if n > 0:
            if z0 > standard_z:
                f = -f
        else:
            if z0 < standard_z:
                f = -f
        return f / np.pi * 180

    def check_angle(self, angle):
        a0 = angle[0]
        a1 = angle[1]
        a2 = angle[2]
        a3 = angle[3]

        if abs(a0) > 90:
            return None

        if a1 < -100 or a1 > 120:
            return None

        if a2 > 85 or a2 < -90:
            return None

        if a3 < -90 or a3 > 110:
            return None

        return 1


    def calulate_to_xyza(self, goal_x, goal_y, goal_z, horizontal_angle):
        horizontal_angle = horizontal_angle / 180 * np.pi
        p = self.calculate_relative_coordinate(goal_x, goal_y, goal_z, horizontal_angle)
        l = self.calculate_angle(p[0], p[1], p[2])

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
            print("fail to get to point")
            return None

        if self.check_angle(data) is None:
            print("fail to get to point")
            return None

        ang = [data[0], data[1], data[2], data[3], turn_clip_angle, open_clip_angle]

        v = self.calculate_vel(ang, id_list, Dy)

        print("ang = ", ang)
        print("vel = ", v)

        vel = [v[0], v[1], v[1], v[2], v[3], v[4], v[5]]
        for i in range(len(id_list)):
            if vel[i] == 0:
                continue
            Dy.profile_velocity(id_list[i], vel[i])


        present_angle = []
        for i in id_list:
            present_angle.append(Dy.present_position(i))

        target_angle = [ang[0], ang[1], -ang[1], ang[2], ang[3], ang[4], ang[5]]
        for i in range(len(id_list)):
            if abs(present_angle[i] - target_angle[i]) < 4:
                continue

            if i <= 4:
                Dy.goal_absolute_direction(id_list[i], target_angle[i])
            elif i == 5:
                self.turn_clip(id_list[i], target_angle[i], Dy)
            elif i == 6:
                self.open_close_clip(id_list[i], target_angle[i], Dy)
        time.sleep(5)

    def yolo_detection(self, model, name, frame):
        model = YOLO(model)
        results = model.predict(
            source=frame,
            conf=0.4,
            imgsz=640,
            verbose=False
        )
        mask_combined = np.zeros(frame.shape[:2], dtype=np.uint8)
        rotated_boxes = []

        for result in results:
            if result.masks is not None:
                for i, mask in enumerate(result.masks.data):
                    class_id = int(result.boxes.cls[i])
                    if model.names[class_id] == name:
                        binary_mask = (mask.cpu().numpy() * 255).astype(np.uint8)
                        binary_mask = cv2.resize(binary_mask, (frame.shape[1], frame.shape[0]))
                        mask_combined = cv2.bitwise_or(mask_combined, binary_mask)

        if np.max(mask_combined) > 0:
            contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 2000]

            for cnt in valid_contours:
                rotated_rect = cv2.minAreaRect(cnt)
                box_points = cv2.boxPoints(rotated_rect)
                box_points = np.int32(box_points)
                rotated_boxes.append(box_points)

        res = []
        if rotated_boxes is None:
            return None
        else:
            for box in rotated_boxes:
                center, size, angle = cv2.minAreaRect(box)
                res.append([center, size, angle, box])
            return res

    def get_real_xyz(self, depth, x, y, posi, frame):
        L = (posi[0] ** 2 + posi[1] ** 2) ** 0.5
        L = L + 35
        if posi[0] == 0:
            angle = np.pi / 2
        else:
            angle = math.atan(posi[1]/posi[0])

        posi[0] = L * math.cos(angle)
        posi[1] = L * math.sin(angle)

        if x < 0 or y < 0:
            return 0, 0, 0

        x = int(x)
        y = int(y)
        a = 48 * np.pi / 180
        b = 62.85 * np.pi / 180
        d = int(depth)
        h, w = frame.shape[:2]
        h = int(h)
        w = int(w)

        x = x - w // 2
        y = y - h // 2
        real_y = int(y) * 2 * int(d) * np.tan(a / 2) / int(h)
        real_x = int(x) * 2 * int(d) * np.tan(b / 2) / int(w)

        real_x = -real_x

        print("real = ", real_x, real_y)
        real_z = depth

        real_x = real_x + posi[0]
        real_y = real_y + posi[1]

        return real_x, real_y, real_z



