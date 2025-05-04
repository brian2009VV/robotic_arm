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
            sita = pi / 2 * abs(goal_position_x) / goal_position_x
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

        if a3 < -90 or a3 > 115:
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
            print("fail to open or close the clip")
            return None

        Dy.goal_absolute_direction(id, alpha)

    def turn_clip(self, id, alpha, Dy):
        if alpha < -90 or alpha > 90:
            print("fail to turn the clip")
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

    def get_double_rotate_coordinate(self, px, py, pz, alpha, beta):
        alpha = -alpha * np.pi / 180
        beta = -beta * np.pi / 180

        if px == 0:
            theta = 0
        elif py == 0:
            theta = math.pi / 2 * abs(px) / px
        else:
            theta = math.atan(px / py)

        L = (px ** 2 + py ** 2 + pz ** 2) ** 0.5

        ax = L * math.sin(alpha + theta)
        ay = L * math.cos(alpha + theta)
        az = pz

        if az == 0:
            sita = 0
        elif ay == 0:
            sita = math.pi / 2 * abs(az) / az
        else:
            sita = math.atan(az / ay)

        bz = L * math.sin(sita + beta)
        by = L * math.cos(sita + beta)
        bx = ax

        return [bx, by, bz]

    def go_to_real_xyz_alpha(self, id_list, goal_position_list, horizontal_angle, turn_clip_angle, open_clip_angle, state, Dy):
        if state == 0:
            turn_clip_angle = -turn_clip_angle

        px = goal_position_list[0]
        py = goal_position_list[1]
        pz = goal_position_list[2]

        data = self.calulate_to_xyza(px, py, pz, horizontal_angle)

        if data is None:
            print("data is None")
            print("fail to get to point")
            return None

        if self.check_angle(data) is None:
            print("angle is fail")
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
            if abs(present_angle[i] - target_angle[i]) < 3:
                continue

            if i <= 4:
                Dy.goal_absolute_direction(id_list[i], target_angle[i])
            elif i == 5:
                self.turn_clip(id_list[i], target_angle[i], Dy)
            elif i == 6:
                self.open_close_clip(id_list[i], target_angle[i], Dy)

        time.sleep(5)

    def yolo_seg_detection(self, model, name, frame, num):
        results = model.predict(
            source=frame,
            conf=0.5,
            imgsz=640,
            verbose=False
        )
        mask_combined = np.zeros(frame.shape[:2], dtype=np.uint8)
        rotated_boxes = []

        count = 0
        for result in results:
            if result.masks is not None:
                for i, mask in enumerate(result.masks.data):
                    class_id = int(result.boxes.cls[i])
                    if model.names[class_id] == name:
                        count += 1
                        binary_mask = (mask.cpu().numpy() * 255).astype(np.uint8)
                        binary_mask = cv2.resize(binary_mask, (frame.shape[1], frame.shape[0]))
                        mask_combined = cv2.bitwise_or(mask_combined, binary_mask)

        if count != num:
            return None

        if np.max(mask_combined) > 0:
            contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1500]

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

    def get_real_xyz_armcam(self, depth, x, y, posi, frame):
        x = int(x)
        y = int(y)
        a = 48 * np.pi / 180
        b = 62 * np.pi / 180
        d = int(depth)
        h, w = frame.shape[:2]
        h = int(h)
        w = int(w)

        x = x - w // 2
        y = y - h // 2
        real_y_pi = int(y) * 2 * int(d) * np.tan(a / 2) / int(h)
        real_x_pi = int(x) * 2 * int(d) * np.tan(b / 2) / int(w)

        real_x_pi = -real_x_pi
        real_z = depth

        L = (posi[0] ** 2 + posi[1] ** 2) ** 0.5 + 30

        theta = math.atan(real_x_pi / (L + real_y_pi))

        if posi[0] == 0:
            phi = 0
        elif posi[1] == 0:
            phi = math.pi / 2 * abs(posi[0]) / posi[0]
        else:
            phi = math.atan(posi[0] / posi[1])

        l = (real_x_pi ** 2 + (real_y_pi + L) ** 2) ** 0.5

        real_x = l * math.sin(phi + theta)
        real_y = l * math.cos(phi + theta)

        return real_x, real_y, real_z

    def get_real_xyz_groundcam(self, depth, x, y, frame):
        a = 42 * np.pi / 180
        b = 69 * np.pi / 180
        d = int(depth)
        h, w = frame.shape[:2]
        x = int(x) - int(w // 2)
        y = int(y) - int(h // 2)
        real_y = round(int(y) * 2 * int(d) * np.tan(a / 2) / int(h))
        real_x = round(int(x) * 2 * int(d) * np.tan(b / 2) / int(w))
        return int(real_x), int(d + 100), int(-real_y - 80)

    def yolo_human_pos_direction(self, cf, dp):
        model = YOLO("yolo11m-pose.pt")

        point = [0, 0, 999999, 0, 0, 999999]
        xyz1 = []
        xyz2 = []

        results = model.predict(
            source=cf,
            conf=0.3,
            imgsz=640,
            verbose=False
        )

        for result in results:
            xy = result.keypoints.xy

            for i in xy:
                if len(i) < 16:
                    continue

                if i[7][0] == 0 and i[7][1] == 0:
                    continue
                if i[9][0] == 0 and i[9][1] == 0:
                    continue

                x1 = int(i[7][0])
                y1 = int(i[7][1])
                x2 = int(i[9][0])
                y2 = int(i[9][1])

                if dp[y1][x1] < point[2] and dp[y1][x1] != 0 and dp[y1][x1] < 1500:
                    point[0] = x1
                    point[1] = y1
                    point[2] = int(dp[y1][x1])

                if dp[y2][x2] < point[5] and dp[y2][x2] != 0 and dp[y2][x2] < 1500:
                    point[3] = x2
                    point[4] = y2
                    point[5] = int(dp[y2][x2])


        if point[2] != 999999 and point[5] != 999999:
            xyz1 = self.get_real_xyz_groundcam(point[2], point[0], point[1], dp)
            xyz2 = self.get_real_xyz_groundcam(point[5], point[3], point[4], dp)
            return [xyz1, xyz2, point]

        return None

    def get_distance(self, px, py, pz, ax, ay, az, bx, by, bz):
        A, B, C, p1, p2, p3, qx, qy, qz, distance = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        A = int(bx) - int(ax)
        B = int(by) - int(ay)
        C = int(bz) - int(az)
        p1 = int(A) * int(px) + int(B) * int(py) + int(C) * int(pz)
        p2 = int(A) * int(ax) + int(B) * int(ay) + int(C) * int(az)
        p3 = int(A) * int(A) + int(B) * int(B) + int(C) * int(C)
        if (p1 - p2) != 0 and p3 != 0:
            t = (int(p1) - int(p2)) / int(p3)
            qx = int(A) * int(t) + int(ax)
            qy = int(B) * int(t) + int(ay)
            qz = int(C) * int(t) + int(az)
            return int(int(pow(((int(qx) - int(px)) ** 2 + (int(qy) - int(py)) ** 2 + (int(qz) - int(pz)) ** 2), 0.5)))
        return 0

    def calulate_box_to_angle_vertical_version(self, point, center, horizontal_angle):
        goalx = center[0]
        goaly = center[1]
        goalz = center[2]

        data = self.calulate_to_xyza(goalx, goaly, goalz, horizontal_angle)

        if data is None:
            print("fail to go to point")
            return None

        a0 = data[0]
        a1 = -data[3]

        res = []
        for i in point:
            x_pi = i[0]
            y_pi = i[1]
            z_pi = i[2]
            res.append(self.get_double_rotate_coordinate(x_pi, y_pi, z_pi, a0, a1))

        ps = []
        for i in res:
            x = i[0]
            y = i[1]
            ps.append([x, y])

        ps = np.array(ps, dtype=np.float32)

        l1 = (ps[3][0] - ps[2][0]) ** 2 + (ps[3][1] - ps[2][1]) ** 2
        l2 = (ps[3][0] - ps[0][0]) ** 2 + (ps[3][1] - ps[0][1]) ** 2

        if l1 > l2:
            dx = ps[3][0] - ps[2][0]
            dy = ps[3][1] - ps[2][1]
        else:
            dx = ps[3][0] - ps[0][0]
            dy = ps[3][1] - ps[0][1]

        if dx == 0:
            turn_clip_angle = 0
        elif dy == 0:
            turn_clip_angle = 90 * abs(dx) / dx
        else:
            turn_clip_angle = math.atan(dx / dy)

        if turn_clip_angle > 0:
            state = 0
        else:
            state = 1

        turn_clip_angle = abs(turn_clip_angle) * 180 / np.pi

        return turn_clip_angle, state
