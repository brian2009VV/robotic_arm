import time
from dynamixel_control import DynamixelController
from robotic_arm_control import RoboticController
import cv2
import numpy as np

Dy = DynamixelController()
Ro = RoboticController()
cam = cv2.VideoCapture(1)
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

id_list = [11, 13, 15, 14, 12, 1, 2]
Ro.open_robotic_arm("COM4", id_list, Dy)
Ro.go_to_real_xyz_alpha(id_list, [0, 200, 150], 85, 0, 0, 0, Dy) #horizontal_angle, trun_clip_angle, open_or_close, state

xyz_list = []
state = -1
turn_clip_angle = -1

while True:
    ret, frame = cam.read()
    frame = np.array(frame)
    frame = np.rot90(frame, 2)

    if not ret:
        continue

    overlay = frame.copy()
    res = Ro.yolo_detection('yolov8m-seg.pt', "bottle", overlay)

    if res is not None:
        for i in range(len(res)):
            t = res[i]
            x = int(t[0][0])
            y = int(t[0][1])
            cv2.circle(overlay, (x, y), 10, (0, 255, 0), -1)
            angle = int(t[2])

            l1 = (t[3][3][0] - t[3][2][0]) ** 2 + (t[3][3][1] - t[3][2][1]) ** 2
            l2 = (t[3][3][0] - t[3][0][0]) ** 2 + (t[3][3][1] - t[3][0][1]) ** 2

            if l1 > l2:
                turn_clip_angle = angle
            else:
                turn_clip_angle = - (90 - angle)

            if turn_clip_angle > 0:
                state = 0
            else:
                state = 1

            turn_clip_angle = abs(turn_clip_angle)

            xyz_list.append(Ro.get_real_xyz(-120, x, y, [0, 200, 150], overlay))

    if len(xyz_list) != 0:
        print(xyz_list)
        break

    cv2.imshow('Camera', overlay)

    if cv2.waitKey(1) == ord('q'):
        break

Ro.go_to_real_xyz_alpha(id_list, [0, 200, 150], 85, 0, 90, 0, Dy)
Ro.go_to_real_xyz_alpha(id_list, [xyz_list[0][0], xyz_list[0][1], xyz_list[0][2] + 80], 85, turn_clip_angle, 90, state, Dy)
Ro.go_to_real_xyz_alpha(id_list, [xyz_list[0][0], xyz_list[0][1], xyz_list[0][2] + 80], 85, turn_clip_angle, 30, state, Dy)
Ro.go_to_real_xyz_alpha(id_list, [0, 100, 350], 0, 0, 30, 0, Dy)
Ro.go_to_real_xyz_alpha(id_list, [-100, 50, 350], 0, 0, 30, 0, Dy)
Ro.go_to_real_xyz_alpha(id_list, [-100, 50, 350], 0, 0, 90, 0, Dy)
