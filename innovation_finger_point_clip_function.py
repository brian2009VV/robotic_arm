import time
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from dynamixel_control import DynamixelController
from robotic_arm_control import RoboticController

Dy = DynamixelController()
Ro = RoboticController()

model = YOLO('yolov8m-seg.pt')
cam = cv2.VideoCapture(1)
id_list = [11, 13, 15, 14, 12, 1, 2]
Ro.open_robotic_arm("COM4", id_list, Dy)

######################################################################################

pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

pipe.start(cfg)

img = None
pose_point = []
while True:
    frame = pipe.wait_for_frames()
    dp = frame.get_depth_frame()
    cf = frame.get_color_frame()

    dp = np.asanyarray(dp.get_data())
    cf = np.asanyarray(cf.get_data())

    pose_point = Ro.yolo_human_pos_direction(cf, dp)

    if pose_point is not None:
        print("human pose = ", pose_point)
        img = cf.copy()
        cv2.destroyAllWindows()
        break

    cv2.imshow("rgb", cf)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#########################################################################################

cv2.circle(img, (pose_point[2][0], pose_point[2][1]), 10, (0, 255, 0), -1)
cv2.circle(img, (pose_point[2][3], pose_point[2][4]), 10, (0, 0, 255), -1)
cv2.imshow("human_pose", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

###########################################################################################

Ro.go_to_real_xyz_alpha(id_list, [0, 200, 150], 82, 0, 0, 0, Dy)

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
    res = Ro.yolo_seg_detection(model, "bottle", overlay, 2)

    if res is not None:
        for i in range(len(res)):
            t = res[i]
            x = int(t[0][0])
            y = int(t[0][1])

            center_x, center_y, center_z = Ro.get_real_xyz_armcam(-355, x, y, [0, 200, 150], overlay)

            boxes_point = t[3]
            real_box_point = []

            for j in boxes_point:
                j_x, j_y, j_z = Ro.get_real_xyz_armcam(-355, j[0], j[1], [0, 200, 150], overlay)
                real_box_point.append([j_x, j_y, j_x])

            turn_clip_angle, state = Ro.calulate_box_to_angle_vertical_version(real_box_point, [center_x, center_y, center_z], 90)
            xyz_list.append([[center_x, center_y, center_z], turn_clip_angle, state])


    if len(xyz_list) != 0:
        cv2.destroyAllWindows()
        print("bottle_xyz_list = ", xyz_list)
        break

    cv2.imshow('Camera', overlay)
    if cv2.waitKey(1) == ord('q'):
        break

d = []
for i in xyz_list:
    t = Ro.get_distance(i[0][0], i[0][1], i[0][2], pose_point[0][0], pose_point[0][1], pose_point[0][2], pose_point[1][0], pose_point[1][1], pose_point[1][2])
    d.append([t, i[0][0], i[0][1], i[0][2], i[1], i[2]])
print("dis = ", d)

m = [99999, 0, 0, 0, 0, 0]
for i in d:
    if i[0] < m[0]:
        m = i

Ro.go_to_real_xyz_alpha(id_list, [0, 200, 150], 82, 0, 90, 0, Dy)
Ro.go_to_real_xyz_alpha(id_list, [m[1], m[2], m[3] + 150], 82, m[4], 90, m[5], Dy)
Ro.go_to_real_xyz_alpha(id_list, [m[1], m[2], m[3] + 60], 82, m[4], 90, m[5], Dy)
Ro.go_to_real_xyz_alpha(id_list, [m[1], m[2], m[3] + 60], 82, m[4], 30, m[5], Dy)
Ro.go_to_real_xyz_alpha(id_list, [0, 100, 350], 0, 0, 30, 0, Dy)
Ro.go_to_real_xyz_alpha(id_list, [-100, 50, 350], 0, 0, 30, 0, Dy)
Ro.go_to_real_xyz_alpha(id_list, [-100, 50, 350], 0, 0, 90, 0, Dy)
