from dynamixel_control import DynamixelController
from robotic_arm_control import RoboticController

Dy = DynamixelController()
Ro = RoboticController()

id_list = [11, 13, 15, 14, 12, 1, 2]
pos = list(input("input x, y, z = ").split(" "))
a0 = int(input("horizontal_angle = "))
a2 = int(input("open_or_close = "))
a1 = abs(int(input("trun_clip_angle = ")))
state = int(input("clockwise or counterclockwise (0 = clockwise , 1 = counterclockwise) = "))

for i in range(len(pos)):
    pos[i] = int(pos[i])

Ro.open_robotic_arm("COM4", id_list, Dy)
Ro.go_to_real_xyz_alpha(id_list, pos, a0, a1, a2, state, Dy)


