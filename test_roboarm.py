from robotic_arm_2_control import RoboticController

Ro = RoboticController()
id_list = [11, 12, 0, 15, 14, 13, 1, 2]
Ro.open_robotic_arm("COM4", id_list)
Ro.go_to_real_xyz_alpha(id_list, [0, 100, 250], 0, 0, 10, 0)