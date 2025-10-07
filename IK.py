from math import atan2, acos, sqrt, sin, cos, pi
self.block_info = []
def analytical_IK(x, y, z, roll = pi/2,elbow_up=True):
    l1 = 205.9
    l2 = 200
    l3 = 150

    base_height = 104
    offset_angle = acos((200 ** 2 + l1 ** 2 - 50 ** 2) / (2 * 200 * l1)) * 180 / pi


    theta1 = atan2(x, y) * 180 / pi
    z_off = z - base_height
    y_off = y-l3
    theta3 = acos((y_off ** 2 + z_off ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)) * 180 / pi

    r = sqrt(y_off ** 2 + z_off ** 2)

    r_wrist = r - l3 * cos(roll)
    z_wrist = z - base_height - l3 * sin(roll)

    # Distance from joint 2 to wrist
    d_to_wrist = sqrt(r_wrist ** 2 + z_wrist ** 2)

    print(f"Distance to wrist: {d_to_wrist:.2f}")

    if d_to_wrist > (l1 + l2):
        print(f"too far! Distance={d_to_wrist:.2f}, Max reach={l1 + l2}")
        return None
    if d_to_wrist < abs(l1 - l2):
        print(f"too close! Distance={d_to_wrist:.2f}, Min reach={abs(l1 - l2)}")
        return None
    if elbow_up:
        theta3 = theta3 + offset_angle
    else:
        theta3 = -theta3

    theta2 = (atan2(z_off, y_off)-atan2(l2*sin(theta3),l1+l2*cos(theta3)))* 180 / pi
    theta2 = theta2 + offset_angle
    theta4 = roll - (90-theta2) - (90-theta3)

    result = [theta1, theta2, theta3, theta4]
    return result
def event2(self):
    # lv1
    event_color = ["red","orange","yellow", "green", "blue","purple"]
    for color in event_color:
        block_info = self.camera.block_info.copy()

        if color == "red":
            place_pos = [250, -50, 0]
        elif color == "orange":
            place_pos = [250, -50, 40]
        elif color == "yellow":
            place_pos = [250, -50, 80]
        elif color == "green":
            place_pos = [-250, -50, 0]
        elif color == "blue":
            place_pos = [-250, -50, 40]
        elif color == "purple":
            place_pos = [-250, -50, 80]
        else:
            print("Non-RGB Color Detected")
        for block in block_info:
            if block[2] == color:
                block_center = block[0]
                block_angle = block[1]

                self.pick_block(block_center, block_angle)
                self.place_block(place_pos, 0)

                continue
    target_pos = [-250, -50, 120]
    target_world = target_pos.copy()
    target_angle_rad = np.pi * target_angle / 180.0

    target_world[2] += 50
    offset_target = target_world.copy()
    offset_target[2] += 40

    if offset_target[2] >= 180:
        orientation_target = [0, 0, clamp(math.atan2(target_world[1], target_world[0]) - np.pi / 2)]
        target_world[2] -= 30

        offset_target[0] *= 0.95
        offset_target[1] *= 0.95

        target_world[0] *= 0.95
        target_world[1] *= 0.95
    else:
        orientation_target = [-1.52, target_angle_rad, -1 * target_angle_rad]

    theta_hat_offset = IK_numerical(offset_target, orientation_target, [0, 0, 0, 0, 0])
    self.rxarm.set_positions(theta_hat_offset)

    self.rxarm.set_positions([0, 0, 0, 0, 0])

    place_order = ["purple","blue","green","yellow","orange","red"]
    for color in place_order:
        block_info = self.camera.block_info.copy()
        if color == "red":
            place_pos = [100, 225,0]
        elif color == "orange":
            place_pos = [50, 225,0]
        elif color == "yellow":
            place_pos = [0, 225,0]
        elif color == "green":
            place_pos = [-50, 225,0]
        elif color == "blue":
            place_pos = [-100, 225,0]
        elif color == "purple":
            place_pos = [-150, 225,0]
        else:
            print("Non-RGB Color Detected")
        for block in block_info:
            if block[2] == color:
                block_center = block[0]
                block_angle = block[1]

                self.pick_block(block_center, block_angle)
                self.place_block(place_pos, 90)
                continue




if __name__ == "__main__":
    target_x = 0
    target_y = 408.575
    target_z = 304.57
    target_roll = pi / 2

    print("\nElbow-UP solution:")
    solution_up = analytical_IK(target_x, target_y, target_z, roll=target_roll, elbow_up=True)
    print(solution_up)


