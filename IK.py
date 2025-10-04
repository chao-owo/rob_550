from math import atan2, acos, sqrt, sin, cos, pi
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

if __name__ == "__main__":
    target_x = 0
    target_y = 408.575
    target_z = 304.57
    target_roll = pi / 2

    print("\nElbow-UP solution:")
    solution_up = analytical_IK(target_x, target_y, target_z, roll=target_roll, elbow_up=True)
    print(solution_up)


