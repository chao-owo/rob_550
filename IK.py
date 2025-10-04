import numpy as np
from math import atan2, acos, sqrt, sin, cos, pi

def analytical_IK(x, y, z, roll = pi/2,elbow_up=True):
    l1 = 205.73
    l2 = 200
    l3 = 151

    base_height = 104

    theta1 = atan2(y, x) * 180 / pi
    theta3 = acos((x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / 2 * l1 * l2) * 180 / pi

    r = sqrt(x ** 2 + y ** 2)

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
        theta3 = theta3
    else:
        theta3 = -theta3

    theta2 = (atan2(y, x)-atan2(l2*sin(theta3),l1+l2*cos(theta3)))* 180 / pi
    offset_angle =acos((200**2+ 205.73**2- 50**2)/(2*200*205.73))*180/pi
    theta2 = theta2-offset_angle
    theta4 = (roll - theta2 - theta3)* 180 / pi

    result = [theta1, theta2, theta3, theta4]
    return result

if __name__ == "__main__":
    target_x = 200
    target_y = 200
    target_z = 300
    target_roll = pi / 2

    print("\nElbow-UP solution:")
    solution_up = analytical_IK(target_x, target_y, target_z, roll=target_roll, elbow_up=True)
    print(solution_up)

    print("\nElbow-DOWN solution:")
    solution_down = analytical_IK(target_x, target_y, target_z, roll=target_roll, elbow_up=False)
    print(solution_down)

