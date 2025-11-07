import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def three_dof(x, y, l1, l2, l3):
    psi = math.radians(0)  # Assuming end-effector orientation is 0 degrees
    
    wrist_x = x - l3 * math.cos(psi)
    wrist_y = y - l3 * math.sin(psi)
    
    d = math.sqrt(wrist_x**2 + wrist_y**2)
    
    if d > l1 + l2 or d < abs(l1 - l2):
        return None
    
    cos_theta2 = (wrist_x**2 + wrist_y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = max(min(cos_theta2, 1), -1)
    theta2 = math.acos(cos_theta2)
    
    theta1 = math.atan2(wrist_y, wrist_x) - math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    
    theta3 = psi - theta1 - theta2
    
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)
    theta3_deg = math.degrees(theta3)

    return theta1_deg, theta2_deg, theta3_deg

def inverse_kinematics(x, y, z, l0, l1, l2, l3):
    r = math.sqrt(x**2 + y**2)
    deg1 = math.degrees(math.atan2(y, x))
    result = three_dof(r, z - l0, l1, l2, l3)  # Adjust z by subtracting l0
    if result is None:
        return None
    deg2, deg3, deg4 = result
    return deg1, deg2, deg3, deg4

if __name__ == "__main__":
    l0 = 2 
    x, y, z = 10, 8, 8
    l1, l2, l3 = 7, 10, 3
    result = inverse_kinematics(x, y, z, l0, l1, l2, l3)
    if result:
        deg1, deg2, deg3, deg4 = result
        print(f"Deg1 (Base): {deg1:.2f}°, Deg2: {deg2:.2f}°, Deg3: {deg3:.2f}°, Deg4: {deg4:.2f}°")
    else:
        print("Target unreachable")
