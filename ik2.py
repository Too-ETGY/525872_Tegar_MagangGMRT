import math

# source : https://www.linkedin.com/pulse/inverse-kinematics-3-degrees-freedom-william-van-bouwel/ & blackbox
def three_dof(x, y, l1, l2, l3):
    gamma = math.radians(0) # Assuming end-effector orientation is 0 degrees (pointing forward)
    
    # Calculate the position of the last servo to use in 2-DoF
    x3 = x - l3 * math.cos(gamma)
    y3 = y - l3 * math.sin(gamma)

    # Calculate the rotations of the second servo
    d = math.sqrt(x3**2 + y3**2)
    if d > l1 + l2 or d < abs(l1 - l2):
        return None
    
    # Theta2 for both configurations
    cos_theta2 = (x3**2 + y3**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = max(min(cos_theta2, 1), -1)
    theta2a = math.acos(cos_theta2)  # Elbow-down
    theta2b = -theta2a  # Elbow-up

    # Theta1 for both configurations
    theta1a = math.atan2(y3, x3) - math.atan2(l2 * math.sin(theta2a), l1 + l2 * math.cos(theta2a))
    theta1b = math.atan2(y3, x3) - math.atan2(l2 * math.sin(theta2b), l1 + l2 * math.cos(theta2b))

    # Calculate the rotations of the third servo
    theta3a = gamma - theta1a - theta2a
    theta3b = gamma - theta1b - theta2b
    
    theta1a_deg = math.degrees(theta1a)
    theta2a_deg = math.degrees(theta2a)
    theta3a_deg = math.degrees(theta3a)
    theta1b_deg = math.degrees(theta1b)
    theta2b_deg = math.degrees(theta2b)
    theta3b_deg = math.degrees(theta3b) 

    return [(theta1a_deg, theta2a_deg, theta3a_deg), (theta1b_deg, theta2b_deg, theta3b_deg)]

def inverse_kinematics(x, y, z, l1, l2, l3):
    r = math.sqrt(x**2 + y**2)
    deg1 = math.degrees(math.atan2(y, x))
    result = three_dof(r, z, l1, l2, l3)
    if result is None:
        return None
    deg2, deg3, deg4 = result[1]  # Choose elbow-up configuration
    return deg1, deg2, deg3, deg4

if __name__ == "__main__": 
    x, y, z = 10, 8, 8
    l1, l2, l3 = 7, 10, 3
    result = inverse_kinematics(x, y, z, l1, l2, l3)
    if result:
        deg1, deg2, deg3, deg4 = result
        print(f"Deg1 (Base): {deg1:.2f}°, Deg2: {deg2:.2f}°, Deg3: {deg3:.2f}°, Deg4: {deg4:.2f}°")
    else:
        print("Target unreachable")