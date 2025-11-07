import math
import matplotlib.pyplot as plt
from ik2 import inverse_kinematics

def visualize_arm_3d(deg1, deg2, deg3, deg4, l1, l2, l3, x, y, z):
    """
    Visualize the 4-DOF arm in 3D with base height l0.
    
    Parameters:
    - deg1, deg2, deg3, deg4: Joint angles in degrees.
    - l1, l2, l3, l0: Link lengths and base height.
    - x, y, z: Target end-effector position.
    """
    # Convert to radians
    theta1 = math.radians(deg1)
    theta2 = math.radians(deg2)
    theta3 = math.radians(deg3)
    theta4 = math.radians(deg4)
    
    # Local positions in the plane (before global rotation), starting from z=l0
    joint1 = [0, 0, 0]
    joint2 = [l1 * math.cos(theta2), 0, l1 * math.sin(theta2)]
    joint3 = [joint2[0] + l2 * math.cos(theta2 + theta3), 0, joint2[2] + l2 * math.sin(theta2 + theta3)]
    end = [joint3[0] + l3 * math.cos(theta2 + theta3 + theta4), 0, joint3[2] + l3 * math.sin(theta2 + theta3 + theta4)]
    
    # Apply global rotation by theta1 around Z-axis
    def rotate_z(point, angle):
        x_new = point[0] * math.cos(angle) - point[1] * math.sin(angle)
        y_new = point[0] * math.sin(angle) + point[1] * math.cos(angle)
        return [x_new, y_new, point[2]]
    
    joint1_rot = rotate_z(joint1, theta1)
    joint2_rot = rotate_z(joint2, theta1)
    joint3_rot = rotate_z(joint3, theta1)
    end_rot = rotate_z(end, theta1)
    
    # Plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Links
    ax.plot([joint1_rot[0], joint2_rot[0]], [joint1_rot[1], joint2_rot[1]], [joint1_rot[2], joint2_rot[2]], 'b-', linewidth=4, label='Link 1')
    ax.plot([joint2_rot[0], joint3_rot[0]], [joint2_rot[1], joint3_rot[1]], [joint2_rot[2], joint3_rot[2]], 'g-', linewidth=4, label='Link 2')
    ax.plot([joint3_rot[0], end_rot[0]], [joint3_rot[1], end_rot[1]], [joint3_rot[2], end_rot[2]], 'r-', linewidth=4, label='Link 3')
    
    # Joints
    joints = [joint1_rot, joint2_rot, joint3_rot, end_rot]
    for j in joints:
        ax.scatter(j[0], j[1], j[2], c='black', s=50)
    
    # Target
    ax.scatter(x, y, z, c='red', s=100, marker='o', label='Target')
    
    # Labels and settings
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('4-DOF Robot Arm IK Visualization with Base Height l0')
    ax.legend()
    ax.set_box_aspect([1,1,1])  # Equal aspect ratio
    plt.show()

# Example usage
if __name__ == "__main__":
    x, y, z = 5, 7, 12
    l1, l2, l3 = 10, 10, 3
    result = inverse_kinematics(x, y, z, l1, l2, l3)
    if result:
        deg1, deg2, deg3, deg4 = result
        print(f"Deg1 (Base): {deg1:.2f}°, Deg2: {deg2:.2f}°, Deg3: {deg3:.2f}°, Deg4: {deg4:.2f}°")
        visualize_arm_3d(deg1, deg2, deg3, deg4, l1, l2, l3, x, y, z)
    else:
        print("Target unreachable")