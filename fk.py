import numpy as np
import matplotlib.pyplot as plt


def homogenous_calc(q1, q2, l1, l2):
    t1 = np.deg2rad(q1)
    t2 = np.deg2rad(q2)
    t12 = t1 + t2

    H = np.array([
        [np.cos(t12), -np.sin(t12), l1 * np.cos(t1) + l2 * np.cos(t12)],
        [np.sin(t12),  np.cos(t12), l1 * np.sin(t1) + l2 * np.sin(t12)],
        [0,            0,           1]
    ])
    return H


def forward_kinematics(q1, q2, l1, l2, x0, y0):
    H = homogenous_calc(q1, q2, l1, l2)
    end_effector = H @ np.array([x0, y0, 1])
    return end_effector[0], end_effector[1]


def run_fk():
    print("\n--- Forward Kinematics ---")
    l1 = float(input("Femur: "))
    l2 = float(input("Tibia: "))
    x0, y0 = map(float, input("Coxa base position (x, y): ").split(","))
    q1 = float(input("Angle q1 (deg): "))
    q2 = float(input("Angle q2 (deg): "))

    t1 = np.deg2rad(q1)
    joint_x = x0 + l1 * np.cos(t1)
    joint_y = y0 + l1 * np.sin(t1)
    x_end, y_end = forward_kinematics(q1, q2, l1, l2, x0, y0)

    print(f"\nEnd effector: ({x_end:.2f}, {y_end:.2f})")

    # Plot
    plt.figure(figsize=(6, 6))
    plt.plot([x0, joint_x], [y0, joint_y], 'b-o', label='Femur')
    plt.plot([joint_x, x_end], [joint_y, y_end], 'r-o', label='Tibia')
    plt.scatter([x0, x_end], [y0, y_end], c=['k', 'g'])
    plt.text(x0, y0, 'Coxa')
    plt.text(x_end, y_end, 'End effector')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title("Forward Kinematics")
    plt.show()